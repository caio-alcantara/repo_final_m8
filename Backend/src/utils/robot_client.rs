use chrono::{DateTime, Utc};
use futures_util::{SinkExt, StreamExt, stream::{SplitSink, SplitStream}};
use log::{debug, error, info, warn};
use sqlx::PgPool;
use std::sync::Arc;
use tokio::sync::{broadcast, Mutex};
use tokio_tungstenite::{connect_async, tungstenite::protocol::Message, WebSocketStream, MaybeTlsStream};
use tokio::net::TcpStream;

use super::websocket::{RobotEvent, RobotCommand, FrontendEvent};

type WsWriter = SplitSink<WebSocketStream<MaybeTlsStream<TcpStream>>, Message>;
type WsReader = SplitStream<WebSocketStream<MaybeTlsStream<TcpStream>>>;

pub struct RobotClient {
    robot_ip: String,
    writer: Arc<Mutex<Option<WsWriter>>>,
    frontend_tx: broadcast::Sender<FrontendEvent>,
    db_pool: PgPool,
}

impl RobotClient {
    pub fn new(
        robot_ip: String,
        frontend_tx: broadcast::Sender<FrontendEvent>,
        db_pool: PgPool,
    ) -> Self {
        Self {
            robot_ip,
            writer: Arc::new(Mutex::new(None)),
            frontend_tx,
            db_pool,
        }
    }
    
    /// Conecta ao WebSocket do robô e inicia o loop de recebimento
    pub async fn connect(&self) -> eyre::Result<()> {
        let url = format!("ws://{}:8080/ws", self.robot_ip);
        info!("🔌 Conectando ao robô em: {}", url);
        
        let (ws_stream, _) = connect_async(&url).await?;
        info!("✅ Conectado ao robô!");
        
        let (write, read) = ws_stream.split();
        
        // Armazenar o writer para enviar comandos depois
        *self.writer.lock().await = Some(write);
        
        // Iniciar task de recebimento de eventos
        self.spawn_reader_task(read).await;
        
        Ok(())
    }
    
    /// Inicia uma task assíncrona para processar eventos do robô
    async fn spawn_reader_task(&self, mut read: WsReader) {
        let frontend_tx = self.frontend_tx.clone();
        let db_pool = self.db_pool.clone();
        
        tokio::spawn(async move {
            info!("📡 Iniciando listener de eventos do robô");
            
            while let Some(msg) = read.next().await {
                match msg {
                    Ok(Message::Text(text)) => {
                        debug!("📥 Evento do robô: {}", text);
                        
                        match serde_json::from_str::<RobotEvent>(&text) {
                            Ok(event) => {
                                if let Err(e) = Self::handle_robot_event(
                                    event,
                                    &frontend_tx,
                                    &db_pool,
                                ).await {
                                    error!("❌ Erro ao processar evento do robô: {}", e);
                                }
                            }
                            Err(e) => error!("❌ Erro ao parsear evento do robô: {}", e),
                        }
                    }
                    Ok(Message::Close(_)) => {
                        warn!("🔌 Conexão com robô fechada");
                        break;
                    }
                    Err(e) => {
                        error!("❌ Erro no WebSocket do robô: {}", e);
                        break;
                    }
                    _ => {}
                }
            }
            
            warn!("⚠️ Listener de eventos do robô encerrado");
        });
    }
    
    /// Processa um evento recebido do robô
    async fn handle_robot_event(
        event: RobotEvent,
        frontend_tx: &broadcast::Sender<FrontendEvent>,
        db_pool: &PgPool,
    ) -> eyre::Result<()> {
        match event {
            RobotEvent::Connected { status } => {
                info!("🤖 Status inicial do robô: robot_connected={}", status.robot_connected);
                
                // Notificar frontend sobre o status do robô
                let _ = frontend_tx.send(FrontendEvent::RobotStatus {
                    robot_connected: status.robot_connected,
                    is_running: status.is_running,
                    current_checkpoint: status.current_checkpoint,
                });
                
                if !status.robot_connected {
                    warn!("⚠️ Robô OFFLINE - comandos não serão executados");
                }
            }
            
            RobotEvent::CheckpointStarted { tipo, ordem, status, inicio_real } => {
                info!("▶️ Checkpoint INICIADO: {} (ordem: {})", tipo, ordem);
                
                // Parsear timestamp
                let inicio_dt = DateTime::parse_from_rfc3339(&inicio_real)?
                    .with_timezone(&Utc);
                
                // Atualizar banco de dados
                sqlx::query(
                    "UPDATE checkpoints 
                     SET status = $1, inicio_real = $2 
                     WHERE tipo = $3 AND ordem = $4"
                )
                .bind(&status)
                .bind(inicio_dt)
                .bind(&tipo)
                .bind(ordem)
                .execute(db_pool)
                .await?;
                
                debug!("💾 Checkpoint atualizado no banco: {} -> {}", tipo, status);
                
                // Enviar para frontend
                let _ = frontend_tx.send(FrontendEvent::CheckpointStarted {
                    tipo,
                    ordem,
                    status,
                    inicio_real: inicio_dt,
                });
            }
            
            RobotEvent::CheckpointCompleted { tipo, ordem, status, inicio_real, fim_real } => {
                info!("✅ Checkpoint CONCLUÍDO: {} (status: {})", tipo, status);
                
                // Parsear timestamps
                let fim_dt = DateTime::parse_from_rfc3339(&fim_real)?
                    .with_timezone(&Utc);
                let inicio_dt = DateTime::parse_from_rfc3339(&inicio_real)?
                    .with_timezone(&Utc);
                
                // Atualizar banco de dados
                sqlx::query(
                    "UPDATE checkpoints 
                     SET status = $1, fim_real = $2 
                     WHERE tipo = $3 AND ordem = $4"
                )
                .bind(&status)
                .bind(fim_dt)
                .bind(&tipo)
                .bind(ordem)
                .execute(db_pool)
                .await?;
                
                debug!("💾 Checkpoint finalizado no banco: {} -> {}", tipo, status);
                
                // Enviar para frontend
                let _ = frontend_tx.send(FrontendEvent::CheckpointCompleted {
                    tipo,
                    ordem,
                    status,
                    inicio_real: inicio_dt,
                    fim_real: fim_dt,
                });
            }
            
            RobotEvent::EmergencyStop { tipo, ordem } => {
                warn!("🛑 PARADA DE EMERGÊNCIA! checkpoint: {:?}", tipo);
                
                // Se tinha um checkpoint em execução, marcar como skipped
                if let (Some(t), Some(o)) = (tipo.clone(), ordem) {
                    sqlx::query(
                        "UPDATE checkpoints 
                         SET status = $1 
                         WHERE tipo = $2 AND ordem = $3 AND status = 'running'"
                    )
                    .bind("skipped")
                    .bind(&t)
                    .bind(o)
                    .execute(db_pool)
                    .await?;
                    
                    debug!("💾 Checkpoint marcado como skipped: {}", t);
                }
                
                // Enviar para frontend
                let _ = frontend_tx.send(FrontendEvent::EmergencyStop { tipo, ordem });
            }
            
            RobotEvent::RobotConnected { status } => {
                info!("🟢 Robô ficou ONLINE! Status: {}", status);
                
                // Notificar frontend
                let _ = frontend_tx.send(FrontendEvent::RobotStatus {
                    robot_connected: true,
                    is_running: false,
                    current_checkpoint: None,
                });
            }
        }
        
        Ok(())
    }
    
    /// Envia um comando para o robô
    pub async fn send_command(&self, command: RobotCommand) -> eyre::Result<()> {
        let mut writer_guard = self.writer.lock().await;
        
        if let Some(writer) = writer_guard.as_mut() {
            let json = serde_json::to_string(&command)?;
            writer.send(Message::Text(json)).await?;
            debug!("📤 Comando enviado ao robô: {:?}", command.action);
            Ok(())
        } else {
            Err(eyre::eyre!("Não conectado ao robô"))
        }
    }
    
    /// Envia comando PLAY
    pub async fn play(&self) -> eyre::Result<()> {
        self.send_command(RobotCommand::play()).await
    }
    
    /// Envia comando STOP
    pub async fn stop(&self) -> eyre::Result<()> {
        self.send_command(RobotCommand::stop()).await
    }
    
    /// Solicita status atual do robô
    pub async fn get_status(&self) -> eyre::Result<()> {
        self.send_command(RobotCommand::get_status()).await
    }
}
