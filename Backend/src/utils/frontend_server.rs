use actix_web::{web, HttpRequest, HttpResponse, Error};
use actix_ws::Message as WsMessage;
use futures_util::StreamExt;
use log::{debug, error, info};

use super::websocket::{FrontendEvent, FrontendCommand};
use crate::utils::robot_client::RobotClient;
use crate::api::models::AppState;

/// Handler para conexões WebSocket do frontend
pub async fn ws_frontend_handler(
    req: HttpRequest,
    stream: web::Payload,
    app_state: web::Data<AppState>,
) -> Result<HttpResponse, Error> {
    let (response, mut session, mut msg_stream) = actix_ws::handle(&req, stream)?;
    
    info!("🌐 Nova conexão WebSocket do frontend");
    
    // Criar um receiver para eventos do robô
    let mut frontend_rx = app_state.frontend_tx.subscribe();
    let robot_client = app_state.robot_client.clone();
    
    // Task para enviar eventos do robô para o frontend
    actix_web::rt::spawn(async move {
        loop {
            tokio::select! {
                // Receber eventos do robô e enviar para o frontend
                Ok(event) = frontend_rx.recv() => {
                    match serde_json::to_string(&event) {
                        Ok(json) => {
                            debug!("📤 Enviando evento para frontend: {}", json);
                            if let Err(e) = session.text(json).await {
                                error!("❌ Erro ao enviar para frontend: {}", e);
                                break;
                            }
                        }
                        Err(e) => error!("❌ Erro ao serializar evento: {}", e),
                    }
                }
                
                // Receber comandos do frontend
                Some(Ok(msg)) = msg_stream.next() => {
                    match msg {
                        WsMessage::Text(text) => {
                            debug!("📥 Comando do frontend: {}", text);
                            
                            match serde_json::from_str::<FrontendCommand>(&text) {
                                Ok(cmd) => {
                                    if let Err(e) = handle_frontend_command(cmd, &robot_client).await {
                                        error!("❌ Erro ao processar comando: {}", e);
                                        
                                        let error_event = FrontendEvent::Error {
                                            message: e.to_string(),
                                        };
                                        
                                        if let Ok(json) = serde_json::to_string(&error_event) {
                                            let _ = session.text(json).await;
                                        }
                                    }
                                }
                                Err(e) => {
                                    error!("❌ Erro ao parsear comando: {}", e);
                                }
                            }
                        }
                        WsMessage::Close(_) => {
                            info!("👋 Frontend desconectou");
                            break;
                        }
                        WsMessage::Ping(bytes) => {
                            let _ = session.pong(&bytes).await;
                        }
                        _ => {}
                    }
                }
                
                else => break,
            }
        }
        
        let _ = session.close(None).await;
        info!("🔌 Conexão com frontend encerrada");
    });
    
    Ok(response)
}

/// Processa comandos recebidos do frontend e encaminha para o robô
async fn handle_frontend_command(
    command: FrontendCommand,
    robot_client: &RobotClient,
) -> eyre::Result<()> {
    match command {
        FrontendCommand::Play => {
            info!("▶️ Frontend solicitou PLAY");
            robot_client.play().await?;
        }
        FrontendCommand::Stop => {
            info!("⏸️ Frontend solicitou STOP");
            robot_client.stop().await?;
        }
        FrontendCommand::GetStatus => {
            info!("📊 Frontend solicitou STATUS");
            robot_client.get_status().await?;
        }
    }
    
    Ok(())
}
