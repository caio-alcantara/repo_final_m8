use log::{debug, info};
use serde_json::Value;
use sqlx::PgPool;
use std::sync::Arc;
use tokio::sync::{broadcast, Mutex};
use tokio_tungstenite::{tungstenite::protocol::Message, WebSocketStream, MaybeTlsStream};
use tokio::net::TcpStream;
use futures_util::stream::{SplitSink, SplitStream};

use super::websocket::{RobotEvent, RobotCommand, FrontendEvent};

#[allow(dead_code)]
type WsWriter = SplitSink<WebSocketStream<MaybeTlsStream<TcpStream>>, Message>;
#[allow(dead_code)]
type WsReader = SplitStream<WebSocketStream<MaybeTlsStream<TcpStream>>>;

pub struct RobotClient {
    #[allow(dead_code)]
    robot_ip: String,
    #[allow(dead_code)]
    writer: Arc<Mutex<Option<WsWriter>>>,
    #[allow(dead_code)]
    frontend_tx: broadcast::Sender<FrontendEvent>,
    #[allow(dead_code)]
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
    
    pub async fn connect(&self) -> eyre::Result<()> {
        info!("🔌 Mock: Ignorando conexão real para teste");
        Ok(())
    }
    
    #[allow(dead_code)]
    async fn spawn_reader_task(&self, _read: WsReader) { }

    #[allow(dead_code)]
    async fn handle_robot_event(
        _event: RobotEvent,
        _frontend_tx: &broadcast::Sender<FrontendEvent>,
        _db_pool: &PgPool,
    ) -> eyre::Result<()> {
        Ok(())
    }
    
    pub async fn send_command(&self, _command: RobotCommand) -> eyre::Result<()> {
        Ok(())
    }

    pub async fn play(&self) -> eyre::Result<()> {
        Ok(())
    }

    pub async fn stop(&self) -> eyre::Result<()> {
        Ok(())
    }

    pub async fn get_status(&self) -> eyre::Result<()> {
        Ok(())
    }

    pub async fn get_status_http(&self) -> eyre::Result<String> {
        Ok(r#"{"robot_connected": true, "is_running": false, "current_checkpoint": "entrada"}"#.to_string())
        
        // self.http_get("/status").await
    }

    pub async fn start_next_http(&self) -> eyre::Result<String> {
        Ok(r#"{"success": true, "message": "Movimentação para o próximo ponto iniciada"}"#.to_string())
        
        // self.http_post("/start_next").await
    }

    pub async fn cancel_http(&self) -> eyre::Result<String> {
        Ok(r#"{"success": true, "message": "Cancelado"}"#.to_string())
        
        // self.http_post("/cancel").await
    }

    pub async fn reset_counter_http(&self) -> eyre::Result<String> {
        Ok(r#"{"success": true, "message": "Resetado"}"#.to_string())
        
        // self.http_post("/reset").await
    }

    pub async fn add_checkpoint_http(&self, _payload: &Value) -> eyre::Result<String> {
        Ok(r#"{"success": true, "message": "Adicionado"}"#.to_string())
        
        // self.http_post_json("/add_checkpoint", payload).await
    }


    /// Faz uma chamada HTTP GET ao robô
    #[allow(dead_code)]
    async fn http_get(&self, endpoint: &str) -> eyre::Result<String> {
        let url = format!("http://{}:8000{}", self.robot_ip, endpoint);
        debug!("🌐 Fazendo GET para: {}", url);
        
        let client = reqwest::Client::new();
        let response = client
            .get(&url)
            .send()
            .await?;
        
        let status = response.status();
        let body = response.text().await?;
        
        if status.is_success() {
            Ok(body)
        } else {
            Err(eyre::eyre!("Erro HTTP {}: {}", status, body))
        }
    }
    
    /// Faz uma chamada HTTP POST sem corpo para o robô
    #[allow(dead_code)]
    async fn http_post(&self, endpoint: &str) -> eyre::Result<String> {
        let url = format!("http://{}:8000{}", self.robot_ip, endpoint);
        debug!("🌐 Fazendo POST para: {}", url);
        
        let client = reqwest::Client::new();
        let response = client
            .post(&url)
            .send()
            .await?;
        
        let status = response.status();
        let body = response.text().await?;
        
        if status.is_success() {
            Ok(body)
        } else {
            Err(eyre::eyre!("Erro HTTP {}: {}", status, body))
        }
    }
    
    /// Faz uma chamada HTTP POST com JSON para o robô
    #[allow(dead_code)]
    async fn http_post_json(&self, endpoint: &str, payload: &Value) -> eyre::Result<String> {
        let url = format!("http://{}:8000{}", self.robot_ip, endpoint);
        debug!("🌐 Fazendo POST JSON para: {}", url);
        
        let client = reqwest::Client::new();
        let response = client
            .post(&url)
            .json(payload)
            .send()
            .await?;
        
        let status = response.status();
        let body = response.text().await?;
        
        if status.is_success() {
            Ok(body)
        } else {
            Err(eyre::eyre!("Erro HTTP {}: {}", status, body))
        }
    }
}