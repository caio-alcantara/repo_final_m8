pub mod database;
pub mod responses;

use actix_web::{Error, HttpResponse, web};
use eyre::Result;
use std::sync::Arc;
use tokio::sync::broadcast;

use crate::utils::{config::Config, database::Database, robot_client::RobotClient, websocket::FrontendEvent};

#[allow(dead_code)]
pub struct AppState {
    pub config: Config,
    pub database: Database,
    pub robot_client: Arc<RobotClient>,
    pub frontend_tx: broadcast::Sender<FrontendEvent>,
}
impl AppState {
    pub async fn new(config: Config) -> Result<Self> {
        let database = Database::new(config.clone()).await?;
        
        // Criar canal de broadcast para eventos do frontend
        let (frontend_tx, _) = broadcast::channel::<FrontendEvent>(100);
        
        // Criar cliente do robô
        let robot_client = Arc::new(RobotClient::new(
            config.robot.ip.clone(),
            frontend_tx.clone(),
            database.as_ref().clone(),
        ));
        
        // Conectar ao robô em background
        let robot_client_clone = robot_client.clone();
        tokio::spawn(async move {
            loop {
                match robot_client_clone.connect().await {
                    Ok(_) => {
                        log::info!("✅ Conectado ao robô com sucesso");
                        break;
                    }
                    Err(e) => {
                        log::warn!("⚠️ Falha ao conectar no robô: {}. Tentando novamente em 5s...", e);
                        tokio::time::sleep(tokio::time::Duration::from_secs(5)).await;
                    }
                }
            }
        });
        
        Ok(AppState {
            database,
            config,
            robot_client,
            frontend_tx,
        })
    }
}

pub type Data = web::Data<AppState>;
pub type Response = Result<HttpResponse, Error>;
