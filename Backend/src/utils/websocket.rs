use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

/// Eventos enviados pelo robô via WebSocket
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "event")]
pub enum RobotEvent {
    #[serde(rename = "connected")]
    Connected { status: RobotStatus },
    
    #[serde(rename = "checkpoint_started")]
    CheckpointStarted {
        tipo: String,
        ordem: i32,
        status: String,
        inicio_real: String,
    },
    
    #[serde(rename = "checkpoint_completed")]
    CheckpointCompleted {
        tipo: String,
        ordem: i32,
        status: String,
        inicio_real: String,
        fim_real: String,
    },
    
    #[serde(rename = "emergency_stop")]
    EmergencyStop {
        tipo: Option<String>,
        ordem: Option<i32>,
    },
    
    #[serde(rename = "robot_connected")]
    RobotConnected { status: String },
}

/// Status do robô recebido na conexão inicial
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotStatus {
    pub robot_connected: bool,
    pub is_running: bool,
    pub current_checkpoint: Option<String>,
    pub current_checkpoint_index: i32,
    pub total_checkpoints: i32,
    #[serde(default)]
    pub checkpoint_names: Vec<String>,
    #[serde(default)]
    pub queue_size: i32,
    #[serde(default)]
    pub is_stopped: bool,
}

/// Comandos que podem ser enviados ao robô
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotCommand {
    pub action: String, // "play", "stop", "get_status"
}

impl RobotCommand {
    pub fn play() -> Self {
        Self { action: "play".to_string() }
    }
    
    pub fn stop() -> Self {
        Self { action: "stop".to_string() }
    }
    
    pub fn get_status() -> Self {
        Self { action: "get_status".to_string() }
    }
}

/// Resposta do robô a comandos
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotCommandResponse {
    pub status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub checkpoint: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
}

/// Eventos enviados para o frontend
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "event")]
pub enum FrontendEvent {
    #[serde(rename = "robot_status")]
    RobotStatus {
        robot_connected: bool,
        is_running: bool,
        current_checkpoint: Option<String>,
    },
    
    #[serde(rename = "checkpoint_started")]
    CheckpointStarted {
        tipo: String,
        ordem: i32,
        status: String,
        inicio_real: DateTime<Utc>,
    },
    
    #[serde(rename = "checkpoint_completed")]
    CheckpointCompleted {
        tipo: String,
        ordem: i32,
        status: String,
        inicio_real: DateTime<Utc>,
        fim_real: DateTime<Utc>,
    },
    
    #[serde(rename = "emergency_stop")]
    EmergencyStop {
        tipo: Option<String>,
        ordem: Option<i32>,
    },
    
    #[serde(rename = "error")]
    Error {
        message: String,
    },
}

/// Comandos recebidos do frontend
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "action")]
pub enum FrontendCommand {
    #[serde(rename = "play")]
    Play,
    
    #[serde(rename = "stop")]
    Stop,
    
    #[serde(rename = "get_status")]
    GetStatus,
}
