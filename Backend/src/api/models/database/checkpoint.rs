use std::fmt::Display;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::{FromRow, Type};
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Checkpoint {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub tour_id: i32,
    pub tipo: CheckpointTipo,
    pub ordem: i32,
    pub status: CheckpointStatus,
    pub inicio_previsto: Option<DateTime<Utc>>,
    pub inicio_real: Option<DateTime<Utc>>,
    pub fim_real: Option<DateTime<Utc>>,
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema, Type)]
#[serde(rename_all = "snake_case")]
pub enum CheckpointTipo {
    Recepcao,
    Auditorio,
    Atelie,
    Casinhas,
    DogHouse,
}

impl From<String> for CheckpointTipo {
    fn from(s: String) -> Self {
        match s.as_str() {
            "recepcao" => Self::Recepcao,
            "auditorio" => Self::Auditorio,
            "atelie" => Self::Atelie,
            "casinhas" => Self::Casinhas,
            "dog_house" => Self::DogHouse,
            _ => Self::Recepcao, // default case
        }
    }
}

impl Display for CheckpointTipo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            CheckpointTipo::Recepcao => "recepcao",
            CheckpointTipo::Auditorio => "auditorio",
            CheckpointTipo::Atelie => "atelie",
            CheckpointTipo::Casinhas => "casinhas",
            CheckpointTipo::DogHouse => "dog_house",
        };
        write!(f, "{}", s)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema, Type)]
#[serde(rename_all = "lowercase")]
#[derive(Default)]
pub enum CheckpointStatus {
    #[default]
    Pending,
    Running,
    Finished,
    Skipped,
}

impl From<String> for CheckpointStatus {
    fn from(s: String) -> Self {
        match s.as_str() {
            "pending" => Self::Pending,
            "running" => Self::Running,
            "finished" => Self::Finished,
            "skipped" => Self::Skipped,
            _ => Self::Pending, // default case
        }
    }
}

impl Display for CheckpointStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            CheckpointStatus::Pending => "pending",
            CheckpointStatus::Running => "running",
            CheckpointStatus::Finished => "finished",
            CheckpointStatus::Skipped => "skipped",
        };
        write!(f, "{}", s)
    }
}
