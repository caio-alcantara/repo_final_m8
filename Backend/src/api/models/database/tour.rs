use std::fmt::Display;

use chrono::{DateTime, NaiveDate, NaiveTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::{FromRow, Type};
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Tour {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub codigo: String,
    pub titulo: Option<String>,
    pub data_local: NaiveDate,
    pub hora_inicio_prevista: Option<NaiveTime>,
    pub hora_fim_prevista: Option<NaiveTime>,
    pub inicio_real: Option<DateTime<Utc>>,
    pub fim_real: Option<DateTime<Utc>>,
    pub status: TourStatus,
    pub robo_id: i32,
    pub responsavel_id: Option<i32>,
    #[serde(skip_deserializing)]
    pub criado_por: Option<i32>,
    #[serde(default = "Utc::now", skip_deserializing)]
    pub criado_em: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema, Type)]
#[sqlx(type_name = "text")]  // <-- ADICIONE ESTA LINHA
#[sqlx(rename_all = "snake_case")]  // <-- ADICIONE ESTA LINHA
#[serde(rename_all = "lowercase")]
#[derive(Default)]
pub enum TourStatus {
    #[default]
    Scheduled,
    #[serde(rename = "in_progress")]
    #[sqlx(rename = "in_progress")]  // <-- ADICIONE ESTA LINHA
    InProgress,
    Completed,
    Cancelled,
}

impl From<String> for TourStatus {
    fn from(s: String) -> Self {
        match s.as_str() {
            "scheduled" => Self::Scheduled,
            "in_progress" => Self::InProgress,
            "completed" => Self::Completed,
            "cancelled" => Self::Cancelled,
            _ => Self::Scheduled, // default case
        }
    }
}

impl From<TourStatus> for String {
    fn from(val: TourStatus) -> Self {
        match val {
            TourStatus::Scheduled => "scheduled".to_string(),
            TourStatus::InProgress => "in_progress".to_string(),
            TourStatus::Completed => "completed".to_string(),
            TourStatus::Cancelled => "cancelled".to_string(),
        }
    }
}

impl Display for TourStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let status_str = match self {
            TourStatus::Scheduled => "Scheduled",
            TourStatus::InProgress => "In Progress",
            TourStatus::Completed => "Completed",
            TourStatus::Cancelled => "Cancelled",
        };
        write!(f, "{}", status_str)
    }
}