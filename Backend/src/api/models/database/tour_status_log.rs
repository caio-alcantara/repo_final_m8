use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::FromRow;
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct TourStatusLog {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub tour_id: i32,
    pub status: String,
    pub atualizado_em: Option<DateTime<Utc>>,
    pub observacoes: Option<String>,
    #[serde(skip_deserializing)]
    pub atualizado_por: Option<i32>,
}
