use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::FromRow;
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct RastreioRobo {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub tour_id: Option<i32>,
    pub checkpoint_id: Option<i32>,
    pub waypoint: Option<String>,
    pub progresso_pct: Option<f64>,
    #[serde(default = "Utc::now", skip_deserializing)]
    pub criado_em: DateTime<Utc>,
}
