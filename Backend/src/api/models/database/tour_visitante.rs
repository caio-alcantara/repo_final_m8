use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::FromRow;
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct TourVisitante {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub tour_id: i32,
    pub visitante_id: i32,
    #[serde(skip_deserializing)]
    pub adicionado_por: Option<i32>,
    #[serde(default = "Utc::now", skip_deserializing)]
    pub adicionado_em: DateTime<Utc>,
}
