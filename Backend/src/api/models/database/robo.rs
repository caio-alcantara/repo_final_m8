use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::FromRow;
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Robo {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub nome: Option<String>,
    pub modelo: Option<String>,
    pub numero_serie: Option<String>,
    pub ativo: Option<bool>,
    #[serde(default = "Utc::now", skip_deserializing)]
    pub criado_em: DateTime<Utc>,
}
