use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sqlx::prelude::FromRow;
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Notificacoes {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub usuario_id: Option<i32>,
    pub titulo: Option<String>,
    pub corpo: Option<String>,
    pub payload_json: Option<Value>,
    pub lido: bool,
    #[serde(default = "Utc::now", skip_deserializing)]
    pub criado_em: DateTime<Utc>,
}
