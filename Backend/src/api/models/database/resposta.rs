use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::FromRow;
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Resposta {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub pergunta_id: i32,
    pub respondido_por_tipo: String,
    pub respondido_por_usuario: Option<i32>,
    pub texto: String,
    #[serde(default = "Utc::now", skip_deserializing)]
    pub criado_em: DateTime<Utc>,
}
