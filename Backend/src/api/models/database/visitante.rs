use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::FromRow;
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Visitante {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub nome: Option<String>,     // Nullable no banco
    pub email: Option<String>,    // Nullable no banco
    pub telefone: Option<String>, // Nullable no banco
    #[serde(default = "Utc::now", skip_deserializing)]
    pub criado_em: DateTime<Utc>,
}
