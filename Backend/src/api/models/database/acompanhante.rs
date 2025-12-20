use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::{FromRow, Type};
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Acompanhante {
    #[serde(skip_deserializing)] 
    pub id: Option<i32>,
    pub nome: Option<String>,
    pub cpf: Option<String>,
    pub visitante_id: Option<i32>,

}

