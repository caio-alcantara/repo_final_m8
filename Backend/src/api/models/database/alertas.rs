use std::fmt::Display;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::{FromRow, Type};
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Alertas {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub tour_id: Option<i32>,
    pub origem: Option<String>,
    pub nivel: Nivel,
    pub mensagem: Option<String>,
    pub resolvido_em: Option<DateTime<Utc>>,
    #[serde(skip_deserializing)]
    pub autor_usuario_id: Option<i32>,
    #[serde(default = "Utc::now", skip_deserializing)]
    pub criado_em: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema, Type)]
#[serde(rename_all = "lowercase")]
#[derive(Default)]
pub enum Nivel {
    Baixo,
    #[default]
    Medio,
    Alto,
}

impl From<String> for Nivel {
    fn from(s: String) -> Self {
        match s.as_str() {
            "baixo" => Self::Baixo,
            "medio" => Self::Medio,
            "alto" => Self::Alto,
            _ => Self::Medio,
        }
    }
}

impl From<Nivel> for String {
    fn from(val: Nivel) -> Self {
        match val {
            Nivel::Baixo => "baixo".to_string(),
            Nivel::Medio => "medio".to_string(),
            Nivel::Alto => "alto".to_string(),
        }
    }
}

impl Display for Nivel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Nivel::Baixo => "baixo",
            Nivel::Medio => "medio",
            Nivel::Alto => "alto",
        };
        write!(f, "{}", s)
    }
}
