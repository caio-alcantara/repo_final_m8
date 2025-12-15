use std::fmt::Display;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::{FromRow, Type};
use utoipa::ToSchema;

#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Pergunta {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub tour_id: Option<i32>,
    pub checkpoint_id: i32,
    pub question_topic: Option<String>,
    pub texto: String,
    pub estado: PerguntaEstado,
    pub liberado_em: Option<DateTime<Utc>>,
    pub respondido_em: Option<DateTime<Utc>>,
    #[serde(default = "Utc::now", skip_deserializing)]
    pub criado_em: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema, Type)]
#[serde(rename_all = "lowercase")]
#[derive(Default)]
pub enum PerguntaEstado {
    #[default]
    Queued,
    Answerable,
    Answered,
    Discarded,
}

impl From<String> for PerguntaEstado {
    fn from(s: String) -> Self {
        match s.as_str() {
            "queued" => Self::Queued,
            "answerable" => Self::Answerable,
            "answered" => Self::Answered,
            "discarded" => Self::Discarded,
            _ => Self::Queued,
        }
    }
}

impl Display for PerguntaEstado {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            PerguntaEstado::Queued => "queued".to_string(),
            PerguntaEstado::Answerable => "answerable".to_string(),
            PerguntaEstado::Answered => "answered".to_string(),
            PerguntaEstado::Discarded => "discarded".to_string(),
        };

        write!(f, "{}", s)
    }
}
