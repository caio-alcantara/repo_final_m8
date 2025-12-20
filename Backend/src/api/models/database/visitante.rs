use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::prelude::FromRow;
use sqlx::Type;  
use utoipa::ToSchema;
use std::fmt::Display;


#[derive(Debug, Clone, Serialize, Deserialize, FromRow, ToSchema)]
pub struct Visitante {
    #[serde(skip_deserializing)]
    pub id: Option<i32>,
    pub nome: Option<String>,    
    pub email: Option<String>,   
    pub telefone: Option<String>,
    #[serde(default = "Utc::now", skip_deserializing)]
    pub criado_em: DateTime<Utc>,
    pub cidade: Option<String>,
    pub estado: Option<String>,
    pub cpf: Option<String>,
    pub perfil: Perfil,
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema, Type)]
#[sqlx(type_name = "text")]
#[sqlx(rename_all = "snake_case")]
#[serde(rename_all = "lowercase")]
#[derive(Default)]
pub enum Perfil {
    #[default]
    Student,
    Executive,
}


impl From<String> for Perfil {
    fn from(s: String) -> Self {
        match s.as_str() {
            "student" => Self::Student,
            "executive" => Self::Executive,
            _ => Self::Executive, // default case
        }
    }
}

impl From<Perfil> for String {
    fn from(val: Perfil) -> Self {
        match val {
            Perfil::Executive => "executive".to_string(),
            Perfil::Student => "student".to_string(),
        }
    }
}

impl Display for Perfil {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let status_str = match self {
            Perfil::Executive => "Executive",
            Perfil::Student => "Student",
        };
        write!(f, "{}", status_str)
    }
}