use std::path::PathBuf;
use std::sync::Arc;

use easy_config_store::ConfigStore;
use eyre::Result;
use serde::{Deserialize, Serialize};

pub type Config = Arc<ConfigStore<ConfigInner>>;
pub fn config(path: PathBuf) -> Result<Config> {
    Ok(ConfigStore::<ConfigInner>::read(path, "settings".to_string())?.arc())
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct ConfigInner {
    pub server: ServerConfig,
    pub database: DatabaseConfig,
    pub ml: MLConfig,
    pub robot: RobotConfig,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct ServerConfig {
    pub host: String,
    pub port: u16,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct MLConfig {
    pub ws_endpoint: String,
    #[serde(default = "default_timeout")]
    pub timeout_secs: u64,
}

fn default_postgres_port() -> u16 {
    5432
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct DatabaseConfig {
    pub host: String,
    pub username: String,
    pub password: String,
    pub database: String,

    pub max_connections: u32,

    #[serde(default = "default_postgres_port")]
    pub port: u16,
}

fn default_timeout() -> u64 {
    30
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct RobotConfig {
    pub ip: String,
}

impl Default for ConfigInner {
    fn default() -> Self {
        let cfg = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/config.default.toml",));

        toml::from_str(cfg).unwrap() // should be okay
    }
}
