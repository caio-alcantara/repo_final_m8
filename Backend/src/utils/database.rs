use colored::Colorize;
use eyre::Result;
use log::info;
use sqlx::{PgPool, postgres::{PgPoolOptions, PgConnectOptions}};
use std::str::FromStr;

use crate::utils::config::Config;

pub struct Database {
    pool: PgPool,
}

impl Database {
    pub async fn new(config: Config) -> Result<Self> {
        let connection_string = format!(
            "postgres://{}:{}@{}:{}/{}",
            config.database.username,
            config.database.password,
            config.database.host,
            config.database.port,
            config.database.database
        );
        
        let connect_options = PgConnectOptions::from_str(&connection_string)?
            .statement_cache_capacity(0); // Disable prepared statement cache to avoid conflicts
        
        let pool = PgPoolOptions::new()
            .max_connections(config.database.max_connections)
            .connect_with(connect_options)
            .await?;

        let db_name: (String,) = sqlx::query_as("SELECT current_database()")
            .fetch_one(&pool)
            .await?;

        info!(
            "successfully connected to database at {} with name {}",
            config.database.host.as_str().magenta(),
            db_name.0.magenta()
        );

        Ok(Database { pool })
    }
}

impl AsRef<PgPool> for Database {
    fn as_ref(&self) -> &PgPool {
        &self.pool
    }
}
