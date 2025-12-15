mod api;
mod utils;

use clap::Parser;
use colored::Colorize;
use log::{debug, info};

use crate::utils::cli::Args;
use crate::utils::config::{Config, config};
use crate::utils::log::Logger;

#[actix_web::main]
async fn main() -> eyre::Result<()> {
    let args = Args::parse();
    Logger::init(args.verbosity);

    info!(
        "initializing m08-backend in {} mode...",
        match args.production {
            true => "PRODUCTION".red(),
            false => "DEVELOPMENT".magenta(),
        }
    );

    let config: Config = config(args.config)?;
    debug!("loaded configuration:\n{:?}", config);

    api::serve(config, args.production).await?;

    Ok(())
}
