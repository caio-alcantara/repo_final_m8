use std::path::PathBuf;

use clap::Parser;
use log::LevelFilter;

#[derive(Parser, Debug)]
#[command(name = "m08-backend")]
pub struct Args {
    /// Path to the configuration file
    #[arg(short, long, value_name = "FILE", default_value = "config.toml")]
    pub config: PathBuf,

    /// Sets the logger's verbosity level
    #[arg(short, long, value_name = "VERBOSITY", default_value_t = LevelFilter::Info)]
    pub verbosity: LevelFilter,

    /// Runs in production mode
    #[arg(long, num_args = 0..=1, value_parser = clap::value_parser!(bool), default_missing_value = "true", default_value_t = !cfg!(debug_assertions))]
    pub production: bool,
}
