use actix_web::middleware::{Compress, ErrorHandlers, NormalizePath, TrailingSlash};
use actix_web::web::Data;
use actix_web::{App, HttpServer};
use colored::Colorize;
use log::info;
use utoipa_actix_web::AppExt;
use utoipa_scalar::{Scalar, Servable};

use crate::api::middleware::errors::json_error_middleware;
use crate::api::models::AppState;
use crate::utils::config::Config;
use crate::utils::http_logger::HttpLogger;
use crate::api::routes::v1::broadcast::init_broadcast_server;

pub mod middleware;
pub mod models;
pub mod routes;

pub async fn serve(config: Config, production: bool) -> std::io::Result<()> {
    let server_host = config.server.host.clone();
    let server_port = config.server.port;

    info!(
        "starting server on {}:{}",
        server_host.magenta(),
        server_port.to_string().yellow()
    );

    if !production {
        info!(
            "development API documentation available at {}",
            format!("http://{}:{}/docs", server_host, server_port).magenta()
        );
    }

    

    let ml_endpoint = config.ml.ws_endpoint.clone();
    info!(
        "ML WebSocket endpoint configured: {}",
        ml_endpoint.magenta()
    );

    init_broadcast_server(ml_endpoint.clone());
    
    let data = Data::new(AppState::new(config).await.map_err(|e| {
        log::error!("failed to initialize application state: {}", e);
        std::io::Error::other("application state initialization failed")
    })?);
    
    let ml_endpoint_data = Data::new(ml_endpoint);

    HttpServer::new(move || {
        let (app, api) = App::new()
            .wrap(NormalizePath::new(TrailingSlash::Trim))
            .wrap(Compress::default())
            .wrap(HttpLogger)
            .wrap(ErrorHandlers::new().default_handler(json_error_middleware))
            .into_utoipa_app()
            .app_data(data.clone())
            .app_data(ml_endpoint_data.clone())
            .configure(routes::router)
            .split_for_parts();

        let app = app.service(
            actix_web::web::scope("/ws")
                .route("", actix_web::web::get().to(crate::utils::frontend_server::ws_frontend_handler))
        );

        match production {
            true => app,
            false => app.service(Scalar::with_url("/docs", api)),
        }
    })
    .bind((server_host, server_port))?
    .run()
    .await
}
