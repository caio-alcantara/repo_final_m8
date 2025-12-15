pub mod v1;

use utoipa_actix_web::service_config::ServiceConfig;

pub fn router(cfg: &mut ServiceConfig) {
    cfg.configure(v1::router);
}
