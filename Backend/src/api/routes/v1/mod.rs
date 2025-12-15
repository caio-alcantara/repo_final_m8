pub mod alertas;
pub mod checkpoint;
pub mod notificacoes;
pub mod pergunta;
pub mod rastreio_robo;
pub mod resposta;
pub mod robo;
pub mod tour;
pub mod tour_status_log;
pub mod tour_visitante;
pub mod visitante;
pub mod audio_ws;
pub mod tour_ws;
mod modelo;
mod text_ws;
pub mod broadcast;

use utoipa_actix_web::scope;
use utoipa_actix_web::service_config::ServiceConfig;

pub fn router(cfg: &mut ServiceConfig) {
    cfg.service(
        scope("/v1")
            .configure(tour::router)
            .configure(visitante::router)
            .configure(tour_visitante::router)
            .configure(tour_status_log::router)
            .configure(resposta::router)
            .configure(pergunta::router)
            .configure(checkpoint::router)
            .configure(robo::router)
            .configure(rastreio_robo::router)
            .configure(alertas::router)
            .configure(notificacoes::router)
            .configure(audio_ws::router)
            .configure(tour_ws::router)
            .configure(text_ws::router)
            .configure(modelo::router),
    );
}
