use actix_web::{
    HttpResponse,
    error::{ErrorInternalServerError, ErrorNotFound},
    post, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};
use serde::{Deserialize, Serialize};
use chrono::Utc;
use crate::api::models::{
    Data, Response,
    database::{Pergunta, Resposta, PerguntaEstado},
    responses::{GenericResponse, SingleItemResponse},
};

pub const SLUG: &str = "/modelo";

pub fn router(cfg: &mut ServiceConfig) {
    cfg.service(
        scope(self::SLUG)
            .service(process_question),
    );
}

#[derive(Debug, Serialize)]
struct ModeloRequest {
    message: String,
}

#[derive(Debug, Deserialize)]
struct ModeloResponse {
    status: u16,
    response: String,
}

#[utoipa::path(
    request_body = Pergunta,
    responses(
        (status = OK, description = "Question processed and answer generated", body = SingleItemResponse<Resposta>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn process_question(data: Data, body: web::Json<Pergunta>) -> Response {
    let pergunta = sqlx::query_as!(
        Pergunta,
        "INSERT INTO perguntas (
            tour_id,
            checkpoint_id,
            question_topic,
            texto,
            estado,
            liberado_em,
            respondido_em,
            criado_em
        )
        VALUES (
            $1,
            $2,
            $3,
            $4,
            $5,
            $6,
            $7,
            $8
        )
        RETURNING *",
        body.tour_id,
        body.checkpoint_id,
        body.question_topic,
        body.texto,
        PerguntaEstado::Queued.to_string(),
        body.liberado_em,
        None::<chrono::DateTime<Utc>>,
        Utc::now()
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    let pergunta_id = pergunta.id.ok_or_else(|| {
        ErrorInternalServerError("Failed to get question ID")
    })?;

    // Mudança aqui: usar unwrap_or para fornecer um valor padrão
    let modelo_url = std::env::var("MODELO_API_URL")
        .unwrap_or_else(|_| "http://localhost:5000".to_string());

    let client = reqwest::Client::new();
    let modelo_request = ModeloRequest {
        message: body.texto.clone(),
    };

    let modelo_response = client
        .post(format!("{}/chat", modelo_url))
        .json(&modelo_request)
        .send()
        .await
        .map_err(|e| ErrorInternalServerError(format!("Failed to call model API: {}", e)))?
        .json::<ModeloResponse>()
        .await
        .map_err(|e| ErrorInternalServerError(format!("Failed to parse model response: {}", e)))?;

    if modelo_response.status != 200 {
        return Err(ErrorInternalServerError(
            format!("Model API returned error status: {}", modelo_response.status)
        ));
    }

    let resposta = sqlx::query_as!(
        Resposta,
        "INSERT INTO respostas (
            pergunta_id,
            respondido_por_tipo,
            respondido_por_usuario,
            texto,
            criado_em
        )
        VALUES (
            $1,
            $2,
            $3,
            $4,
            $5
        )
        RETURNING *",
        pergunta_id,
        "modelo".to_string(),
        None::<i32>,
        modelo_response.response,
        Utc::now()
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    sqlx::query!(
        "UPDATE perguntas SET 
            estado = $1,
            respondido_em = $2
        WHERE id = $3",
        PerguntaEstado::Answered.to_string(),
        Utc::now(),
        pergunta_id
    )
    .execute(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Question processed and answered successfully".to_string(),
        data: resposta,
    }))
}