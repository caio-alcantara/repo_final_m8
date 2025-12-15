use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::Pergunta,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/perguntas";
pub fn router(cfg: &mut ServiceConfig) {
    cfg.service(
        scope(self::SLUG)
            .service(get_all)
            .service(get_one)
            .service(get_by_tour)
            .service(create)
            .service(update)
            .service(delete),
    );
}

#[utoipa::path(
    responses(
        (status = OK, body = ListAllResponse<Pergunta>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let perguntas: Vec<Pergunta> = sqlx::query_as!(Pergunta, "select * from perguntas")
        .fetch_all(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all questions".to_string(),
        data: perguntas,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Question found", body = SingleItemResponse<Pergunta>),
        (status = NOT_FOUND, description = "Question not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let pergunta: Pergunta =
        sqlx::query_as!(Pergunta, "select * from perguntas where id = $1", *id)
            .fetch_one(data.database.as_ref())
            .await
            .map_err(|e| match e {
                sqlx::Error::RowNotFound => ErrorNotFound(e),
                _ => ErrorInternalServerError(e),
            })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved question".to_string(),
        data: pergunta,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, body = ListAllResponse<Pergunta>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/tour/{tour_id}")]
async fn get_by_tour(tour_id: web::Path<i32>, data: Data) -> Response {
    let perguntas: Vec<Pergunta> = sqlx::query_as!(
        Pergunta,
        "select * from perguntas where tour_id = $1",
        *tour_id
    )
    .fetch_all(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all questions".to_string(),
        data: perguntas,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Question created", body = SingleItemResponse<Pergunta>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<Pergunta>) -> Response {
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
        body.estado.to_string(),
        body.liberado_em,
        body.respondido_em,
        body.criado_em
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created question".to_string(),
        data: pergunta,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Question updated", body = SingleItemResponse<Pergunta>),
        (status = NOT_FOUND, description = "Question not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<Pergunta>) -> Response {
    let pergunta = sqlx::query_as!(
        Pergunta,
        "UPDATE perguntas SET
            tour_id = $1,
            checkpoint_id = $2,
            question_topic = $3,
            texto = $4,
            estado = $5,
            liberado_em = $6,
            respondido_em = $7
        WHERE id = $8
        RETURNING *",
        body.tour_id,
        body.checkpoint_id,
        body.question_topic,
        body.texto,
        body.estado.to_string(),
        body.liberado_em,
        body.respondido_em,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated question".to_string(),
        data: pergunta,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Question deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Question not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM perguntas WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Question not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted question".to_string(),
    }))
}
