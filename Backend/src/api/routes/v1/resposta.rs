use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::Resposta,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/respostas";
pub fn router(cfg: &mut ServiceConfig) {
    cfg.service(
        scope(self::SLUG)
            .service(get_all)
            .service(get_one)
            .service(get_by_pergunta)
            .service(create)
            .service(update)
            .service(delete),
    );
}

#[utoipa::path(
    responses(
        (status = OK, body = ListAllResponse<Resposta>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let respostas: Vec<Resposta> = sqlx::query_as!(Resposta, "select * from respostas")
        .fetch_all(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all answers".to_string(),
        data: respostas,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Answer found", body = SingleItemResponse<Resposta>),
        (status = NOT_FOUND, description = "Answer not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let resposta: Resposta =
        sqlx::query_as!(Resposta, "select * from respostas where id = $1", *id)
            .fetch_one(data.database.as_ref())
            .await
            .map_err(|e| match e {
                sqlx::Error::RowNotFound => ErrorNotFound(e),
                _ => ErrorInternalServerError(e),
            })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved answer".to_string(),
        data: resposta,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, body = ListAllResponse<Resposta>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/pergunta/{pergunta_id}")]
async fn get_by_pergunta(pergunta_id: web::Path<i32>, data: Data) -> Response {
    let perguntas: Vec<Resposta> = sqlx::query_as!(
        Resposta,
        "select * from respostas where pergunta_id = $1",
        *pergunta_id
    )
    .fetch_all(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all answers".to_string(),
        data: perguntas,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Answer created", body = SingleItemResponse<Resposta>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<Resposta>) -> Response {
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
        body.pergunta_id,
        body.respondido_por_tipo,
        body.respondido_por_usuario,
        body.texto,
        body.criado_em
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created answer".to_string(),
        data: resposta,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Answer updated", body = SingleItemResponse<Resposta>),
        (status = NOT_FOUND, description = "Answer not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<Resposta>) -> Response {
    let resposta = sqlx::query_as!(
        Resposta,
        "UPDATE respostas SET
            pergunta_id = $1,
            respondido_por_tipo = $2,
            respondido_por_usuario = $3,
            texto = $4
        WHERE id = $5
        RETURNING *",
        body.pergunta_id,
        body.respondido_por_tipo,
        body.respondido_por_usuario,
        body.texto,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated answer".to_string(),
        data: resposta,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Answer deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Answer not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM respostas WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Answer not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted answer".to_string(),
    }))
}
