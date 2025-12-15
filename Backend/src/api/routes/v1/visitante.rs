use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::Visitante,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/visitante";

pub fn router(cfg: &mut ServiceConfig) {
    cfg.service(
        scope(self::SLUG)
            .service(get_all)
            .service(get_one)
            .service(create)
            .service(update)
            .service(delete),
    );
}

#[utoipa::path(
    responses(
        (status = OK, body = ListAllResponse<Visitante>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let visitantes = sqlx::query_as!(Visitante, "SELECT * FROM visitantes")
        .fetch_all(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    log::info!("Visitantes retornados: {}", visitantes.len());

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all visitantes".to_string(),
        data: visitantes,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Visitante found", body = SingleItemResponse<Visitante>),
        (status = NOT_FOUND, description = "Visitante not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let visitante: Visitante =
        sqlx::query_as!(Visitante, "SELECT * FROM visitantes WHERE id = $1", *id)
            .fetch_one(data.database.as_ref())
            .await
            .map_err(|e| match e {
                sqlx::Error::RowNotFound => ErrorNotFound(e),
                _ => ErrorInternalServerError(e),
            })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved visitante".to_string(),
        data: visitante,
    }))
}

#[utoipa::path(
    responses(
        (status = CREATED, description = "Visitante created", body = SingleItemResponse<Visitante>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<Visitante>) -> Response {
    let visitante = sqlx::query_as!(
        Visitante,
        "INSERT INTO visitantes (
            nome,
            email,
            telefone,
            criado_em
        )
        VALUES (
            $1,
            $2,
            $3,
            $4
        )
        RETURNING *",
        body.nome,
        body.email,
        body.telefone,
        body.criado_em,
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created visitante".to_string(),
        data: visitante,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Visitante updated", body = SingleItemResponse<Visitante>),
        (status = NOT_FOUND, description = "Visitante not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<Visitante>) -> Response {
    let visitante = sqlx::query_as!(
        Visitante,
        "UPDATE visitantes SET
            nome = $1,
            email = $2,
            telefone = $3
         WHERE id = $4
         RETURNING *",
        body.nome,
        body.email,
        body.telefone,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated visitante".to_string(),
        data: visitante,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Visitante deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Visitante not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM visitantes WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Visitante not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted visitante".to_string(),
    }))
}
