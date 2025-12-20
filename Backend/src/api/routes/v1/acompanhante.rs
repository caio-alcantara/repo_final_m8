use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::Acompanhante,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/acompanhante";

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
        (status = OK, body = ListAllResponse<Acompanhante>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let acompanhantes = sqlx::query_as!(Acompanhante, "SELECT * FROM acompanhante")
        .fetch_all(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    log::info!("Acompanhantes retornados: {}", acompanhantes.len());

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all acompanhantes".to_string(),
        data: acompanhantes,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Acompanhante found", body = SingleItemResponse<Acompanhante>),
        (status = NOT_FOUND, description = "Acompanhante not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let acompanhante: Acompanhante =
        sqlx::query_as!(Acompanhante, "SELECT * FROM acompanhante WHERE id = $1", *id)
            .fetch_one(data.database.as_ref())
            .await
            .map_err(|e| match e {
                sqlx::Error::RowNotFound => ErrorNotFound(e),
                _ => ErrorInternalServerError(e),
            })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved acompanhante".to_string(),
        data: acompanhante,
    }))
}

#[utoipa::path(
    responses(
        (status = CREATED, description = "Acompanhante created", body = SingleItemResponse<Acompanhante>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<Acompanhante>) -> Response {
    let acompanhante = sqlx::query_as!(
        Acompanhante,
        "INSERT INTO acompanhante (
            nome,
            cpf,
            visitante_id
        )
        VALUES (
            $1,
            $2,
            $3
        )
        RETURNING *",
        body.nome,
        body.cpf,
        body.visitante_id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created acompanhante".to_string(),
        data: acompanhante,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Acompanhante updated", body = SingleItemResponse<Acompanhante>),
        (status = NOT_FOUND, description = "Acompanhante not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<Acompanhante>) -> Response {
    let acompanhante = sqlx::query_as!(
        Acompanhante,
        "UPDATE acompanhante SET
            nome = $1,
            cpf = $2,
            visitante_id = $3
        WHERE id = $4
        RETURNING *",
        body.nome,
        body.cpf,
        body.visitante_id,  // ← VÍRGULA ADICIONADA
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated acompanhante".to_string(),
        data: acompanhante,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Acompanhante deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Acompanhante not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM acompanhante WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Acompanhante not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted acompanhante".to_string(),
    }))
}