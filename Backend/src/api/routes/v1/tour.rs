use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::Tour,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/tour";
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
        (status = OK, body = ListAllResponse<Tour>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let tours: Vec<Tour> = sqlx::query_as!(Tour, "select * from tours")
        .fetch_all(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all tours".to_string(),
        data: tours,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tour found", body = SingleItemResponse<Tour>),
        (status = NOT_FOUND, description = "Tour not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let tour: Tour = sqlx::query_as!(Tour, "select * from tours where id = $1", *id)
        .fetch_one(data.database.as_ref())
        .await
        .map_err(|e| match e {
            sqlx::Error::RowNotFound => ErrorNotFound(e),
            _ => ErrorInternalServerError(e),
        })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved tour".to_string(),
        data: tour,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tour created", body = SingleItemResponse<Tour>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<Tour>) -> Response {
    let user_id = 1; // TODO get from auth

    let tour = sqlx::query_as!(
        Tour,
        "INSERT INTO tours (
            codigo,
            titulo,
            data_local,
            hora_inicio_prevista,
            hora_fim_prevista,
            inicio_real,
            fim_real,
            status,
            robo_id,
            responsavel_id,
            criado_por
        )
        VALUES (
            $1,
            $2,
            $3,
            $4,
            $5,
            $6,
            $7,
            $8,
            $9,
            $10,
            $11
        )
        RETURNING *",
        body.codigo,
        body.titulo,
        body.data_local,
        body.hora_inicio_prevista,
        body.hora_fim_prevista,
        body.inicio_real,
        body.fim_real,
        body.status.to_string(),
        body.robo_id,
        body.responsavel_id,
        user_id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created tour".to_string(),
        data: tour,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tour updated", body = SingleItemResponse<Tour>),
        (status = NOT_FOUND, description = "Tour not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<Tour>) -> Response {
    let tour = sqlx::query_as!(
        Tour,
        "UPDATE tours SET
            codigo = $1,
            titulo = $2,
            data_local = $3,
            hora_inicio_prevista = $4,
            hora_fim_prevista = $5,
            inicio_real = $6,
            fim_real = $7,
            status = $8,
            robo_id = $9,
            responsavel_id = $10
        WHERE id = $11
        RETURNING *",
        body.codigo,
        body.titulo,
        body.data_local,
        body.hora_inicio_prevista,
        body.hora_fim_prevista,
        body.inicio_real,
        body.fim_real,
        body.status.to_string(),
        body.robo_id,
        body.responsavel_id,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated tour".to_string(),
        data: tour,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tour deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Tour not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM tours WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Tour not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted tour".to_string(),
    }))
}
