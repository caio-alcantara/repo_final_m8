use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::RastreioRobo,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/rastreio-robo";
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
        (status = OK, body = ListAllResponse<RastreioRobo>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let rastreios_robo: Vec<RastreioRobo> =
        sqlx::query_as!(RastreioRobo, "select * from rastreios_robo")
            .fetch_all(data.database.as_ref())
            .await
            .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all rastreios".to_string(),
        data: rastreios_robo,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Rastreio found", body = SingleItemResponse<RastreioRobo>),
        (status = NOT_FOUND, description = "Rastreio not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let rastreio_robo: RastreioRobo = sqlx::query_as!(
        RastreioRobo,
        "select * from rastreios_robo where id = $1",
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved rastreio".to_string(),
        data: rastreio_robo,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Rastreio created", body = SingleItemResponse<RastreioRobo>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<RastreioRobo>) -> Response {
    let rastreio_robo = sqlx::query_as!(
        RastreioRobo,
        "INSERT INTO rastreios_robo (
            tour_id,
            checkpoint_id,
            waypoint,
            progresso_pct,
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
        body.tour_id,
        body.checkpoint_id,
        body.waypoint,
        body.progresso_pct,
        body.criado_em
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created rastreio".to_string(),
        data: rastreio_robo,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Rastreio updated", body = SingleItemResponse<RastreioRobo>),
        (status = NOT_FOUND, description = "Rastreio not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<RastreioRobo>) -> Response {
    let rastreio_robo = sqlx::query_as!(
        RastreioRobo,
        "UPDATE rastreios_robo SET
            tour_id = $1,
            checkpoint_id = $2,
            waypoint = $3,
            progresso_pct = $4
        WHERE id = $5
        RETURNING *",
        body.tour_id,
        body.checkpoint_id,
        body.waypoint,
        body.progresso_pct,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated rastreio".to_string(),
        data: rastreio_robo,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Rastreio deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Rastreio not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM rastreios_robo WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Rastreio not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted rastreio".to_string(),
    }))
}
