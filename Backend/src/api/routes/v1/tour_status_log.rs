use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::TourStatusLog,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/tour-status-log";
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
        (status = OK, body = ListAllResponse<TourStatusLog>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let logs: Vec<TourStatusLog> = sqlx::query_as!(
        TourStatusLog,
        "select * from tour_status_log order by atualizado_em desc"
    )
    .fetch_all(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all tour status logs".to_string(),
        data: logs,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tour status log found", body = SingleItemResponse<TourStatusLog>),
        (status = NOT_FOUND, description = "Tour status log not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let log: TourStatusLog = sqlx::query_as!(
        TourStatusLog,
        "select * from tour_status_log where id = $1",
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved tour status log".to_string(),
        data: log,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tour status logs found", body = ListAllResponse<TourStatusLog>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/tour/{tour_id}")]
async fn get_by_tour(tour_id: web::Path<i32>, data: Data) -> Response {
    let logs: Vec<TourStatusLog> = sqlx::query_as!(
        TourStatusLog,
        "select * from tour_status_log where tour_id = $1 order by atualizado_em desc",
        *tour_id
    )
    .fetch_all(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved tour status logs for tour".to_string(),
        data: logs,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tour status log created", body = SingleItemResponse<TourStatusLog>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<TourStatusLog>) -> Response {
    let user_id = 1; // TODO get from auth

    let log = sqlx::query_as!(
        TourStatusLog,
        "INSERT INTO tour_status_log (
            tour_id,
            status,
            atualizado_por,
            observacoes
        )
        VALUES (
            $1,
            $2,
            $3,
            $4
        )
        RETURNING *",
        body.tour_id,
        body.status,
        user_id,
        body.observacoes
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created tour status log".to_string(),
        data: log,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tour status log updated", body = SingleItemResponse<TourStatusLog>),
        (status = NOT_FOUND, description = "Tour status log not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<TourStatusLog>) -> Response {
    let user_id = 1; // TODO get from auth

    let log = sqlx::query_as!(
        TourStatusLog,
        "UPDATE tour_status_log SET
            tour_id = $1,
            status = $2,
            atualizado_por = $3,
            observacoes = $4
        WHERE id = $5
        RETURNING *",
        body.tour_id,
        body.status,
        user_id,
        body.observacoes,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated tour status log".to_string(),
        data: log,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tour status log deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Tour status log not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM tour_status_log WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Tour status log not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted tour status log".to_string(),
    }))
}
