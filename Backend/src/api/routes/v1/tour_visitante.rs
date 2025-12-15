use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::TourVisitante,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/tour-visitante";

pub fn router(cfg: &mut ServiceConfig) {
    cfg.service(
        scope(self::SLUG)
            .service(get_all)
            .service(get_one)
            .service(get_by_tour)
            .service(get_by_visitante)
            .service(create)
            .service(delete),
    );
}

#[utoipa::path(
    responses(
        (status = OK, body = ListAllResponse<TourVisitante>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let tour_visitantes = sqlx::query_as!(TourVisitante, "SELECT * FROM tour_visitantes",)
        .fetch_all(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all tour_visitantes".to_string(),
        data: tour_visitantes,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "TourVisitante found", body = SingleItemResponse<TourVisitante>),
        (status = NOT_FOUND, description = "TourVisitante not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let tour_visitante = sqlx::query_as!(
        TourVisitante,
        "SELECT * FROM tour_visitantes WHERE id = $1",
        *id,
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved tour_visitante".to_string(),
        data: tour_visitante,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Visitantes of tour found", body = ListAllResponse<TourVisitante>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/tour/{tour_id}")]
async fn get_by_tour(tour_id: web::Path<i32>, data: Data) -> Response {
    let tour_visitantes = sqlx::query_as!(
        TourVisitante,
        "SELECT * FROM tour_visitantes WHERE tour_id = $1",
        *tour_id,
    )
    .fetch_all(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved visitantes for tour".to_string(),
        data: tour_visitantes,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Tours of visitante found", body = ListAllResponse<TourVisitante>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/visitante/{visitante_id}")]
async fn get_by_visitante(visitante_id: web::Path<i32>, data: Data) -> Response {
    let tour_visitantes: Vec<TourVisitante> = sqlx::query_as!(
        TourVisitante,
        "SELECT * FROM tour_visitantes WHERE visitante_id = $1",
        *visitante_id
    )
    .fetch_all(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved tours for visitante".to_string(),
        data: tour_visitantes,
    }))
}

#[utoipa::path(
    responses(
        (status = CREATED, description = "TourVisitante created", body = SingleItemResponse<TourVisitante>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<TourVisitante>) -> Response {
    let adicionado_por = 1; // TODO get from auth

    let tour_visitante = sqlx::query_as!(
        TourVisitante,
        "INSERT INTO tour_visitantes (
            tour_id,
            visitante_id,
            adicionado_por,
            adicionado_em
        )
            VALUES (
            $1,
            $2,
            $3,
            $4
        )
        RETURNING *",
        body.tour_id,
        body.visitante_id,
        adicionado_por,
        body.adicionado_em,
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created tour_visitante".to_string(),
        data: tour_visitante,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "TourVisitante updated", body = SingleItemResponse<TourVisitante>),
        (status = NOT_FOUND, description = "TourVisitante not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<TourVisitante>) -> Response {
    let tour_visitante = sqlx::query_as!(
        TourVisitante,
        "UPDATE tour_visitantes SET
            tour_id = $1,
            visitante_id = $2
        WHERE id = $3
        RETURNING *",
        body.tour_id,
        body.visitante_id,
        *id,
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated tour_visitante".to_string(),
        data: tour_visitante,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "TourVisitante deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "TourVisitante not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM tour_visitantes WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("TourVisitante not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted tour_visitante".to_string(),
    }))
}
