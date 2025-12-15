use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::Checkpoint,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/checkpoint";
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
        (status = OK, body = ListAllResponse<Checkpoint>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let checkpoints: Vec<Checkpoint> = sqlx::query_as!(Checkpoint, "select * from checkpoints")
        .fetch_all(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all checkpoints".to_string(),
        data: checkpoints,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Checkpoint found", body = SingleItemResponse<Checkpoint>),
        (status = NOT_FOUND, description = "Checkpoint not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let checkpoint: Checkpoint =
        sqlx::query_as!(Checkpoint, "select * from checkpoints where id = $1", *id)
            .fetch_one(data.database.as_ref())
            .await
            .map_err(|e| match e {
                sqlx::Error::RowNotFound => ErrorNotFound(e),
                _ => ErrorInternalServerError(e),
            })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved checkpoint".to_string(),
        data: checkpoint,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Checkpoints found", body = ListAllResponse<Checkpoint>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/tour/{tour_id}")]
async fn get_by_tour(tour_id: web::Path<i32>, data: Data) -> Response {
    let checkpoints: Vec<Checkpoint> = sqlx::query_as!(
        Checkpoint,
        "select * from checkpoints where tour_id = $1 order by ordem asc",
        *tour_id
    )
    .fetch_all(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved checkpoints for tour".to_string(),
        data: checkpoints,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Checkpoint created", body = SingleItemResponse<Checkpoint>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<Checkpoint>) -> Response {
    let checkpoint = sqlx::query_as!(
        Checkpoint,
        "INSERT INTO checkpoints (
            tour_id,
            tipo,
            ordem,
            status,
            inicio_previsto,
            inicio_real,
            fim_real
        )
        VALUES (
            $1,
            $2,
            $3,
            $4,
            $5,
            $6,
            $7
        )
        RETURNING *",
        body.tour_id,
        body.tipo.to_string(),
        body.ordem,
        body.status.to_string(),
        body.inicio_previsto,
        body.inicio_real,
        body.fim_real
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created checkpoint".to_string(),
        data: checkpoint,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Checkpoint updated", body = SingleItemResponse<Checkpoint>),
        (status = NOT_FOUND, description = "Checkpoint not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<Checkpoint>) -> Response {
    let checkpoint = sqlx::query_as!(
        Checkpoint,
        "UPDATE checkpoints SET
            tour_id = $1,
            tipo = $2,
            ordem = $3,
            status = $4,
            inicio_previsto = $5,
            inicio_real = $6,
            fim_real = $7
        WHERE id = $8
        RETURNING *",
        body.tour_id,
        body.tipo.to_string(),
        body.ordem,
        body.status.to_string(),
        body.inicio_previsto,
        body.inicio_real,
        body.fim_real,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated checkpoint".to_string(),
        data: checkpoint,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Checkpoint deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Checkpoint not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM checkpoints WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Checkpoint not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted checkpoint".to_string(),
    }))
}
