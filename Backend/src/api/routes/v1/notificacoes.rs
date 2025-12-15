use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::Notificacoes,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/notificacoes";
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
        (status = OK, body = ListAllResponse<Notificacoes>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let notificacoes: Vec<Notificacoes> =
        sqlx::query_as!(Notificacoes, "select * from notificacoes")
            .fetch_all(data.database.as_ref())
            .await
            .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all notifications".to_string(),
        data: notificacoes,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Notifications found", body = SingleItemResponse<Notificacoes>),
        (status = NOT_FOUND, description = "Notifications not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let notificacoes: Notificacoes = sqlx::query_as!(
        Notificacoes,
        "select * from notificacoes where id = $1",
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved notifications".to_string(),
        data: notificacoes,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Notifications created", body = SingleItemResponse<Notificacoes>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<Notificacoes>) -> Response {
    let notificacoes = sqlx::query_as!(
        Notificacoes,
        "INSERT INTO notificacoes (
            usuario_id,
            titulo,
            corpo,
            payload_json,
            lido,
            criado_em
        )
        VALUES (
            $1,
            $2,
            $3,
            $4,
            $5,
            $6
        )
        RETURNING *",
        body.usuario_id,
        body.titulo,
        body.corpo,
        body.payload_json,
        body.lido,
        body.criado_em
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created notifications".to_string(),
        data: notificacoes,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Notifications updated", body = SingleItemResponse<Notificacoes>),
        (status = NOT_FOUND, description = "Notifications not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<Notificacoes>) -> Response {
    let notificacoes = sqlx::query_as!(
        Notificacoes,
        "UPDATE notificacoes SET
            usuario_id = $1,
            titulo = $2,
            corpo = $3,
            payload_json = $4,
            lido = $5,
            criado_em = $6
        WHERE id = $7
        RETURNING *",
        body.usuario_id,
        body.titulo,
        body.corpo,
        body.payload_json,
        body.lido,
        body.criado_em,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated notifications".to_string(),
        data: notificacoes,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Notifications deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Notifications not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM notificacoes WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Notifications not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted notifications".to_string(),
    }))
}
