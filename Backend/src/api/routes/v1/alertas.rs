use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::Alertas,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};

pub const SLUG: &str = "/alertas";
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
        (status = OK, body = ListAllResponse<Alertas>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let alertas: Vec<Alertas> = sqlx::query_as!(Alertas, "select * from alertas")
        .fetch_all(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all alerts".to_string(),
        data: alertas,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Alert found", body = SingleItemResponse<Alertas>),
        (status = NOT_FOUND, description = "Alert not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let alertas: Alertas = sqlx::query_as!(Alertas, "select * from alertas where id = $1", *id)
        .fetch_one(data.database.as_ref())
        .await
        .map_err(|e| match e {
            sqlx::Error::RowNotFound => ErrorNotFound(e),
            _ => ErrorInternalServerError(e),
        })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved alert".to_string(),
        data: alertas,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Alert created", body = SingleItemResponse<Alertas>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<Alertas>) -> Response {
    let user_id = 1; // TODO

    let alertas = sqlx::query_as!(
        Alertas,
        "INSERT INTO alertas (
            tour_id,
            origem,
            nivel,
            mensagem,
            autor_usuario_id,
            criado_em,
            resolvido_em
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
        body.origem,
        body.nivel.to_string(),
        body.mensagem,
        user_id,
        body.criado_em,
        body.resolvido_em
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created alert".to_string(),
        data: alertas,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Alert updated", body = SingleItemResponse<Alertas>),
        (status = NOT_FOUND, description = "Alert not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<Alertas>) -> Response {
    let alertas = sqlx::query_as!(
        Alertas,
        "UPDATE alertas SET
            tour_id = $1,
            origem = $2,
            nivel = $3,
            mensagem = $4,
            autor_usuario_id = $5,
            criado_em = $6,
            resolvido_em = $7
        WHERE id = $8
        RETURNING *",
        body.tour_id,
        body.origem,
        body.nivel.to_string(),
        body.mensagem,
        body.autor_usuario_id,
        body.criado_em,
        body.resolvido_em,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated alert".to_string(),
        data: alertas,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Alert deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Alert not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM alertas WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Alert not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted alert".to_string(),
    }))
}
