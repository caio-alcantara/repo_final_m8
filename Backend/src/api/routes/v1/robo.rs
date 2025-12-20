use actix_web::{
    HttpResponse, delete,
    error::{ErrorInternalServerError, ErrorNotFound},
    get, post, put, web,
};
use utoipa_actix_web::{scope, service_config::ServiceConfig};

use crate::api::models::{
    Data, Response,
    database::Robo,
    responses::{GenericResponse, ListAllResponse, SingleItemResponse},
};
use serde_json::Value;

pub const SLUG: &str = "/robo";
pub fn router(cfg: &mut ServiceConfig) {
    cfg.service(
        scope(self::SLUG)
            // Rotas específicas do robô (devem vir ANTES das rotas com parâmetros)
            .service(status)
            .service(start_next)
            .service(cancel)
            .service(reset_counter)
            .service(add_checkpoint)
            // Rotas CRUD do banco de dados
            .service(get_all)
            .service(create)
            .service(get_one)  // GET /{id} - deve vir DEPOIS das rotas específicas
            .service(update)
            .service(delete),
    );
}

#[utoipa::path(
    responses(
        (status = OK, body = ListAllResponse<Robo>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("")]
async fn get_all(data: Data) -> Response {
    let robos: Vec<Robo> = sqlx::query_as!(Robo, "select * from robos")
        .fetch_all(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Ok().json(ListAllResponse {
        message: "Successfully retrieved all robos".to_string(),
        data: robos,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Robo found", body = SingleItemResponse<Robo>),
        (status = NOT_FOUND, description = "Robo not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[get("/{id}")]
async fn get_one(id: web::Path<i32>, data: Data) -> Response {
    let robo: Robo = sqlx::query_as!(Robo, "select * from robos where id = $1", *id)
        .fetch_one(data.database.as_ref())
        .await
        .map_err(|e| match e {
            sqlx::Error::RowNotFound => ErrorNotFound(e),
            _ => ErrorInternalServerError(e),
        })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully retrieved robo".to_string(),
        data: robo,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Robo created", body = SingleItemResponse<Robo>),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[post("")]
async fn create(data: Data, body: web::Json<Robo>) -> Response {
    let robo = sqlx::query_as!(
        Robo,
        "INSERT INTO robos (
            nome,
            modelo,
            numero_serie,
            ativo,
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
        body.nome,
        body.modelo,
        body.numero_serie,
        body.ativo,
        body.criado_em
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(ErrorInternalServerError)?;

    Ok(HttpResponse::Created().json(SingleItemResponse {
        message: "Successfully created robo".to_string(),
        data: robo,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Robo updated", body = SingleItemResponse<Robo>),
        (status = NOT_FOUND, description = "Robo not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[put("/{id}")]
async fn update(id: web::Path<i32>, data: Data, body: web::Json<Robo>) -> Response {
    let robo = sqlx::query_as!(
        Robo,
        "UPDATE robos SET
            nome = $1,
            modelo = $2,
            numero_serie = $3,
            ativo = $4
        WHERE id = $5
        RETURNING *",
        body.nome,
        body.modelo,
        body.numero_serie,
        body.ativo,
        *id
    )
    .fetch_one(data.database.as_ref())
    .await
    .map_err(|e| match e {
        sqlx::Error::RowNotFound => ErrorNotFound(e),
        _ => ErrorInternalServerError(e),
    })?;

    Ok(HttpResponse::Ok().json(SingleItemResponse {
        message: "Successfully updated robo".to_string(),
        data: robo,
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Robo deleted", body = GenericResponse),
        (status = NOT_FOUND, description = "Robo not found", body = GenericResponse),
        (status = INTERNAL_SERVER_ERROR, description = "Unexpected error", body = GenericResponse),
    )
)]
#[delete("/{id}")]
async fn delete(id: web::Path<i32>, data: Data) -> Response {
    let result = sqlx::query!("DELETE FROM robos WHERE id = $1", *id)
        .execute(data.database.as_ref())
        .await
        .map_err(ErrorInternalServerError)?;

    if result.rows_affected() == 0 {
        return Err(ErrorNotFound("Robo not found"));
    }

    Ok(HttpResponse::Ok().json(GenericResponse {
        message: "Successfully deleted robo".to_string(),
    }))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Status do robô obtido com sucesso"),
        (status = INTERNAL_SERVER_ERROR, description = "Erro ao obter status do robô", body = GenericResponse),
    )
)]
#[get("/status")]
async fn status(data: Data) -> Response {
    let response = data.robot_client
        .get_status_http()
        .await
        .map_err(ErrorInternalServerError)?;
    
    // Retorna a resposta do robô como JSON (assumindo que o robô retorna JSON)
    Ok(HttpResponse::Ok()
        .content_type("application/json")
        .body(response))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Próximo passo iniciado com sucesso"),
        (status = INTERNAL_SERVER_ERROR, description = "Erro ao iniciar próximo passo", body = GenericResponse),
    )
)]
#[post("/start_next")]
async fn start_next(data: Data) -> Response {
    let response = data.robot_client
        .start_next_http()
        .await
        .map_err(ErrorInternalServerError)?;
    
    Ok(HttpResponse::Ok()
        .content_type("application/json")
        .body(response))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Cancelamento enviado com sucesso"),
        (status = INTERNAL_SERVER_ERROR, description = "Erro ao cancelar execução", body = GenericResponse),
    )
)]
#[post("/cancel")]
async fn cancel(data: Data) -> Response {
    let response = data.robot_client
        .cancel_http()
        .await
        .map_err(ErrorInternalServerError)?;
    
    Ok(HttpResponse::Ok()
        .content_type("application/json")
        .body(response))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Reset do contador enviado com sucesso"),
        (status = INTERNAL_SERVER_ERROR, description = "Erro ao resetar contador", body = GenericResponse),
    )
)]
#[post("/reset")]
async fn reset_counter(data: Data) -> Response {
    let response = data.robot_client
        .reset_counter_http()
        .await
        .map_err(ErrorInternalServerError)?;
    
    Ok(HttpResponse::Ok()
        .content_type("application/json")
        .body(response))
}

#[utoipa::path(
    responses(
        (status = OK, description = "Checkpoint adicionado com sucesso"),
        (status = INTERNAL_SERVER_ERROR, description = "Erro ao adicionar checkpoint", body = GenericResponse),
    )
)]
#[post("/add_checkpoint")]
async fn add_checkpoint(data: Data, body: web::Json<Value>) -> Response {
    let response = data.robot_client
        .add_checkpoint_http(&body.0)
        .await
        .map_err(ErrorInternalServerError)?;
    
    Ok(HttpResponse::Ok()
        .content_type("application/json")
        .body(response))
}
