use actix::{Actor, StreamHandler, AsyncContext, ActorContext, ActorFutureExt};
use actix_web::{web, Error, HttpRequest, HttpResponse};
use actix_web_actors::ws;
use serde::{Deserialize, Serialize};
use sqlx::{PgPool, FromRow};
use std::collections::HashMap;
use std::time::{Duration, Instant};
use utoipa_actix_web::service_config::ServiceConfig;
use chrono::{NaiveDate, NaiveTime};
use crate::api::models::AppState;
use crate::api::models::database::Tour;

#[derive(Deserialize)]
struct CheckTourRequest {
    data_local: String,
    horario_verificacao: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, FromRow)]
struct TourQuestion {
    id_pergunta: i32,
    texto_pergunta: String,
    checkpoint: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize, FromRow)]
struct TourResponse {
    id_pergunta: i32,
    texto_resposta: String,
}

#[derive(Serialize)]
struct QuestionWithResponse {
    id_pergunta: i32,
    texto_pergunta: String,
    checkpoint: i32,
    texto_resposta: Option<String>,
}

#[derive(Serialize)]
struct TourActiveResponse {
    status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    tour_id: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    codigo: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    perguntas: Option<Vec<QuestionWithResponse>>,
}


struct TourWebSocket {
    heartbeat: Instant,
    db_pool: PgPool,
}

impl TourWebSocket {
    fn new(db_pool: PgPool) -> Self {
        Self {
            heartbeat: Instant::now(),
            db_pool,
        }
    }

    fn hb(&self, ctx: &mut ws::WebsocketContext<Self>) {
        ctx.run_interval(Duration::from_secs(5), |act, ctx| {
            if Instant::now().duration_since(act.heartbeat) > Duration::from_secs(10) {
                println!("WebSocket timeout, disconnecting");
                ctx.stop();
                return;
            }
            ctx.ping(b"");
        });
    }

    async fn check_active_tour(
        pool: &PgPool,
        data_local: String,
        horario_verificacao: String,
    ) -> Result<TourActiveResponse, String> {
        let data_parsed = NaiveDate::parse_from_str(&data_local, "%Y-%m-%d")
            .map_err(|e| format!("Invalid date format: {}", e))?;
        
        let horario_parsed = NaiveTime::parse_from_str(&horario_verificacao, "%H:%M:%S")
            .map_err(|e| format!("Invalid time format: {}", e))?;

        let tour = sqlx::query_as::<_, Tour>(
            r#"
            SELECT 
                id, codigo, titulo, data_local, hora_inicio_prevista, 
                hora_fim_prevista, inicio_real, fim_real, status, 
                robo_id, responsavel_id, criado_por, criado_em
            FROM tours
            WHERE data_local = $1
            AND hora_inicio_prevista IS NOT NULL
            AND hora_fim_prevista IS NOT NULL
            AND $2 >= hora_inicio_prevista
            AND $2 <= hora_fim_prevista
            AND status IN ('in_progress')
            ORDER BY hora_inicio_prevista ASC
            LIMIT 1
            "#,
        )
        .bind(data_parsed)
        .bind(horario_parsed)
        .fetch_optional(pool)
        .await
        .map_err(|e| format!("Database error: {}", e))?;

        match tour {
            Some(t) => {
                let tour_id = t.id.ok_or("Tour ID is missing")?;

                let questions = sqlx::query_as::<_, TourQuestion>(
                    r#"
                    SELECT 
                        id as id_pergunta,
                        texto as texto_pergunta,
                        checkpoint_id as checkpoint
                    FROM perguntas
                    WHERE tour_id = $1
                    ORDER BY id ASC
                    "#,
                )
                .bind(tour_id)
                .fetch_all(pool)
                .await
                .map_err(|e| format!("Error fetching questions: {}", e))?;

                let responses = sqlx::query_as::<_, TourResponse>(
                    r#"
                    SELECT DISTINCT ON (r.pergunta_id)
                        r.pergunta_id as id_pergunta,
                        r.texto as texto_resposta
                    FROM respostas r
                    INNER JOIN perguntas p ON r.pergunta_id = p.id
                    WHERE p.tour_id = $1
                    ORDER BY r.pergunta_id, r.id DESC
                    "#,
                )
                .bind(tour_id)
                .fetch_all(pool)
                .await
                .map_err(|e| format!("Error fetching responses: {}", e))?;

                let response_map: HashMap<i32, String> = responses
                    .into_iter()
                    .map(|r| (r.id_pergunta, r.texto_resposta))
                    .collect();

                let perguntas_com_respostas: Vec<QuestionWithResponse> = questions
                    .into_iter()
                    .map(|question| QuestionWithResponse {
                        id_pergunta: question.id_pergunta,
                        texto_pergunta: question.texto_pergunta,
                        checkpoint: question.checkpoint,
                        texto_resposta: response_map.get(&question.id_pergunta).cloned(),
                    })
                    .collect();

                Ok(TourActiveResponse {
                    status: "active".to_string(),
                    tour_id: Some(tour_id),
                    codigo: Some(t.codigo),
                    perguntas: Some(perguntas_com_respostas),
                })
            }
            None => Ok(TourActiveResponse {
                status: "inactive".to_string(),
                tour_id: None,
                codigo: None,
                perguntas: None,
            }),
        }
    }
}

impl Actor for TourWebSocket {
    type Context = ws::WebsocketContext<Self>;

    fn started(&mut self, ctx: &mut Self::Context) {
        println!("Tour WebSocket connection established");
        self.hb(ctx);
    }

    fn stopped(&mut self, _: &mut Self::Context) {
        println!("Tour WebSocket connection closed");
    }
}

impl StreamHandler<Result<ws::Message, ws::ProtocolError>> for TourWebSocket {
    fn handle(&mut self, msg: Result<ws::Message, ws::ProtocolError>, ctx: &mut Self::Context) {
        match msg {
            Ok(ws::Message::Ping(msg)) => {
                self.heartbeat = Instant::now();
                ctx.pong(&msg);
            }
            Ok(ws::Message::Pong(_)) => {
                self.heartbeat = Instant::now();
            }
            Ok(ws::Message::Text(text)) => {
                println!("Received tour check request: {}", text);

                match serde_json::from_str::<CheckTourRequest>(&text) {
                    Ok(request) => {
                        let pool = self.db_pool.clone();

                        let fut = async move {
                            match Self::check_active_tour(
                                &pool,
                                request.data_local,
                                request.horario_verificacao
                            ).await {
                                Ok(response) => {
                                    serde_json::to_string(&response).unwrap_or_else(|_| {
                                        r#"{"status": "error", "message": "Failed to serialize response"}"#.to_string()
                                    })
                                }
                                Err(e) => {
                                    eprintln!("Error checking active tour: {}", e);
                                    format!(r#"{{"status": "error", "message": "{}"}}"#, e)
                                }
                            }
                        };

                        let fut = actix::fut::wrap_future::<_, Self>(fut);
                        let fut = fut.map(|result, _act, ctx| {
                            ctx.text(result);
                        });

                        ctx.spawn(fut);
                    }
                    Err(e) => {
                        eprintln!("Failed to parse request: {}", e);
                        ctx.text(format!(
                            r#"{{"status": "error", "message": "Invalid JSON format: {}"}}"#,
                            e
                        ));
                    }
                }
            }
            Ok(ws::Message::Binary(_)) => {
                println!("Binary messages not supported for tour checks");
                ctx.text(r#"{"status": "error", "message": "Binary messages not supported"}"#);
            }
            Ok(ws::Message::Close(reason)) => {
                println!("Tour WebSocket closing: {:?}", reason);
                ctx.close(reason);
                ctx.stop();
            }
            _ => (),
        }
    }
}

pub async fn tour_ws(
    req: HttpRequest,
    stream: web::Payload,
    app_state: web::Data<AppState>,
) -> Result<HttpResponse, Error> {
    let pool = app_state.database.as_ref().clone();
    ws::start(TourWebSocket::new(pool), &req, stream)
}

pub fn router(cfg: &mut ServiceConfig) {
    cfg.route("/ws/tour/check", web::get().to(tour_ws));
}