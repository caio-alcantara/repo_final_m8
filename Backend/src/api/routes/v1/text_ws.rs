use actix::{Actor, StreamHandler, AsyncContext, ActorContext, ActorFutureExt, Handler};
use actix_web::{web, Error, HttpRequest, HttpResponse};
use actix_web_actors::ws;
use std::time::{Duration, Instant};
use utoipa_actix_web::service_config::ServiceConfig;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

// Importa o módulo de broadcast
use crate::api::routes::v1::broadcast::{
    BroadcastServer, BroadcastMessage, Connect, Disconnect, 
    get_broadcast_server, MessageType as BroadcastMessageType,
    RequestTTS
};

#[derive(Deserialize)]
#[serde(tag = "type")]
enum ClientMessage {
    #[serde(rename = "text")]
    Text {
        checkpoint_id: i32,
        estado: String,
        liberado_em: Option<String>,
        question_topic: Option<String>,
        respondido_em: Option<String>,
        texto: String,
        tour_id: Option<i32>,
    },
}

#[derive(Serialize)]
struct ModelRequest {
    checkpoint_id: i32,
    estado: String,
    liberado_em: Option<String>,
    question_topic: Option<String>,
    respondido_em: Option<String>,
    texto: String,
    tour_id: Option<i32>,
}

#[derive(Serialize)]
struct TextResponse {
    texto: String,
    message_type: MessageType,
}

#[derive(Serialize)]
#[serde(rename_all = "lowercase")]
enum MessageType {
    Pergunta,
    Resposta,
}

struct TextWebSocket {
    heartbeat: Instant,
    backend_endpoint: String,
    client_id: String,
}

impl TextWebSocket {
    fn new(backend_endpoint: String) -> Self {
        Self {
            heartbeat: Instant::now(),
            backend_endpoint,
            client_id: Uuid::new_v4().to_string(),
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

    async fn send_to_modelo(
        backend_endpoint: String,
        request_data: ModelRequest,
    ) -> Result<String, Box<dyn std::error::Error>> {
        let client = reqwest::Client::new();
        let url = format!("{}/v1/modelo", backend_endpoint);
        
        let response = client
            .post(&url)
            .json(&request_data)
            .send()
            .await?;
        
        if !response.status().is_success() {
            return Err(format!("HTTP error: {}", response.status()).into());
        }
        
        let response_text = response.text().await?;
        Ok(response_text)
    }
}

impl Actor for TextWebSocket {
    type Context = ws::WebsocketContext<Self>;

    fn started(&mut self, ctx: &mut Self::Context) {
        println!("WebSocket TEXT connection established - ID: {}", self.client_id);
        self.hb(ctx);
        
        // Registra este cliente no servidor de broadcast
        if let Some(server) = get_broadcast_server() {
            server.do_send(Connect {
                addr: ctx.address().recipient(),
                client_id: self.client_id.clone(),
            });
        }
    }

    fn stopped(&mut self, _ctx: &mut Self::Context) {
        println!("WebSocket TEXT connection closed - ID: {}", self.client_id);
        
        // Desregistra este cliente do servidor de broadcast
        if let Some(server) = get_broadcast_server() {
            server.do_send(Disconnect {
                client_id: self.client_id.clone(),
            });
        }
    }
}

// Handler para receber mensagens broadcast
impl Handler<BroadcastMessage> for TextWebSocket {
    type Result = ();

    fn handle(&mut self, msg: BroadcastMessage, ctx: &mut Self::Context) {
        println!("TEXT WS {} received broadcast: {}", self.client_id, msg.texto);
        
        let message_type = match msg.message_type {
            BroadcastMessageType::Pergunta => MessageType::Pergunta,
            BroadcastMessageType::Resposta => MessageType::Resposta,
        };
        
        let response = TextResponse {
            texto: msg.texto,
            message_type,
        };
        
        if let Ok(json) = serde_json::to_string(&response) {
            ctx.text(json);
        }
    }
}

impl StreamHandler<Result<ws::Message, ws::ProtocolError>> for TextWebSocket {
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
                println!("Received text message: {}", text);
                match serde_json::from_str::<ClientMessage>(&text) {
                    Ok(ClientMessage::Text {
                        checkpoint_id,
                        estado,
                        liberado_em,
                        question_topic,
                        respondido_em,
                        texto,
                        tour_id,
                    }) => {
                        println!("Processing text request - checkpoint_id: {}, texto: {}", checkpoint_id, texto);
                        
                        // Envia a pergunta do usuário primeiro
                        let question_response = TextResponse {
                            texto: texto.clone(),
                            message_type: MessageType::Pergunta,
                        };
                        
                        if let Ok(json) = serde_json::to_string(&question_response) {
                            ctx.text(json);
                        }
                        
                        let backend_endpoint = self.backend_endpoint.clone();
                        let client_id = self.client_id.clone();
                        
                        let request_data = ModelRequest {
                            checkpoint_id,
                            estado,
                            liberado_em,
                            question_topic,
                            respondido_em,
                            texto,
                            tour_id,
                        };
                        
                        let fut = async move {
                            match TextWebSocket::send_to_modelo(
                                backend_endpoint,
                                request_data,
                            ).await {
                                Ok(result) => (result, checkpoint_id, tour_id),
                                Err(e) => {
                                    eprintln!("Error sending to modelo: {}", e);
                                    (format!("{{\"error\": \"{}\"}}", e), checkpoint_id, tour_id)
                                }
                            }
                        };
                        
                        let fut = actix::fut::wrap_future::<_, Self>(fut);
                        let fut = fut.map(move |(result, checkpoint_id, tour_id), _act, ctx| {
                            // Envia a resposta do modelo
                            let answer_response = TextResponse {
                                texto: result.clone(),
                                message_type: MessageType::Resposta,
                            };
                            
                            if let Ok(json) = serde_json::to_string(&answer_response) {
                                ctx.text(json);
                            }
                            
                            // Solicita conversão TTS via broadcast
                            if let Some(server) = get_broadcast_server() {
                                println!("Requesting TTS conversion for response");
                                server.do_send(RequestTTS {
                                    texto: result,
                                    checkpoint_id,
                                    tour_id,
                                    exclude_client: Some(client_id),
                                });
                            }
                        });
                        ctx.spawn(fut);
                    }
                    Err(e) => {
                        eprintln!("Error parsing message: {}", e);
                        ctx.text(format!("{{\"error\": \"Invalid message format: {}\"}}", e));
                    }
                }
            }
            Ok(ws::Message::Close(reason)) => {
                println!("WebSocket closing: {:?}", reason);
                ctx.close(reason);
                ctx.stop();
            }
            _ => (),
        }
    }
}

pub async fn text_ws(
    req: HttpRequest,
    stream: web::Payload,
) -> Result<HttpResponse, Error> {
    // Carrega o .env (ignora erro se já estiver carregado)
    dotenvy::dotenv().ok();
    
    let backend_url = std::env::var("BACKEND_URL")
        .expect("BACKEND_URL must be set in .env");
    
    let resp = ws::start(
        TextWebSocket::new(backend_url),
        &req,
        stream,
    );
    println!("WebSocket request");
    resp
}

pub fn router(cfg: &mut ServiceConfig) {
    cfg.route("/text", web::get().to(text_ws));
}