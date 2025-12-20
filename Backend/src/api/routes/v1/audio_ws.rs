use actix::{Actor, StreamHandler, AsyncContext, ActorContext, ActorFutureExt, Handler};
use actix_web::{web, Error, HttpRequest, HttpResponse};
use actix_web_actors::ws;
use bytes::Bytes;
use std::time::{Duration, Instant};
use tokio_tungstenite::{connect_async, tungstenite::protocol::Message as WsMessage};
use futures::{SinkExt, StreamExt};
use utoipa_actix_web::service_config::ServiceConfig;
use base64::{Engine as _, engine::general_purpose};
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use actix::Message;

// Importa o módulo de broadcast
use crate::api::routes::v1::broadcast::{
    BroadcastMessage, BroadcastAudio, Broadcast, ConnectAudio, Disconnect,
    get_broadcast_server, MessageType as BroadcastMessageType, RequestTTS
};

#[derive(Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
enum ClientMessage {
    #[serde(rename = "stt")]
    SpeechToText { 
        audio: String,
        checkpoint_id: i32,
        estado: String,
        liberado_em: Option<String>,
        question_topic: Option<String>,
        respondido_em: Option<String>,
        tour_id: Option<i32>,
    },
    #[serde(rename = "text")]
    TextToModel {
        texto: String,
        checkpoint_id: i32,
        estado: String,
        liberado_em: Option<String>,
        question_topic: Option<String>,
        respondido_em: Option<String>,
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

#[derive(Deserialize)]
struct ModelResponse {
    texto: String,
    #[serde(flatten)]
    other: serde_json::Value,
}

struct AudioWebSocket {
    heartbeat: Instant,
    ml_endpoint: String,
    backend_endpoint: String,
    client_id: String,
}

impl AudioWebSocket {
    fn new(ml_endpoint: String, backend_endpoint: String) -> Self {
        Self {
            heartbeat: Instant::now(),
            ml_endpoint,
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

    async fn send_to_ml_ws_stt(
        ml_endpoint: String,
        audio_data: Bytes,
    ) -> Result<String, Box<dyn std::error::Error>> {
        let base64_audio = general_purpose::STANDARD.encode(&audio_data);
        let stt_endpoint = format!("{}/stt", ml_endpoint.trim_end_matches('/'));
        let (ws_stream, _) = connect_async(&stt_endpoint).await?;
        let (mut write, mut read) = ws_stream.split();

        write.send(WsMessage::Text(base64_audio)).await?;

        if let Some(msg) = read.next().await {
            match msg? {
                WsMessage::Text(text) => Ok(text),
                WsMessage::Binary(bin) => {
                    Ok(String::from_utf8_lossy(&bin).to_string())
                }
                _ => Ok(String::from("{\"status\": \"received\"}")),
            }
        } else {
            Err("No response from ML model".into())
        }
    }

    async fn send_to_modelo(
        backend_endpoint: String,
        request_data: ModelRequest,
    ) -> Result<String, Box<dyn std::error::Error>> {
        let client = reqwest::Client::new();
        let url = format!("{}/v1/modelo", backend_endpoint.trim_end_matches('/'));
        
        println!("Sending to modelo at {}: {:?}", url, request_data.texto);
        
        let response = client
            .post(&url)
            .json(&request_data)
            .send()
            .await?;
        
        if !response.status().is_success() {
            return Err(format!("HTTP error: {}", response.status()).into());
        }
        
        let response_text = response.text().await?;
        println!("Received from modelo: {}", response_text);
        
        // Extrai apenas o campo "texto" da resposta
        match serde_json::from_str::<ModelResponse>(&response_text) {
            Ok(model_resp) => Ok(model_resp.texto),
            Err(_) => {
                // Se não conseguir parsear, tenta extrair o texto manualmente
                if let Ok(json) = serde_json::from_str::<serde_json::Value>(&response_text) {
                    if let Some(texto) = json.get("texto").and_then(|v| v.as_str()) {
                        Ok(texto.to_string())
                    } else {
                        Ok(response_text)
                    }
                } else {
                    Ok(response_text)
                }
            }
        }
    }

    // Agora apenas envia para o cliente conectado, não coleta chunks
    async fn stream_tts_audio_to_client(
        ml_endpoint: String,
        text: String,
        ctx_addr: actix::Addr<AudioWebSocket>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let tts_endpoint = format!("{}/tts", ml_endpoint.trim_end_matches('/'));
        let (ws_stream, _) = connect_async(&tts_endpoint).await?;
        let (mut write, mut read) = ws_stream.split();

        write.send(WsMessage::Text(text)).await?;

        while let Some(msg) = read.next().await {
            match msg? {
                WsMessage::Binary(bin) => {
                    println!("Received binary chunk: {} bytes", bin.len());
                    
                    if bin == b"END" {
                        println!("END marker received");
                        break;
                    }
                    
                    let audio_bytes = match general_purpose::STANDARD.decode(&bin) {
                        Ok(decoded) => {
                            println!("Decoded base64 binary: {} bytes", decoded.len());
                            decoded
                        }
                        Err(_) => {
                            println!("Raw binary data: {} bytes", bin.len());
                            bin.to_vec()
                        }
                    };
                    
                    ctx_addr.do_send(SendAudioChunk(audio_bytes));
                }
                WsMessage::Text(text_msg) => {
                    println!("Received text message: {}", text_msg);
                    
                    if text_msg.trim() == "END" {
                        println!("END signal received");
                        break;
                    }
                    
                    if let Ok(audio_bytes) = general_purpose::STANDARD.decode(text_msg.trim()) {
                        println!("Decoded base64 text: {} bytes", audio_bytes.len());
                        ctx_addr.do_send(SendAudioChunk(audio_bytes));
                    }
                }
                WsMessage::Close(_) => {
                    println!("Close message received");
                    break;
                }
                _ => {}
            }
        }

        Ok(())
    }

    async fn process_audio_to_audio(
        ml_endpoint: String,
        backend_endpoint: String,
        audio_data: Bytes,
        model_request: ModelRequest,
    ) -> Result<(String, i32, Option<i32>), Box<dyn std::error::Error>> {
        println!("Step 1: Converting audio to text (STT)");
        let transcribed_text = Self::send_to_ml_ws_stt(ml_endpoint.clone(), audio_data).await?;
        println!("Transcribed text: {}", transcribed_text);

        println!("Step 2: Sending to modelo");
        let checkpoint_id = model_request.checkpoint_id;
        let tour_id = model_request.tour_id;
        let mut request = model_request;
        request.texto = transcribed_text.clone();
        let model_response_text = Self::send_to_modelo(backend_endpoint, request).await?;
        println!("Model response (texto only): {}", model_response_text);

        Ok((model_response_text, checkpoint_id, tour_id))
    }

    async fn process_text_to_audio(
        ml_endpoint: String,
        backend_endpoint: String,
        model_request: ModelRequest,
    ) -> Result<(String, i32, Option<i32>), Box<dyn std::error::Error>> {
        println!("Step 1: Sending text to modelo");
        let checkpoint_id = model_request.checkpoint_id;
        let tour_id = model_request.tour_id;
        let model_response_text = Self::send_to_modelo(backend_endpoint, model_request).await?;
        println!("Model response (texto only): {}", model_response_text);

        Ok((model_response_text, checkpoint_id, tour_id))
    }
}

// Mensagem interna para enviar chunks de áudio
#[derive(Message)]
#[rtype(result = "()")]
struct SendAudioChunk(Vec<u8>);

impl Handler<SendAudioChunk> for AudioWebSocket {
    type Result = ();

    fn handle(&mut self, msg: SendAudioChunk, ctx: &mut Self::Context) {
        ctx.binary(msg.0);
    }
}

impl Actor for AudioWebSocket {
    type Context = ws::WebsocketContext<Self>;

    fn started(&mut self, ctx: &mut Self::Context) {
        println!("WebSocket AUDIO connection established - ID: {}", self.client_id);
        self.hb(ctx);
        
        // Registra este cliente no servidor de broadcast para receber áudios
        if let Some(server) = get_broadcast_server() {
            server.do_send(ConnectAudio {
                addr: ctx.address().recipient(),
                client_id: self.client_id.clone(),
            });
        }
    }

    fn stopped(&mut self, _ctx: &mut Self::Context) {
        println!("WebSocket AUDIO connection closed - ID: {}", self.client_id);
        
        // Desregistra este cliente do servidor de broadcast
        if let Some(server) = get_broadcast_server() {
            server.do_send(Disconnect {
                client_id: self.client_id.clone(),
            });
        }
    }
}

// Handler para receber broadcasts de áudio
impl Handler<BroadcastAudio> for AudioWebSocket {
    type Result = ();

    fn handle(&mut self, msg: BroadcastAudio, ctx: &mut Self::Context) {
        println!("AUDIO WS {} received broadcast audio: {} bytes for checkpoint {}", 
                 self.client_id, msg.audio_data.len(), msg.checkpoint_id);
        
        // Envia o áudio para o cliente
        ctx.binary(msg.audio_data);
    }
}

impl StreamHandler<Result<ws::Message, ws::ProtocolError>> for AudioWebSocket {
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
                    Ok(ClientMessage::SpeechToText { 
                        audio,
                        checkpoint_id,
                        estado,
                        liberado_em,
                        question_topic,
                        respondido_em,
                        tour_id,
                    }) => {
                        println!("Processing STT -> Modelo -> TTS flow");
                        match general_purpose::STANDARD.decode(&audio) {
                            Ok(audio_bytes) => {
                                let ml_endpoint = self.ml_endpoint.clone();
                                let backend_endpoint = self.backend_endpoint.clone();
                                let client_id = self.client_id.clone();
                                let ctx_addr = ctx.address();
                                let model_request = ModelRequest {
                                    checkpoint_id,
                                    estado,
                                    liberado_em,
                                    question_topic,
                                    respondido_em,
                                    texto: String::new(),
                                    tour_id,
                                };
                                
                                let fut = async move {
                                    // Processa áudio -> texto -> modelo
                                    let result = AudioWebSocket::process_audio_to_audio(
                                        ml_endpoint.clone(),
                                        backend_endpoint,
                                        audio_bytes.into(),
                                        model_request,
                                    ).await;
                                    
                                    match result {
                                        Ok((text_response, checkpoint_id, tour_id)) => {
                                            // Faz broadcast da resposta de texto
                                            if let Some(server) = get_broadcast_server() {
                                                let broadcast_msg = BroadcastMessage {
                                                    checkpoint_id,
                                                    texto: text_response.clone(),
                                                    message_type: BroadcastMessageType::Resposta,
                                                    tour_id,
                                                };
                                                
                                                server.do_send(Broadcast {
                                                    message: broadcast_msg,
                                                    exclude_client: Some(client_id.clone()),
                                                });
                                                
                                                // Solicita TTS com streaming via broadcast
                                                server.do_send(RequestTTS {
                                                    texto: text_response.clone(),
                                                    checkpoint_id,
                                                    tour_id,
                                                    exclude_client: Some(client_id.clone()),
                                                });
                                            }
                                            
                                            // Stream TTS para este cliente
                                            let _ = AudioWebSocket::stream_tts_audio_to_client(
                                                ml_endpoint,
                                                text_response.clone(),
                                                ctx_addr,
                                            ).await;
                                            
                                            Ok(text_response)
                                        }
                                        Err(e) => Err(e),
                                    }
                                };
                                
                                let fut = actix::fut::wrap_future::<_, Self>(fut);
                                let fut = fut.map(|result, _act, ctx| {
                                    match result {
                                        Ok(text_response) => {
                                            ctx.text(text_response);
                                            ctx.text("{\"done\": true}");
                                        }
                                        Err(e) => {
                                            eprintln!("Error in audio processing flow: {}", e);
                                            ctx.text(format!("{{\"error\": \"{}\"}}", e));
                                        }
                                    }
                                });
                                ctx.spawn(fut);
                            }
                            Err(e) => {
                                ctx.text(format!("{{\"error\": \"Invalid base64 audio: {}\"}}", e));
                            }
                        }
                    }
                    Ok(ClientMessage::TextToModel {
                        texto,
                        checkpoint_id,
                        estado,
                        liberado_em,
                        question_topic,
                        respondido_em,
                        tour_id,
                    }) => {
                        println!("Processing Text -> Modelo -> TTS flow");
                        let ml_endpoint = self.ml_endpoint.clone();
                        let backend_endpoint = self.backend_endpoint.clone();
                        let client_id = self.client_id.clone();
                        let ctx_addr = ctx.address();
                        let model_request = ModelRequest {
                            checkpoint_id,
                            estado,
                            liberado_em,
                            question_topic,
                            respondido_em,
                            texto,
                            tour_id,
                        };
                        
                        let fut = async move {
                            // Processa texto -> modelo
                            let result = AudioWebSocket::process_text_to_audio(
                                ml_endpoint.clone(),
                                backend_endpoint,
                                model_request,
                            ).await;
                            
                            match result {
                                Ok((text_response, checkpoint_id, tour_id)) => {
                                    // Faz broadcast da resposta de texto
                                    if let Some(server) = get_broadcast_server() {
                                        let broadcast_msg = BroadcastMessage {
                                            checkpoint_id,
                                            texto: text_response.clone(),
                                            message_type: BroadcastMessageType::Resposta,
                                            tour_id,
                                        };
                                        
                                        server.do_send(Broadcast {
                                            message: broadcast_msg,
                                            exclude_client: Some(client_id.clone()),
                                        });
                                        
                                        // Solicita TTS com streaming via broadcast
                                        server.do_send(RequestTTS {
                                            texto: text_response.clone(),
                                            checkpoint_id,
                                            tour_id,
                                            exclude_client: Some(client_id.clone()),
                                        });
                                    }
                                    
                                    // Stream TTS para este cliente
                                    let _ = AudioWebSocket::stream_tts_audio_to_client(
                                        ml_endpoint,
                                        text_response.clone(),
                                        ctx_addr,
                                    ).await;
                                    
                                    Ok(text_response)
                                }
                                Err(e) => Err(e),
                            }
                        };
                        
                        let fut = actix::fut::wrap_future::<_, Self>(fut);
                        let fut = fut.map(|result, _act, ctx| {
                            match result {
                                Ok(text_response) => {
                                    ctx.text(text_response);
                                    ctx.text("{\"done\": true}");
                                }
                                Err(e) => {
                                    eprintln!("Error in text processing flow: {}", e);
                                    ctx.text(format!("{{\"error\": \"{}\"}}", e));
                                }
                            }
                        });
                        ctx.spawn(fut);
                    }
                    Err(e) => {
                        eprintln!("Failed to parse message: {}", e);
                        ctx.text(format!(
                            "{{\"error\": \"Invalid message format. Expected JSON with 'type' field (stt or text): {}\"}}",
                            e
                        ));
                    }
                }
            }
            Ok(ws::Message::Binary(bin)) => {
                println!("Received raw binary audio: {} bytes", bin.len());
                ctx.text("{\"error\": \"Binary audio must be sent with metadata in JSON format using 'stt' type\"}");
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

pub async fn audio_ws(
    req: HttpRequest,
    stream: web::Payload,
    ml_endpoint: web::Data<String>,
) -> Result<HttpResponse, Error> {
    // Carrega o .env
    dotenvy::dotenv().ok();
    
    let backend_url = std::env::var("BACKEND_URL")
        .expect("BACKEND_URL must be set in .env");
    
    let resp = ws::start(
        AudioWebSocket::new(ml_endpoint.get_ref().clone(), backend_url),
        &req,
        stream,
    );
    println!("WebSocket request");
    resp
}

pub fn router(cfg: &mut ServiceConfig) {
    cfg.route("/audio", web::get().to(audio_ws));
}