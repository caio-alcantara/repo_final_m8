use actix::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BroadcastMessage {
    pub checkpoint_id: i32,
    pub texto: String,
    pub message_type: MessageType,
    pub tour_id: Option<i32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BroadcastAudio {
    pub checkpoint_id: i32,
    pub audio_data: Vec<u8>,
    pub tour_id: Option<i32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MessageType {
    Pergunta,
    Resposta,
}

// Mensagem para registrar um cliente de texto
#[derive(Message)]
#[rtype(result = "()")]
pub struct Connect {
    pub addr: Recipient<BroadcastMessage>,
    pub client_id: String,
}

// Mensagem para registrar um cliente de áudio
#[derive(Message)]
#[rtype(result = "()")]
pub struct ConnectAudio {
    pub addr: Recipient<BroadcastAudio>,
    pub client_id: String,
}

// Mensagem para desregistrar um cliente
#[derive(Message)]
#[rtype(result = "()")]
pub struct Disconnect {
    pub client_id: String,
}

// Mensagem para broadcast de texto
#[derive(Message, Clone)]
#[rtype(result = "()")]
pub struct Broadcast {
    pub message: BroadcastMessage,
    pub exclude_client: Option<String>,
}

// Mensagem para broadcast de áudio
#[derive(Message, Clone)]
#[rtype(result = "()")]
pub struct BroadcastAudioMessage {
    pub audio: BroadcastAudio,
    pub exclude_client: Option<String>,
}

// Mensagem para solicitar conversão TTS com streaming
#[derive(Message)]
#[rtype(result = "()")]
pub struct RequestTTS {
    pub texto: String,
    pub checkpoint_id: i32,
    pub tour_id: Option<i32>,
    pub exclude_client: Option<String>,
}

// Implementação das mensagens para os recipients
impl Message for BroadcastMessage {
    type Result = ();
}

impl Message for BroadcastAudio {
    type Result = ();
}

// Servidor de broadcast
pub struct BroadcastServer {
    text_clients: HashMap<String, Recipient<BroadcastMessage>>,
    audio_clients: HashMap<String, Recipient<BroadcastAudio>>,
    ml_endpoint: String,
}

impl BroadcastServer {
    pub fn new(ml_endpoint: String) -> Self {
        BroadcastServer {
            text_clients: HashMap::new(),
            audio_clients: HashMap::new(),
            ml_endpoint,
        }
    }

    // Função que faz streaming de áudio enviando chunks automaticamente conforme são recebidos
    async fn stream_tts_to_clients(
        ml_endpoint: String,
        text: String,
        checkpoint_id: i32,
        tour_id: Option<i32>,
        audio_clients: Vec<(String, Recipient<BroadcastAudio>)>,
        exclude_client: Option<String>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        use tokio_tungstenite::{connect_async, tungstenite::protocol::Message as WsMessage};
        use futures::{SinkExt, StreamExt};
        use base64::{Engine as _, engine::general_purpose};

        if audio_clients.is_empty() {
            println!("No audio clients to stream to");
            return Ok(());
        }

        println!("Starting TTS streaming for {} audio clients", audio_clients.len());

        let tts_endpoint = format!("{}/tts", ml_endpoint.trim_end_matches('/'));
        let (ws_stream, _) = connect_async(&tts_endpoint).await?;
        let (mut write, mut read) = ws_stream.split();

        // Envia o texto (apenas o texto, sem criado_em)
        write.send(WsMessage::Text(text)).await?;

        let mut chunk_count = 0;

        // Loop para receber e distribuir chunks em tempo real
        while let Some(msg) = read.next().await {
            match msg? {
                WsMessage::Binary(bin) => {
                    if bin == b"END" {
                        println!("TTS streaming completed - {} chunks sent", chunk_count);
                        break;
                    }
                    
                    let audio_bytes = match general_purpose::STANDARD.decode(&bin) {
                        Ok(decoded) => {
                            println!("Decoded base64 binary chunk: {} bytes", decoded.len());
                            decoded
                        }
                        Err(_) => {
                            println!("Raw binary chunk: {} bytes", bin.len());
                            bin.to_vec()
                        }
                    };
                    
                    chunk_count += 1;
                    println!("Broadcasting audio chunk #{}: {} bytes to {} clients", 
                             chunk_count, audio_bytes.len(), audio_clients.len());
                    
                    // Envia o chunk imediatamente para todos os clientes
                    for (client_id, client) in &audio_clients {
                        // Pula o cliente excluído (quem originou a requisição)
                        if let Some(ref exclude) = exclude_client {
                            if client_id == exclude {
                                continue;
                            }
                        }
                        
                        let audio_msg = BroadcastAudio {
                            checkpoint_id,
                            audio_data: audio_bytes.clone(),
                            tour_id,
                        };
                        
                        // Tenta enviar, mas não bloqueia se o cliente não conseguir receber
                        if let Err(e) = client.try_send(audio_msg) {
                            eprintln!("Error sending audio chunk to client {}: {}", client_id, e);
                        } else {
                            println!("✓ Chunk #{} sent to client {}", chunk_count, client_id);
                        }
                    }
                }
                WsMessage::Text(text_msg) => {
                    if text_msg.trim() == "END" {
                        println!("TTS streaming completed (text END) - {} chunks sent", chunk_count);
                        break;
                    }
                    
                    if let Ok(audio_bytes) = general_purpose::STANDARD.decode(text_msg.trim()) {
                        chunk_count += 1;
                        println!("Broadcasting audio chunk #{}: {} bytes to {} clients", 
                                 chunk_count, audio_bytes.len(), audio_clients.len());
                        
                        // Envia o chunk imediatamente para todos os clientes
                        for (client_id, client) in &audio_clients {
                            if let Some(ref exclude) = exclude_client {
                                if client_id == exclude {
                                    continue;
                                }
                            }
                            
                            let audio_msg = BroadcastAudio {
                                checkpoint_id,
                                audio_data: audio_bytes.clone(),
                                tour_id,
                            };
                            
                            if let Err(e) = client.try_send(audio_msg) {
                                eprintln!("Error sending audio chunk to client {}: {}", client_id, e);
                            } else {
                                println!("✓ Chunk #{} sent to client {}", chunk_count, client_id);
                            }
                        }
                    }
                }
                WsMessage::Close(_) => {
                    println!("TTS connection closed - {} chunks sent", chunk_count);
                    break;
                }
                _ => {}
            }
        }

        println!("TTS streaming finished successfully - total {} chunks", chunk_count);
        Ok(())
    }
}

impl Actor for BroadcastServer {
    type Context = Context<Self>;
}

impl Handler<Connect> for BroadcastServer {
    type Result = ();

    fn handle(&mut self, msg: Connect, _: &mut Context<Self>) {
        println!("Text client {} connected to broadcast server", msg.client_id);
        self.text_clients.insert(msg.client_id, msg.addr);
    }
}

impl Handler<ConnectAudio> for BroadcastServer {
    type Result = ();

    fn handle(&mut self, msg: ConnectAudio, _: &mut Context<Self>) {
        println!("Audio client {} connected to broadcast server", msg.client_id);
        self.audio_clients.insert(msg.client_id, msg.addr);
    }
}

impl Handler<Disconnect> for BroadcastServer {
    type Result = ();

    fn handle(&mut self, msg: Disconnect, _: &mut Context<Self>) {
        println!("Client {} disconnected from broadcast server", msg.client_id);
        self.text_clients.remove(&msg.client_id);
        self.audio_clients.remove(&msg.client_id);
    }
}

impl Handler<Broadcast> for BroadcastServer {
    type Result = ();

    fn handle(&mut self, msg: Broadcast, _: &mut Context<Self>) {
        println!("Broadcasting text message to {} clients", self.text_clients.len());
        
        for (client_id, client) in &self.text_clients {
            if let Some(ref exclude) = msg.exclude_client {
                if client_id == exclude {
                    println!("Skipping text broadcast to excluded client {}", client_id);
                    continue;
                }
            }
            
            if let Err(e) = client.try_send(msg.message.clone()) {
                eprintln!("Error sending to text client {}: {}", client_id, e);
            } else {
                println!("✓ Text message sent to client {}", client_id);
            }
        }
    }
}

impl Handler<BroadcastAudioMessage> for BroadcastServer {
    type Result = ();

    fn handle(&mut self, msg: BroadcastAudioMessage, _: &mut Context<Self>) {
        println!("Broadcasting audio message to {} clients", self.audio_clients.len());
        
        for (client_id, client) in &self.audio_clients {
            if let Some(ref exclude) = msg.exclude_client {
                if client_id == exclude {
                    println!("Skipping audio broadcast to excluded client {}", client_id);
                    continue;
                }
            }
            
            if let Err(e) = client.try_send(msg.audio.clone()) {
                eprintln!("Error sending to audio client {}: {}", client_id, e);
            }
        }
    }
}

impl Handler<RequestTTS> for BroadcastServer {
    type Result = ResponseActFuture<Self, ()>;

    fn handle(&mut self, msg: RequestTTS, _: &mut Context<Self>) -> Self::Result {
        println!("=== TTS STREAMING REQUEST ===");
        println!("Checkpoint: {}", msg.checkpoint_id);
        println!("Tour ID: {:?}", msg.tour_id);
        println!("Text length: {} chars", msg.texto.len());
        println!("Exclude client: {:?}", msg.exclude_client);
        println!("Audio clients connected: {}", self.audio_clients.len());
        
        let ml_endpoint = self.ml_endpoint.clone();
        let texto = msg.texto.clone();
        let checkpoint_id = msg.checkpoint_id;
        let tour_id = msg.tour_id;
        let exclude_client = msg.exclude_client;
        
        // Clona a lista de clientes de áudio para usar na task assíncrona
        let audio_clients: Vec<(String, Recipient<BroadcastAudio>)> = 
            self.audio_clients.iter()
                .map(|(id, recipient)| (id.clone(), recipient.clone()))
                .collect();
        
        if audio_clients.is_empty() {
            println!("⚠ No audio clients available for streaming");
        } else {
            println!("✓ Will stream to {} audio clients", audio_clients.len());
        }
        
        let fut = async move {
            println!("Starting async TTS streaming task...");
            
            match Self::stream_tts_to_clients(
                ml_endpoint,
                texto,
                checkpoint_id,
                tour_id,
                audio_clients,
                exclude_client,
            ).await {
                Ok(_) => {
                    println!("✓✓✓ TTS streaming completed successfully ✓✓✓");
                }
                Err(e) => {
                    eprintln!("✗✗✗ Error in TTS streaming: {} ✗✗✗", e);
                }
            }
        };
        
        Box::pin(actix::fut::wrap_future::<_, Self>(fut).map(|_, _, _| {}))
    }
}

// Singleton global do servidor de broadcast
lazy_static::lazy_static! {
    pub static ref BROADCAST_SERVER: Arc<RwLock<Option<Addr<BroadcastServer>>>> = 
        Arc::new(RwLock::new(None));
}

pub fn init_broadcast_server(ml_endpoint: String) -> Addr<BroadcastServer> {
    let server = BroadcastServer::new(ml_endpoint).start();
    *BROADCAST_SERVER.write().unwrap() = Some(server.clone());
    server
}

pub fn get_broadcast_server() -> Option<Addr<BroadcastServer>> {
    BROADCAST_SERVER.read().unwrap().clone()
}