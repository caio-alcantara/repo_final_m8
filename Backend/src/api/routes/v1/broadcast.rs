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

// Mensagem para solicitar conversão TTS
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

    async fn convert_text_to_audio(
        ml_endpoint: String,
        text: String,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        use tokio_tungstenite::{connect_async, tungstenite::protocol::Message as WsMessage};
        use futures::{SinkExt, StreamExt};
        use base64::{Engine as _, engine::general_purpose};

        let tts_endpoint = format!("{}/tts", ml_endpoint.trim_end_matches('/'));
        let (ws_stream, _) = connect_async(&tts_endpoint).await?;
        let (mut write, mut read) = ws_stream.split();

        write.send(WsMessage::Text(text)).await?;

        let mut audio_buffer = Vec::new();

        while let Some(msg) = read.next().await {
            match msg? {
                WsMessage::Binary(bin) => {
                    if bin == b"END" {
                        break;
                    }
                    
                    match general_purpose::STANDARD.decode(&bin) {
                        Ok(audio_bytes) => {
                            audio_buffer.extend_from_slice(&audio_bytes);
                        }
                        Err(_) => {
                            audio_buffer.extend_from_slice(&bin);
                        }
                    }
                }
                WsMessage::Text(text) => {
                    if text.trim() == "END" {
                        break;
                    }
                    
                    if let Ok(audio_bytes) = general_purpose::STANDARD.decode(text.trim()) {
                        audio_buffer.extend_from_slice(&audio_bytes);
                    }
                }
                WsMessage::Close(_) => break,
                _ => {}
            }
        }

        if audio_buffer.is_empty() {
            Err("No audio data received from ML model".into())
        } else {
            Ok(audio_buffer)
        }
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
                    continue;
                }
            }
            
            if let Err(e) = client.try_send(msg.message.clone()) {
                eprintln!("Error sending to text client {}: {}", client_id, e);
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
        println!("Received TTS request for checkpoint {}", msg.checkpoint_id);
        
        let ml_endpoint = self.ml_endpoint.clone();
        let texto = msg.texto.clone();
        let checkpoint_id = msg.checkpoint_id;
        let tour_id = msg.tour_id;
        let exclude_client = msg.exclude_client;
        
        let fut = async move {
            match Self::convert_text_to_audio(ml_endpoint, texto).await {
                Ok(audio_data) => {
                    println!("TTS conversion successful: {} bytes", audio_data.len());
                    Some(BroadcastAudio {
                        checkpoint_id,
                        audio_data,
                        tour_id,
                    })
                }
                Err(e) => {
                    eprintln!("Error in TTS conversion: {}", e);
                    None
                }
            }
        };
        
        Box::pin(
            actix::fut::wrap_future::<_, Self>(fut)
                .map(move |audio_result, act, _ctx| {
                    if let Some(audio) = audio_result {
                        // Broadcast o áudio para todos os clientes de áudio
                        for (client_id, client) in &act.audio_clients {
                            if let Some(ref exclude) = exclude_client {
                                if client_id == exclude {
                                    continue;
                                }
                            }
                            
                            if let Err(e) = client.try_send(audio.clone()) {
                                eprintln!("Error sending audio to client {}: {}", client_id, e);
                            }
                        }
                    }
                })
        )
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