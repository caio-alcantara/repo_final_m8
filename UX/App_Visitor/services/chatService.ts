import { apiClient } from '@/api/client';
import { CHAT_ENDPOINTS } from '@/api/endpoints';

/**
 * SERVIÇO DE CHAT/ASSISTENTE
 * 
 * Este arquivo contém todos os métodos relacionados ao chat com a IA (LIA).
 * Gerencia mensagens, conversas e transcrições de voz.
 */

// Tipos/Interfaces
export interface Message {
  id: string;
  text: string;
  sender: 'user' | 'assistant';
  timestamp: string;
  audioUrl?: string;
  metadata?: Record<string, any>;
}

export interface Conversation {
  id: string;
  userId: string;
  tourId?: string;
  messages: Message[];
  createdAt: string;
  updatedAt: string;
}

export interface SendMessageRequest {
  text: string;
  conversationId?: string;
  tourId?: string;
  context?: Record<string, any>;
}

export interface SendMessageResponse {
  message: Message;
  assistantReply: Message;
  conversationId: string;
}

export interface VoiceToTextRequest {
  audioFile: Blob | File;
  language?: string;
}

export interface VoiceToTextResponse {
  text: string;
  confidence: number;
}

/**
 * Classe de serviço de Chat
 */
class ChatService {
  /**
   * Envia uma mensagem de texto para o assistente
   * 
   * @param request - Dados da mensagem
   * @returns Resposta do assistente
   * 
   * Exemplo de uso:
   * ```typescript
   * const response = await chatService.sendMessage({
   *   text: 'Onde fica o refeitório?',
   *   tourId: 'tour_123'
   * });
   * console.log('Resposta:', response.assistantReply.text);
   * ```
   */
  async sendMessage(request: SendMessageRequest): Promise<SendMessageResponse> {
    // TODO: Descomente quando a API estiver pronta
    // const response = await apiClient.post<SendMessageResponse>(
    //   CHAT_ENDPOINTS.SEND_MESSAGE,
    //   request
    // );
    // return response.data;

    // Mock de resposta para desenvolvimento
    console.log('💬 Mock Send Message:', request.text);
    
    // Simula delay de resposta da IA
    await new Promise(resolve => setTimeout(resolve, 1000));

    return {
      message: {
        id: Date.now().toString(),
        text: request.text,
        sender: 'user',
        timestamp: new Date().toISOString(),
      },
      assistantReply: {
        id: (Date.now() + 1).toString(),
        text: `Esta é uma resposta simulada da LIA para: "${request.text}". Quando a API estiver pronta, você receberá respostas reais!`,
        sender: 'assistant',
        timestamp: new Date().toISOString(),
      },
      conversationId: request.conversationId || 'conv_mock_123',
    };
  }

  /**
   * Busca uma conversa por ID
   * 
   * @param conversationId - ID da conversa
   * @returns Dados da conversa
   * 
   * Exemplo de uso:
   * ```typescript
   * const conversation = await chatService.getConversation('conv_123');
   * setMessages(conversation.messages);
   * ```
   */
  async getConversation(conversationId: string): Promise<Conversation> {
    // TODO: Descomente quando a API estiver pronta
    // const response = await apiClient.get<Conversation>(
    //   CHAT_ENDPOINTS.CONVERSATION_BY_ID(conversationId)
    // );
    // return response.data;

    // Mock de resposta para desenvolvimento
    console.log('🔍 Mock Get Conversation:', conversationId);
    return {
      id: conversationId,
      userId: 'user_1',
      messages: [
        {
          id: '1',
          text: 'Olá! Como posso ajudar?',
          sender: 'assistant',
          timestamp: new Date().toISOString(),
        },
      ],
      createdAt: new Date().toISOString(),
      updatedAt: new Date().toISOString(),
    };
  }

  /**
   * Busca todas as conversas do usuário
   * 
   * @returns Lista de conversas
   * 
   * Exemplo de uso:
   * ```typescript
   * const conversations = await chatService.getAllConversations();
   * console.log('Total de conversas:', conversations.length);
   * ```
   */
  async getAllConversations(): Promise<Conversation[]> {
    // TODO: Descomente quando a API estiver pronta
    // const response = await apiClient.get<Conversation[]>(
    //   CHAT_ENDPOINTS.CONVERSATION
    // );
    // return response.data;

    // Mock de resposta para desenvolvimento
    console.log('📋 Mock Get All Conversations');
    return [];
  }

  /**
   * Converte áudio em texto (Speech-to-Text)
   * 
   * @param request - Arquivo de áudio e configurações
   * @returns Texto transcrito
   * 
   * Exemplo de uso:
   * ```typescript
   * const audioBlob = ... // obtido do gravador de áudio
   * const result = await chatService.voiceToText({
   *   audioFile: audioBlob,
   *   language: 'pt-BR'
   * });
   * console.log('Texto:', result.text);
   * ```
   */
  async voiceToText(request: VoiceToTextRequest): Promise<VoiceToTextResponse> {
    // TODO: Descomente quando a API estiver pronta
    // const formData = new FormData();
    // formData.append('audio', request.audioFile);
    // if (request.language) {
    //   formData.append('language', request.language);
    // }
    // 
    // const response = await apiClient.post<VoiceToTextResponse>(
    //   CHAT_ENDPOINTS.VOICE_TO_TEXT,
    //   formData,
    //   {
    //     headers: {
    //       'Content-Type': 'multipart/form-data',
    //     },
    //   }
    // );
    // return response.data;

    // Mock de resposta para desenvolvimento
    console.log('🎤 Mock Voice to Text');
    
    // Simula delay de processamento
    await new Promise(resolve => setTimeout(resolve, 1500));

    return {
      text: 'Este é um texto simulado da transcrição de áudio. Implemente a API real para funcionalidade completa.',
      confidence: 0.95,
    };
  }

  /**
   * Converte texto em áudio (Text-to-Speech)
   * 
   * @param text - Texto para converter em áudio
   * @returns URL do áudio gerado
   * 
   * Exemplo de uso:
   * ```typescript
   * const audioUrl = await chatService.textToVoice('Bem-vindo ao Inteli!');
   * // Reproduzir o áudio
   * ```
   */
  async textToVoice(text: string): Promise<string> {
    // TODO: Descomente quando a API estiver pronta
    // const response = await apiClient.post<{ audioUrl: string }>(
    //   CHAT_ENDPOINTS.TEXT_TO_VOICE,
    //   { text }
    // );
    // return response.data.audioUrl;

    // Mock de resposta para desenvolvimento
    console.log('🔊 Mock Text to Voice:', text);
    return 'https://example.com/mock_audio.mp3';
  }

  /**
   * Cria uma nova conversa
   * 
   * @param tourId - ID do tour associado (opcional)
   * @returns ID da nova conversa
   * 
   * Exemplo de uso:
   * ```typescript
   * const conversationId = await chatService.createConversation('tour_123');
   * ```
   */
  async createConversation(tourId?: string): Promise<string> {
    // TODO: Descomente quando a API estiver pronta
    // const response = await apiClient.post<{ conversationId: string }>(
    //   CHAT_ENDPOINTS.CONVERSATION,
    //   { tourId }
    // );
    // return response.data.conversationId;

    // Mock de resposta para desenvolvimento
    console.log('➕ Mock Create Conversation, tourId:', tourId);
    return `conv_${Date.now()}`;
  }

  /**
   * Deleta uma conversa
   * 
   * @param conversationId - ID da conversa a deletar
   * 
   * Exemplo de uso:
   * ```typescript
   * await chatService.deleteConversation('conv_123');
   * ```
   */
  async deleteConversation(conversationId: string): Promise<void> {
    // TODO: Descomente quando a API estiver pronta
    // await apiClient.delete(CHAT_ENDPOINTS.CONVERSATION_BY_ID(conversationId));

    console.log('🗑️ Mock Delete Conversation:', conversationId);
  }
}

// Exporta uma instância única do serviço (Singleton)
export const chatService = new ChatService();

/**
 * COMO USAR NO COMPONENTE:
 * 
 * import { chatService } from '@/services/chatService';
 * 
 * // No componente de chat:
 * const handleSendMessage = async (text: string) => {
 *   try {
 *     const response = await chatService.sendMessage({
 *       text,
 *       conversationId: currentConversationId
 *     });
 *     
 *     // Adicionar mensagens ao estado
 *     setMessages(prev => [
 *       ...prev,
 *       response.message,
 *       response.assistantReply
 *     ]);
 *   } catch (error) {
 *     console.error('Erro ao enviar mensagem:', error);
 *   }
 * };
 * 
 * // Com transcrição de voz:
 * const handleVoiceInput = async (audioBlob: Blob) => {
 *   const transcription = await chatService.voiceToText({ audioFile: audioBlob });
 *   await handleSendMessage(transcription.text);
 * };
 */
