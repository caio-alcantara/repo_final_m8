/**
 * CENTRALIZAÇÃO DE ENDPOINTS DA API
 * 
 * Este arquivo centraliza todos os endpoints da API em um único lugar.
 * Facilita a manutenção e evita erros de digitação.
 * 
 * Organize os endpoints por módulo/recurso.
 */

/**
 * Endpoints de Autenticação
 */
export const AUTH_ENDPOINTS = {
  LOGIN: '/auth/login',
  LOGOUT: '/auth/logout',
  REFRESH_TOKEN: '/auth/refresh',
  REGISTER: '/auth/register',
  FORGOT_PASSWORD: '/auth/forgot-password',
  RESET_PASSWORD: '/auth/reset-password',
  VERIFY_EMAIL: '/auth/verify-email',
};

/**
 * Endpoints de Tour
 */
export const TOUR_ENDPOINTS = {
  // Endpoints originais
  START: '/tour/start',
  CHECKPOINTS: '/tour/checkpoints',
  CHECKPOINT_BY_ID: (id: number) => `/tour/checkpoints/${id}`,
  COMPLETE_CHECKPOINT: (id: number) => `/tour/checkpoints/${id}/complete`,
  CURRENT_TOUR: '/tour/current',
  HISTORY: '/tour/history',
  
  // Novos endpoints da API real
  GET_ALL: '/v1/tour',
  GET_BY_ID: (id: number) => `/v1/tour/${id}`,
  CREATE: '/v1/tour',
  UPDATE: (id: number) => `/v1/tour/${id}`,
  DELETE: (id: number) => `/v1/tour/${id}`,
  
  // Endpoints de Tour-Visitante
  GET_VISITORS_BY_TOUR: (tourId: number) => `/v1/tour-visitante/tour/${tourId}`,
  GET_TOURS_BY_VISITOR: (visitorId: number) => `/v1/tour-visitante/visitante/${visitorId}`,
  LINK_VISITOR: '/v1/tour-visitante',
  GET_LINK_BY_ID: (id: number) => `/v1/tour-visitante/${id}`,
  DELETE_LINK: (id: number) => `/v1/tour-visitante/${id}`,
  
  // Endpoints de Status Log
  GET_STATUS_LOGS: '/v1/tour-status-log',
  CREATE_STATUS_LOG: '/v1/tour-status-log',
  GET_STATUS_LOG_BY_ID: (id: number) => `/v1/tour-status-log/${id}`,
  GET_STATUS_LOGS_BY_TOUR: (tourId: number) => `/v1/tour-status-log/tour/${tourId}`,
  UPDATE_STATUS_LOG: (id: number) => `/v1/tour-status-log/${id}`,
  DELETE_STATUS_LOG: (id: number) => `/v1/tour-status-log/${id}`,
};

/**
 * Endpoints de Visitante
 */
export const VISITANTE_ENDPOINTS = {
  GET_ALL: '/v1/visitante',
  GET_BY_ID: (id: number) => `/v1/visitante/${id}`,
  CREATE: '/v1/visitante',
  UPDATE: (id: number) => `/v1/visitante/${id}`,
  DELETE: (id: number) => `/v1/visitante/${id}`,
};

/**
 * Endpoints de Chat/Assistente
 */
export const CHAT_ENDPOINTS = {
  SEND_MESSAGE: '/chat/message',
  CONVERSATION: '/chat/conversation',
  CONVERSATION_BY_ID: (id: string) => `/chat/conversation/${id}`,
  VOICE_TO_TEXT: '/chat/voice-to-text',
  TEXT_TO_VOICE: '/chat/text-to-voice',
};

/**
 * Endpoints de Emergência
 */
export const EMERGENCY_ENDPOINTS = {
  TRIGGER: '/emergency/trigger',
  STATUS: '/emergency/status',
  CANCEL: '/emergency/cancel',
  HISTORY: '/emergency/history',
};

/**
 * Endpoints de Mapa
 */
export const MAP_ENDPOINTS = {
  GET_MAP: '/map',
  GET_LOCATIONS: '/map/locations',
  GET_LOCATION_BY_ID: (id: number) => `/map/locations/${id}`,
  UPDATE_LOCATION: (id: number) => `/map/locations/${id}`,
};

/**
 * Endpoints de Usuário
 */
export const USER_ENDPOINTS = {
  PROFILE: '/user/profile',
  UPDATE_PROFILE: '/user/profile',
  PREFERENCES: '/user/preferences',
  UPDATE_PREFERENCES: '/user/preferences',
};

/**
 * COMO USAR:
 * 
 * Importe os endpoints nos seus services:
 * 
 * import { TOUR_ENDPOINTS } from '@/api/endpoints';
 * 
 * const response = await apiClient.get(TOUR_ENDPOINTS.CHECKPOINTS);
 * const checkpoint = await apiClient.get(TOUR_ENDPOINTS.CHECKPOINT_BY_ID(1));
 */