/**
 * TIPOS COMPARTILHADOS DA API
 * 
 * Este arquivo centraliza tipos/interfaces comuns usados em toda a aplicação.
 * Evita duplicação de código e mantém a tipagem consistente.
 */

/**
 * Tipo base para respostas paginadas da API
 */
export interface PaginatedResponse<T> {
  data: T[];
  meta: {
    currentPage: number;
    perPage: number;
    total: number;
    lastPage: number;
  };
  links?: {
    first: string;
    last: string;
    prev: string | null;
    next: string | null;
  };
}

/**
 * Tipo base para respostas de sucesso da API
 */
export interface ApiSuccessResponse<T = any> {
  success: true;
  data: T;
  message?: string;
}

/**
 * Tipo base para respostas de erro da API
 */
export interface ApiErrorResponse {
  success: false;
  error: {
    code: string;
    message: string;
    details?: any;
  };
  statusCode: number;
}

/**
 * Status HTTP comuns
 */
export enum HttpStatus {
  OK = 200,
  CREATED = 201,
  NO_CONTENT = 204,
  BAD_REQUEST = 400,
  UNAUTHORIZED = 401,
  FORBIDDEN = 403,
  NOT_FOUND = 404,
  CONFLICT = 409,
  UNPROCESSABLE_ENTITY = 422,
  TOO_MANY_REQUESTS = 429,
  INTERNAL_SERVER_ERROR = 500,
  SERVICE_UNAVAILABLE = 503,
}

/**
 * Códigos de erro customizados
 */
export enum ErrorCode {
  NETWORK_ERROR = 'NETWORK_ERROR',
  TIMEOUT = 'TIMEOUT',
  UNAUTHORIZED = 'UNAUTHORIZED',
  FORBIDDEN = 'FORBIDDEN',
  NOT_FOUND = 'NOT_FOUND',
  VALIDATION_ERROR = 'VALIDATION_ERROR',
  SERVER_ERROR = 'SERVER_ERROR',
  UNKNOWN_ERROR = 'UNKNOWN_ERROR',
}

/**
 * Tipo para opções de requisição customizadas
 */
export interface RequestOptions {
  showLoader?: boolean; // Mostrar loading na UI
  showErrorAlert?: boolean; // Mostrar alert de erro
  retryOnFailure?: boolean; // Tentar novamente em caso de falha
  cache?: boolean; // Usar cache
  timeout?: number; // Timeout customizado
}

/**
 * Tipo para localização geográfica
 */
export interface Location {
  latitude: number;
  longitude: number;
  accuracy?: number;
  altitude?: number;
  heading?: number;
  speed?: number;
}

/**
 * Tipo para coordenadas no mapa (%)
 */
export interface MapCoordinates {
  x: number; // Porcentagem da largura (0-100)
  y: number; // Porcentagem da altura (0-100)
}

/**
 * Tipo para filtros de busca
 */
export interface SearchFilters {
  query?: string;
  page?: number;
  limit?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
  filters?: Record<string, any>;
}

/**
 * Tipo para upload de arquivos
 */
export interface FileUpload {
  uri: string;
  name: string;
  type: string;
  size?: number;
}

/**
 * COMO USAR:
 * 
 * import { PaginatedResponse, ApiSuccessResponse } from '@/api/types';
 * 
 * // Em um service:
 * async getUsers(): Promise<PaginatedResponse<User>> {
 *   const response = await apiClient.get('/users');
 *   return response.data;
 * }
 */
