/**
 * BARREL EXPORT - API
 * 
 * Este arquivo facilita as importações centralizando todos os exports da camada de API.
 * Permite importar tudo de um único lugar.
 */

// Cliente Axios configurado
export { apiClient } from './client';

// Endpoints
export * from './endpoints';

// Tipos
export * from './types';

// Error Handler
export * from './errorHandler';

/**
 * COMO USAR:
 * 
 * Antes (múltiplas importações):
 * import { apiClient } from '@/api/client';
 * import { AUTH_ENDPOINTS } from '@/api/endpoints';
 * import { handleApiError } from '@/api/errorHandler';
 * 
 * Depois (import único):
 * import { apiClient, AUTH_ENDPOINTS, handleApiError } from '@/api';
 */
