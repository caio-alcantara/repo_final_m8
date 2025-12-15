/**
 * BARREL EXPORT - SERVICES
 * 
 * Este arquivo facilita as importações centralizando todos os services.
 */

export * from './authService';
export * from './tourService';
export * from './chatService';
export * from './emergencyService';

/**
 * COMO USAR:
 * 
 * Antes:
 * import { authService } from '@/services/authService';
 * import { tourService } from '@/services/tourService';
 * 
 * Depois:
 * import { authService, tourService } from '@/services';
 */
