import { apiClient } from '@/api/client';
import { EMERGENCY_ENDPOINTS } from '@/api/endpoints';

/**
 * SERVIÇO DE EMERGÊNCIA
 * 
 * Este arquivo contém todos os métodos relacionados ao sistema de emergência.
 * Gerencia acionamento, cancelamento e histórico de emergências.
 */

// Tipos/Interfaces
export interface Emergency {
  id: string;
  userId: string;
  tourId?: string;
  type: 'medical' | 'security' | 'general';
  status: 'active' | 'resolved' | 'cancelled';
  location?: {
    latitude: number;
    longitude: number;
    description: string;
  };
  triggeredAt: string;
  resolvedAt?: string;
  notes?: string;
}

export interface TriggerEmergencyRequest {
  type?: 'medical' | 'security' | 'general';
  location?: {
    latitude: number;
    longitude: number;
    description: string;
  };
  notes?: string;
  tourId?: string;
}

export interface TriggerEmergencyResponse {
  emergency: Emergency;
  message: string;
  estimatedResponseTime?: string;
}

/**
 * Classe de serviço de Emergência
 */
class EmergencyService {
  /**
   * Aciona uma emergência
   * 
   * @param request - Dados da emergência (opcional)
   * @returns Dados da emergência acionada
   * 
   * Exemplo de uso:
   * ```typescript
   * const emergency = await emergencyService.triggerEmergency({
   *   type: 'medical',
   *   notes: 'Visitante passou mal',
   *   tourId: 'tour_123'
   * });
   * Alert.alert('Emergência', emergency.message);
   * ```
   */
  async triggerEmergency(
    request?: TriggerEmergencyRequest
  ): Promise<TriggerEmergencyResponse> {
    // TODO: Descomente quando a API estiver pronta
    // const response = await apiClient.post<TriggerEmergencyResponse>(
    //   EMERGENCY_ENDPOINTS.TRIGGER,
    //   request || { type: 'general' }
    // );
    // return response.data;

    // Mock de resposta para desenvolvimento
    console.log('🚨 Mock Trigger Emergency:', request);
    
    return {
      emergency: {
        id: `emergency_${Date.now()}`,
        userId: 'user_1',
        tourId: request?.tourId,
        type: request?.type || 'general',
        status: 'active',
        location: request?.location,
        triggeredAt: new Date().toISOString(),
        notes: request?.notes,
      },
      message: 'Emergência acionada! A equipe Inteli foi notificada e chegará em breve.',
      estimatedResponseTime: '2-3 minutos',
    };
  }

  /**
   * Verifica o status de uma emergência ativa
   * 
   * @returns Dados da emergência ativa ou null
   * 
   * Exemplo de uso:
   * ```typescript
   * const activeEmergency = await emergencyService.getActiveEmergency();
   * if (activeEmergency) {
   *   console.log('Emergência ativa:', activeEmergency.status);
   * }
   * ```
   */
  async getActiveEmergency(): Promise<Emergency | null> {
    // TODO: Descomente quando a API estiver pronta
    // try {
    //   const response = await apiClient.get<Emergency>(
    //     EMERGENCY_ENDPOINTS.STATUS
    //   );
    //   return response.data;
    // } catch (error) {
    //   // Se não houver emergência ativa, retorna null
    //   return null;
    // }

    // Mock de resposta para desenvolvimento
    console.log('🔍 Mock Get Active Emergency');
    return null; // Simula que não há emergência ativa
  }

  /**
   * Cancela uma emergência ativa
   * 
   * @param emergencyId - ID da emergência a cancelar
   * @param reason - Motivo do cancelamento (opcional)
   * 
   * Exemplo de uso:
   * ```typescript
   * await emergencyService.cancelEmergency('emergency_123', 'Falso alarme');
   * Alert.alert('Cancelado', 'Emergência cancelada com sucesso');
   * ```
   */
  async cancelEmergency(emergencyId: string, reason?: string): Promise<void> {
    // TODO: Descomente quando a API estiver pronta
    // await apiClient.post(EMERGENCY_ENDPOINTS.CANCEL, {
    //   emergencyId,
    //   reason,
    // });

    console.log('❌ Mock Cancel Emergency:', emergencyId, reason);
  }

  /**
   * Busca o histórico de emergências do usuário
   * 
   * @returns Lista de emergências anteriores
   * 
   * Exemplo de uso:
   * ```typescript
   * const history = await emergencyService.getEmergencyHistory();
   * console.log('Total de emergências:', history.length);
   * ```
   */
  async getEmergencyHistory(): Promise<Emergency[]> {
    // TODO: Descomente quando a API estiver pronta
    // const response = await apiClient.get<Emergency[]>(
    //   EMERGENCY_ENDPOINTS.HISTORY
    // );
    // return response.data;

    // Mock de resposta para desenvolvimento
    console.log('📜 Mock Get Emergency History');
    return [];
  }

  /**
   * Atualiza o status de uma emergência (geralmente usado pela equipe)
   * 
   * @param emergencyId - ID da emergência
   * @param status - Novo status
   * @param notes - Notas adicionais (opcional)
   * 
   * Exemplo de uso:
   * ```typescript
   * await emergencyService.updateEmergencyStatus(
   *   'emergency_123',
   *   'resolved',
   *   'Situação resolvida pela equipe'
   * );
   * ```
   */
  async updateEmergencyStatus(
    emergencyId: string,
    status: 'active' | 'resolved' | 'cancelled',
    notes?: string
  ): Promise<Emergency> {
    // TODO: Descomente quando a API estiver pronta
    // const response = await apiClient.patch<Emergency>(
    //   `${EMERGENCY_ENDPOINTS.STATUS}/${emergencyId}`,
    //   { status, notes }
    // );
    // return response.data;

    // Mock de resposta para desenvolvimento
    console.log('🔄 Mock Update Emergency Status:', emergencyId, status, notes);
    return {
      id: emergencyId,
      userId: 'user_1',
      type: 'general',
      status,
      triggeredAt: new Date().toISOString(),
      resolvedAt: status === 'resolved' ? new Date().toISOString() : undefined,
      notes,
    };
  }

  /**
   * Verifica se há uma emergência ativa no tour atual
   * 
   * @param tourId - ID do tour
   * @returns true se houver emergência ativa
   * 
   * Exemplo de uso:
   * ```typescript
   * const hasEmergency = await emergencyService.hasActiveEmergencyInTour('tour_123');
   * if (hasEmergency) {
   *   // Mostrar aviso na UI
   * }
   * ```
   */
  async hasActiveEmergencyInTour(tourId: string): Promise<boolean> {
    // TODO: Descomente quando a API estiver pronta
    // try {
    //   const response = await apiClient.get<{ hasActiveEmergency: boolean }>(
    //     `${EMERGENCY_ENDPOINTS.STATUS}?tourId=${tourId}`
    //   );
    //   return response.data.hasActiveEmergency;
    // } catch (error) {
    //   return false;
    // }

    // Mock de resposta para desenvolvimento
    console.log('🔍 Mock Has Active Emergency In Tour:', tourId);
    return false;
  }
}

// Exporta uma instância única do serviço (Singleton)
export const emergencyService = new EmergencyService();

/**
 * COMO USAR NO COMPONENTE:
 * 
 * import { emergencyService } from '@/services/emergencyService';
 * 
 * // No componente de emergência:
 * const handleEmergencyTrigger = async () => {
 *   try {
 *     const response = await emergencyService.triggerEmergency({
 *       type: 'general',
 *       tourId: currentTourId,
 *       notes: 'Visitante solicitou ajuda'
 *     });
 *     
 *     Alert.alert(
 *       'Emergência Acionada',
 *       response.message,
 *       [{ text: 'OK' }]
 *     );
 *     
 *     // Interromper tour
 *     // Mostrar feedback visual
 *   } catch (error) {
 *     console.error('Erro ao acionar emergência:', error);
 *     Alert.alert('Erro', 'Não foi possível acionar a emergência');
 *   }
 * };
 * 
 * // Verificar se há emergência ativa
 * useEffect(() => {
 *   const checkEmergency = async () => {
 *     const active = await emergencyService.getActiveEmergency();
 *     setHasActiveEmergency(!!active);
 *   };
 *   checkEmergency();
 * }, []);
 */
