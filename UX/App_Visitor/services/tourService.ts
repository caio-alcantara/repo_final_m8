import { apiClient } from '../api/client';
import { TOUR_ENDPOINTS, VISITANTE_ENDPOINTS } from '../api/endpoints';

/**
 * SERVIÇO DE TOUR
 * 
 * Este arquivo contém todos os métodos relacionados ao tour guiado.
 * Gerencia checkpoints, progresso do tour, etc.
 */

// Tipos/Interfaces da API Real
export interface TourDTO {
  id?: number;
  codigo: string;
  data_local: string;
  fim_real: string | null;
  hora_fim_prevista: string | null;
  hora_inicio_prevista: string | null;
  inicio_real: string | null;
  responsavel_id: number | null;
  robo_id: number;
  status: 'scheduled' | 'inprogress' | 'completed' | 'cancelled';
  titulo: string | null;
}

export interface VisitanteDTO {
  id?: number;
  email: string | null;
  nome: string | null;
  telefone: string | null;
}

export interface TourVisitanteDTO {
  id?: number;
  tour_id: number;
  visitante_id: number;
}

export interface TourStatusLogDTO {
  id?: number;
  tour_id: number;
  status: string;
  atualizado_em: string | null;
  observacoes: string | null;
}

export interface TourLoginResponse {
  tour: TourDTO;
  visitantes: VisitanteDTO[];
}

export interface ApiResponse<T> {
  data: T;
  message: string;
}

// Tipos/Interfaces originais (mantidos para compatibilidade)
export interface Checkpoint {
  id: number;
  name: string;
  description: string;
  x: number;
  y: number;
  state: 'unvisited' | 'visited' | 'visiting';
  order: number;
  locationId: string;
}

export interface Tour {
  id: string;
  userId: string;
  startTime: string;
  endTime?: string;
  status: 'in_progress' | 'completed' | 'cancelled';
  currentCheckpointId?: number;
  checkpoints: Checkpoint[];
}

export interface StartTourResponse {
  tour: Tour;
  message: string;
}

export interface CompleteCheckpointData {
  completedAt: string;
  notes?: string;
}

/**
 * Classe de serviço de Tour
 */
class TourService {
  /**
   * Autentica um usuário pelo código do Tour
   * 1. Busca todos os tours
   * 2. Encontra o tour com o código correspondente
   * 3. Busca os visitantes vinculados ao tour
   * 4. Retorna os dados do tour e visitantes
   * 
   * @param code - Código do tour
   * @returns Dados do tour e visitantes
   */
  async loginByCode(code: string): Promise<TourLoginResponse> {
    try {
      console.log('🔍 Buscando tour com código:', code);
      
      // 1. Buscar todos os tours
      const toursResponse = await apiClient.get<ApiResponse<TourDTO[]>>(
        TOUR_ENDPOINTS.GET_ALL
      );
      
      console.log('📋 Tours encontrados:', toursResponse.data.data.length);
      
      // 2. Encontrar o tour com o código correspondente (case insensitive)
      const tour = toursResponse.data.data.find(
        t => t.codigo?.toUpperCase() === code.toUpperCase()
      );
      
      if (!tour) {
        throw new Error('Tour não encontrado com o código fornecido');
      }
      
      if (!tour.id) {
        throw new Error('Tour encontrado mas sem ID válido');
      }
      
      console.log('✅ Tour encontrado:', tour);
      
      // 3. Buscar os visitantes vinculados ao tour
      let visitantes: VisitanteDTO[] = [];
      
      try {
        const visitantesLinkResponse = await apiClient.get<ApiResponse<TourVisitanteDTO[]>>(
          TOUR_ENDPOINTS.GET_VISITORS_BY_TOUR(tour.id)
        );
        
        console.log('🔗 Links encontrados:', visitantesLinkResponse.data.data.length);
        
        // 4. Buscar os dados completos de cada visitante
        const visitantesPromises = visitantesLinkResponse.data.data.map(link =>
          apiClient.get<ApiResponse<VisitanteDTO>>(
            VISITANTE_ENDPOINTS.GET_BY_ID(link.visitante_id)
          )
        );
        
        const visitantesResponses = await Promise.all(visitantesPromises);
        visitantes = visitantesResponses.map(response => response.data.data);
        
        console.log('👥 Visitantes encontrados:', visitantes.length);
      } catch (error) {
        console.warn('⚠️ Erro ao buscar visitantes, continuando sem eles:', error);
        // Continua mesmo sem visitantes
      }
      
      return {
        tour,
        visitantes
      };
      
    } catch (error: any) {
      console.error('❌ Erro ao fazer login com código:', error);
      
      if (error.message === 'Tour não encontrado com o código fornecido') {
        throw error;
      }
      
      if (error.response) {
        throw new Error(`Erro na API: ${error.response.status} - ${error.response.data?.message || 'Erro desconhecido'}`);
      }
      
      if (error.request) {
        throw new Error('Erro de conexão. Verifique sua internet e tente novamente.');
      }
      
      throw new Error('Erro ao processar requisição. Tente novamente.');
    }
  }

  /**
   * Busca um tour específico por ID
   * 
   * @param id - ID do tour
   * @returns Dados do tour
   */
  async getTourById(id: number): Promise<TourDTO> {
    try {
      const response = await apiClient.get<ApiResponse<TourDTO>>(
        TOUR_ENDPOINTS.GET_BY_ID(id)
      );
      return response.data.data;
    } catch (error) {
      console.error('❌ Erro ao buscar tour por ID:', error);
      throw error;
    }
  }

  /**
   * Busca todos os tours
   * 
   * @returns Lista de tours
   */
  async getAllTours(): Promise<TourDTO[]> {
    try {
      const response = await apiClient.get<ApiResponse<TourDTO[]>>(
        TOUR_ENDPOINTS.GET_ALL
      );
      return response.data.data;
    } catch (error) {
      console.error('❌ Erro ao buscar todos os tours:', error);
      throw error;
    }
  }

  /**
   * Cria um novo tour
   * 
   * @param tourData - Dados do tour
   * @returns Tour criado
   */
  async createTour(tourData: Omit<TourDTO, 'id'>): Promise<TourDTO> {
    try {
      const response = await apiClient.post<ApiResponse<TourDTO>>(
        TOUR_ENDPOINTS.CREATE,
        tourData
      );
      return response.data.data;
    } catch (error) {
      console.error('❌ Erro ao criar tour:', error);
      throw error;
    }
  }

  /**
   * Atualiza um tour existente
   * 
   * @param id - ID do tour
   * @param tourData - Dados atualizados
   * @returns Tour atualizado
   */
  async updateTour(id: number, tourData: Omit<TourDTO, 'id'>): Promise<TourDTO> {
    try {
      const response = await apiClient.put<ApiResponse<TourDTO>>(
        TOUR_ENDPOINTS.UPDATE(id),
        tourData
      );
      return response.data.data;
    } catch (error) {
      console.error('❌ Erro ao atualizar tour:', error);
      throw error;
    }
  }

  /**
   * Deleta um tour
   * 
   * @param id - ID do tour
   */
  async deleteTour(id: number): Promise<void> {
    try {
      await apiClient.delete(TOUR_ENDPOINTS.DELETE(id));
    } catch (error) {
      console.error('❌ Erro ao deletar tour:', error);
      throw error;
    }
  }

  /**
   * Busca os logs de status de um tour
   * 
   * @param tourId - ID do tour
   * @returns Lista de logs de status
   */
  async getTourStatusLogs(tourId: number): Promise<TourStatusLogDTO[]> {
    try {
      const response = await apiClient.get<ApiResponse<TourStatusLogDTO[]>>(
        TOUR_ENDPOINTS.GET_STATUS_LOGS_BY_TOUR(tourId)
      );
      return response.data.data;
    } catch (error) {
      console.error('❌ Erro ao buscar logs de status:', error);
      throw error;
    }
  }

  /**
   * Cria um novo log de status para um tour
   * 
   * @param logData - Dados do log
   * @returns Log criado
   */
  async createStatusLog(logData: Omit<TourStatusLogDTO, 'id'>): Promise<TourStatusLogDTO> {
    try {
      const response = await apiClient.post<ApiResponse<TourStatusLogDTO>>(
        TOUR_ENDPOINTS.CREATE_STATUS_LOG,
        logData
      );
      return response.data.data;
    } catch (error) {
      console.error('❌ Erro ao criar log de status:', error);
      throw error;
    }
  }

  /**
   * Vincula um visitante a um tour
   * 
   * @param tourId - ID do tour
   * @param visitanteId - ID do visitante
   * @returns Link criado
   */
  async linkVisitorToTour(tourId: number, visitanteId: number): Promise<TourVisitanteDTO> {
    try {
      const response = await apiClient.post<ApiResponse<TourVisitanteDTO>>(
        TOUR_ENDPOINTS.LINK_VISITOR,
        { tour_id: tourId, visitante_id: visitanteId }
      );
      return response.data.data;
    } catch (error) {
      console.error('❌ Erro ao vincular visitante ao tour:', error);
      throw error;
    }
  }

  // ========== MÉTODOS ORIGINAIS (MOCK) ==========
  // Mantidos para compatibilidade com o código existente

  /**
   * Inicia um novo tour
   * 
   * @returns Dados do tour iniciado
   */
  async startTour(): Promise<StartTourResponse> {
    console.log('🚀 Mock Start Tour');
    return {
      tour: {
        id: 'tour_123',
        userId: 'user_1',
        startTime: new Date().toISOString(),
        status: 'in_progress',
        currentCheckpointId: 1,
        checkpoints: [],
      },
      message: 'Tour iniciado com sucesso',
    };
  }

  /**
   * Busca todos os checkpoints do tour
   * 
   * @returns Lista de checkpoints
   */
  async getCheckpoints(): Promise<Checkpoint[]> {
    console.log('📍 Mock Get Checkpoints');
    return [
      { id: 1, name: 'Recepção', description: 'Entrada principal', x: 77, y: 50, state: 'visited', order: 1, locationId: 'loc_1' },
      { id: 2, name: 'Auditório', description: 'Sala de eventos', x: 50, y: 47, state: 'visited', order: 2, locationId: 'loc_2' },
      { id: 3, name: 'Labs', description: 'Laboratórios', x: 40, y: 38, state: 'visiting', order: 3, locationId: 'loc_3' },
      { id: 4, name: 'Refeitório', description: 'Área de alimentação', x: 14.5, y: 58, state: 'unvisited', order: 4, locationId: 'loc_4' },
      { id: 5, name: 'Biblioteca', description: 'Espaço de estudos', x: 5, y: 28, state: 'unvisited', order: 5, locationId: 'loc_5' },
    ];
  }

  /**
   * Busca um checkpoint específico por ID
   * 
   * @param id - ID do checkpoint
   * @returns Dados do checkpoint
   */
  async getCheckpointById(id: number): Promise<Checkpoint> {
    console.log('📍 Mock Get Checkpoint By ID:', id);
    return {
      id,
      name: `Checkpoint ${id}`,
      description: 'Descrição do checkpoint',
      x: 50,
      y: 50,
      state: 'visiting',
      order: id,
      locationId: `loc_${id}`,
    };
  }

  /**
   * Marca um checkpoint como completo
   * 
   * @param checkpointId - ID do checkpoint
   * @param data - Dados de conclusão (opcional)
   * @returns Checkpoint atualizado
   */
  async completeCheckpoint(
    checkpointId: number,
    data?: CompleteCheckpointData
  ): Promise<Checkpoint> {
    console.log('✅ Mock Complete Checkpoint:', checkpointId, data);
    return {
      id: checkpointId,
      name: `Checkpoint ${checkpointId}`,
      description: 'Checkpoint concluído',
      x: 50,
      y: 50,
      state: 'visited',
      order: checkpointId,
      locationId: `loc_${checkpointId}`,
    };
  }

  /**
   * Busca o tour atual em andamento
   * 
   * @returns Tour em andamento ou null
   */
  async getCurrentTour(): Promise<Tour | null> {
    console.log('🔍 Mock Get Current Tour');
    return null;
  }

  /**
   * Busca o histórico de tours do usuário
   * 
   * @returns Lista de tours realizados
   */
  async getTourHistory(): Promise<Tour[]> {
    console.log('📜 Mock Get Tour History');
    return [];
  }

  /**
   * Cancela o tour atual
   * 
   * @param tourId - ID do tour a ser cancelado
   */
  async cancelTour(tourId: string): Promise<void> {
    console.log('❌ Mock Cancel Tour:', tourId);
  }
}

// Exporta uma instância única do serviço (Singleton)
export const tourService = new TourService();

/**
 * COMO USAR NO COMPONENTE:
 * 
 * import { tourService } from '@/services/tourService';
 * 
 * // Login com código:
 * const handleLogin = async (code: string) => {
 *   try {
 *     const data = await tourService.loginByCode(code);
 *     console.log('Tour:', data.tour);
 *     console.log('Visitantes:', data.visitantes);
 *   } catch (error) {
 *     console.error('Erro:', error);
 *   }
 * };
 * 
 * // No componente de mapa:
 * useEffect(() => {
 *   const loadCheckpoints = async () => {
 *     const data = await tourService.getCheckpoints();
 *     setCheckpoints(data);
 *   };
 *   loadCheckpoints();
 * }, []);
 */