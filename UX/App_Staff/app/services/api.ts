import axios from 'axios';

// Axios instance shared by all services
const api = axios.create({
  baseURL: 'http://10.140.0.11:8080/v1',
  headers: {
    'Content-Type': 'application/json',
  },
});

// Shared response envelope
export interface ApiResponse<T> {
  data: T;
  message: string;
}

// --- Alertas ---
export type AlertaNivel = 'Baixo' | 'medio' | 'Alto';
export type AlertaOrigem =  'visitor' | 'robot' | 'manager' | null;

export interface Alerta {
  id?: number;
  tour_id: number;
  origem: AlertaOrigem;
  nivel: AlertaNivel;
  mensagem: string | null;
  autor_usuario_id: number | null;
  criado_em?: string;
  resolvido_em: string | null;
}

export const alertasService = {
  list: async () => {
    const response = await api.get<ApiResponse<Alerta[]>>('/alertas');
    return response.data;
  },
  create: async (data: Omit<Alerta, 'id' | 'criado_em'>) => {
    const response = await api.post<ApiResponse<Alerta>>('/alertas', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<Alerta>>(`/alertas/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<Alerta, 'id' | 'criado_em'>) => {
    const response = await api.put<ApiResponse<Alerta>>(`/alertas/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/alertas/${id}`);
    return response.data;
  },
};

// --- Checkpoints ---
export type CheckpointStatus = 'pending' | 'running' | 'finished';
export type CheckpointTipo = 'recepcao' | 'auditorio' | 'atelie' | 'casinhas' | 'dog_house';

export interface Checkpoint {
  id?: number;
  tour_id: number;
  tipo: CheckpointTipo;
  ordem: number;
  status: CheckpointStatus;
  inicio_previsto: string | null;
  inicio_real: string | null;
  fim_real: string | null;
}

export const checkpointService = {
  list: async () => {
    const response = await api.get<ApiResponse<Checkpoint[]>>('/checkpoint');
    return response.data;
  },
  listByTour: async (tourId: number) => {
    const response = await api.get<ApiResponse<Checkpoint[]>>(`/checkpoint/tour/${tourId}`);
    return response.data;
  },
  create: async (data: Omit<Checkpoint, 'id'>) => {
    const response = await api.post<ApiResponse<Checkpoint>>('/checkpoint', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<Checkpoint>>(`/checkpoint/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<Checkpoint, 'id'>) => {
    const response = await api.put<ApiResponse<Checkpoint>>(`/checkpoint/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/checkpoint/${id}`);
    return response.data;
  },
};

// --- Notificacoes ---
export interface Notificacao {
  id?: number;
  usuario_id: number | null;
  titulo: string;
  corpo: string | null;
  payload_json: Record<string, unknown> | null;
  lido: boolean;
  criado_em?: string;
}

export const notificacoesService = {
  list: async () => {
    const response = await api.get<ApiResponse<Notificacao[]>>('/notificacoes');
    return response.data;
  },
  create: async (data: Omit<Notificacao, 'id' | 'criado_em'>) => {
    const response = await api.post<ApiResponse<Notificacao>>('/notificacoes', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<Notificacao>>(`/notificacoes/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<Notificacao, 'id' | 'criado_em'>) => {
    const response = await api.put<ApiResponse<Notificacao>>(`/notificacoes/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/notificacoes/${id}`);
    return response.data;
  },
};

// --- Perguntas ---
export type PerguntaEstado = 'queued' | 'answerable' | 'answered' | 'discarded';

export interface Pergunta {
  id?: number;
  tour_id: number;
  checkpoint_id: number;
  question_topic: string | null;
  texto: string;
  estado: PerguntaEstado;
  criado_em?: string;
  liberado_em: string | null;
  respondido_em: string | null;
}

export const perguntasService = {
  list: async () => {
    const response = await api.get<ApiResponse<Pergunta[]>>('/perguntas');
    return response.data;
  },
  listByTour: async (tourId: number) => {
    const response = await api.get<ApiResponse<Pergunta[]>>(`/perguntas/tour/${tourId}`);
    return response.data;
  },
  create: async (data: Omit<Pergunta, 'id' | 'criado_em'>) => {
    const response = await api.post<ApiResponse<Pergunta>>('/perguntas', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<Pergunta>>(`/perguntas/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<Pergunta, 'id' | 'criado_em'>) => {
    const response = await api.put<ApiResponse<Pergunta>>(`/perguntas/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/perguntas/${id}`);
    return response.data;
  },
};

// --- Respostas ---
export type RespostaAutorTipo = 'robot' | 'manager';

export interface Resposta {
  id?: number;
  pergunta_id: number;
  respondido_por_tipo: RespostaAutorTipo;
  respondido_por_usuario: number | null;
  texto: string;
  criado_em?: string;
}

export const respostasService = {
  list: async () => {
    const response = await api.get<ApiResponse<Resposta[]>>('/respostas');
    return response.data;
  },
  listByPergunta: async (perguntaId: number) => {
    const response = await api.get<ApiResponse<Resposta[]>>(`/respostas/pergunta/${perguntaId}`);
    return response.data;
  },
  create: async (data: Omit<Resposta, 'id' | 'criado_em'>) => {
    const response = await api.post<ApiResponse<Resposta>>('/respostas', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<Resposta>>(`/respostas/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<Resposta, 'id' | 'criado_em'>) => {
    const response = await api.put<ApiResponse<Resposta>>(`/respostas/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/respostas/${id}`);
    return response.data;
  },
};

// --- Rastreio do robo ---
export interface RastreioRobo {
  id?: number;
  tour_id: number;
  checkpoint_id: number | null;
  waypoint: string | null;
  progresso_pct: number | null;
  criado_em?: string;
}

export const rastreioRoboService = {
  list: async () => {
    const response = await api.get<ApiResponse<RastreioRobo[]>>('/rastreio-robo');
    return response.data;
  },
  create: async (data: Omit<RastreioRobo, 'id' | 'criado_em'>) => {
    const response = await api.post<ApiResponse<RastreioRobo>>('/rastreio-robo', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<RastreioRobo>>(`/rastreio-robo/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<RastreioRobo, 'id' | 'criado_em'>) => {
    const response = await api.put<ApiResponse<RastreioRobo>>(`/rastreio-robo/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/rastreio-robo/${id}`);
    return response.data;
  },
};

// --- Robo ---
export interface Robo {
  id?: number;
  nome: string;
  modelo: string | null;
  numero_serie: string | null;
  ativo: boolean;
  criado_em?: string;
}

export const roboService = {
  list: async () => {
    const response = await api.get<ApiResponse<Robo[]>>('/robo');
    return response.data;
  },
  create: async (data: Omit<Robo, 'id' | 'criado_em'>) => {
    const response = await api.post<ApiResponse<Robo>>('/robo', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<Robo>>(`/robo/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<Robo, 'id' | 'criado_em'>) => {
    const response = await api.put<ApiResponse<Robo>>(`/robo/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/robo/${id}`);
    return response.data;
  },
};

// --- Tour ---
export type TourStatus = 'scheduled' | 'in_progress' | 'paused' | 'finished';

export interface Tour {
  id?: number;
  codigo: string;
  titulo: string | null;
  data_local: string;
  hora_inicio_prevista: string | null; // TIME
  hora_fim_prevista: string | null; // TIME
  inicio_real: string | null;
  fim_real: string | null;
  status: TourStatus;
  robo_id: number;
  responsavel_id: number | null;
  criado_por: number;
  criado_em?: string;
}

export const tourService = {
  list: async () => {
    const response = await api.get<ApiResponse<Tour[]>>('/tour');
    return response.data;
  },
  create: async (data: Omit<Tour, 'id' | 'criado_em'>) => {
    const response = await api.post<ApiResponse<Tour>>('/tour', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<Tour>>(`/tour/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<Tour, 'id' | 'criado_em'>) => {
    const response = await api.put<ApiResponse<Tour>>(`/tour/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/tour/${id}`);
    return response.data;
  },
  // Mock helper enquanto a rota real não existe
  tourMock: async () => {
    return 27;
  },
  tourNow: async(id:number) =>{
    if(id === 27) return true;
    return false;
  }
};

// --- Tour status log ---
export interface TourStatusLog {
  id?: number;
  tour_id: number;
  status: string;
  atualizado_por: number | null;
  atualizado_em: string | null;
  observacoes: string | null;
}

export const tourStatusLogService = {
  list: async () => {
    const response = await api.get<ApiResponse<TourStatusLog[]>>('/tour-status-log');
    return response.data;
  },
  listByTour: async (tourId: number) => {
    const response = await api.get<ApiResponse<TourStatusLog[]>>(`/tour-status-log/tour/${tourId}`);
    return response.data;
  },
  create: async (data: Omit<TourStatusLog, 'id' | 'atualizado_em'>) => {
    const response = await api.post<ApiResponse<TourStatusLog>>('/tour-status-log', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<TourStatusLog>>(`/tour-status-log/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<TourStatusLog, 'id' | 'atualizado_em'>) => {
    const response = await api.put<ApiResponse<TourStatusLog>>(`/tour-status-log/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/tour-status-log/${id}`);
    return response.data;
  },
};

// --- Tour-visitante ---
export interface TourVisitante {
  id?: number;
  tour_id: number;
  visitante_id: number;
  adicionado_por?: number | null;
  adicionado_em?: string;
}

export const tourVisitanteService = {
  list: async () => {
    const response = await api.get<ApiResponse<TourVisitante[]>>('/tour-visitante');
    return response.data;
  },
  create: async (data: Omit<TourVisitante, 'id' | 'adicionado_em'>) => {
    const response = await api.post<ApiResponse<TourVisitante>>('/tour-visitante', data);
    return response.data;
  },
  listByTour: async (tourId: number) => {
    const response = await api.get<ApiResponse<TourVisitante[]>>(`/tour-visitante/tour/${tourId}`);
    return response.data;
  },
  listByVisitante: async (visitanteId: number) => {
    const response = await api.get<ApiResponse<TourVisitante[]>>(
      `/tour-visitante/visitante/${visitanteId}`,
    );
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<TourVisitante>>(`/tour-visitante/${id}`);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/tour-visitante/${id}`);
    return response.data;
  },
};

// --- Visitante ---
export interface Visitante {
  id?: number;
  nome: string | null;
  email: string | null;
  telefone: string | null;
  criado_em?: string;
}

export const visitanteService = {
  list: async () => {
    const response = await api.get<ApiResponse<Visitante[]>>('/visitante');
    return response.data;
  },
  create: async (data: Omit<Visitante, 'id' | 'criado_em'>) => {
    const response = await api.post<ApiResponse<Visitante>>('/visitante', data);
    return response.data;
  },
  getById: async (id: number) => {
    const response = await api.get<ApiResponse<Visitante>>(`/visitante/${id}`);
    return response.data;
  },
  update: async (id: number, data: Omit<Visitante, 'id' | 'criado_em'>) => {
    const response = await api.put<ApiResponse<Visitante>>(`/visitante/${id}`, data);
    return response.data;
  },
  remove: async (id: number) => {
    const response = await api.delete<ApiResponse<{ message: string }>>(`/visitante/${id}`);
    return response.data;
  },
};

export interface Usuario {
  id?: number;
  nome: string;
  email: string;
}
export default api;
