// api/chatService.ts
import axios from "axios";
import { apiClient } from "@/api/client";

/**
 * Tipos das estruturas de Pergunta e Resposta
 * de acordo com o backend
 */

export type PerguntaEstado =
  | "queued"
  | "answerable"
  | "answered"
  | "discarded";

export interface Pergunta {
  id: number;
  tour_id: number;
  checkpoint_id: number;
  question_topic: string | null;
  texto: string;
  estado: PerguntaEstado;
  liberado_em: string | null;
  respondido_em: string | null;
  criado_em: string;
}

export interface Resposta {
  id: number;
  pergunta_id: number;
  respondido_por_tipo: string;
  respondido_por_usuario: string | null;
  texto: string;
  criado_em: string;
}

export interface CreatePerguntaRequest {
  tour_id: number;
  checkpoint_id: number;
  question_topic: string | null;
  texto: string;
  estado: PerguntaEstado;
}

/**
 * Payload esperado pelo serviço de modelo (porta 8000)
 */
export interface ModeloRequest {
  tour_id: number;
  checkpoint_id: number;
  question_topic: string | null;
  texto: string;
  estado: PerguntaEstado;
  liberado_em: string | null;
  respondido_em: string | null;
}

/**
 * Client para o backend principal (porta 8080) já vem de apiClient
 * Agora criamos um client específico para o serviço do modelo (porta 8000)
 */
const MODEL_API_BASE_URL =
  process.env.EXPO_PUBLIC_MODEL_API_URL || "http://10.140.0.11:8000";

const modelClient = axios.create({
  baseURL: MODEL_API_BASE_URL,
  headers: {
    Accept: "application/json",
    "Content-Type": "application/json",
  },
});

/**
 * Cria uma nova pergunta diretamente no backend principal
 * (ainda usado para histórico ou se precisar em outros fluxos)
 * POST /v1/perguntas (porta 8080)
 */
export async function createPergunta(
  data: CreatePerguntaRequest
): Promise<Pergunta> {
  console.log("[createPergunta] POST /v1/perguntas", data);
  const response = await apiClient.post("/v1/perguntas", data);

  return response.data.data as Pergunta;
}

/**
 * Busca respostas de uma pergunta pelo novo endpoint:
 * GET /v1/respostas/pergunta/{pergunta_id} (porta 8080)
 *
 * Pode retornar:
 * - { data: [] } → ainda sem resposta
 * - { data: { ...resposta } } → objeto único
 * - { data: [{...}] } → array com uma ou mais respostas
 */
export async function getRespostaByPerguntaId(
  perguntaId: number
): Promise<Resposta | null> {
  try {
    console.log(
      `[getRespostaByPerguntaId] GET /v1/respostas/pergunta/${perguntaId}`
    );
    const response = await apiClient.get(
      `/v1/respostas/pergunta/${perguntaId}`
    );

    const { data } = response.data;

    if (!data) {
      console.log(
        `[getRespostaByPerguntaId] data vazio para pergunta ${perguntaId}`
      );
      return null;
    }

    // Se vier array
    if (Array.isArray(data)) {
      if (data.length === 0) {
        console.log(
          `[getRespostaByPerguntaId] Nenhuma resposta ainda para pergunta ${perguntaId}.`
        );
        return null;
      }
      // pega a primeira (ou poderia ser alguma lógica diferente)
      return data[0] as Resposta;
    }

    // Se vier objeto único
    return data as Resposta;
  } catch (error: any) {
    if (error?.response?.status === 404) {
      console.log(
        `[getRespostaByPerguntaId] 404 para pergunta ${perguntaId} (sem resposta ainda)`
      );
      return null;
    }
    console.error(
      "[getRespostaByPerguntaId] Erro ao buscar resposta por perguntaId:",
      error
    );
    throw error;
  }
}

/**
 * Busca todas as perguntas (histórico bruto)
 * GET /v1/perguntas (porta 8080)
 */
export async function getHistoricoChat(): Promise<Pergunta[]> {
  console.log("[getHistoricoChat] Buscando histórico em /v1/perguntas...");
  const response = await apiClient.get("/v1/perguntas");
  const perguntas = response.data.data as Pergunta[];
  console.log("[getHistoricoChat] Total recebido:", perguntas.length);
  return perguntas;
}

/**
 * Integração definitiva com o modelo (porta 8000)
 *
 * POST /v1/modelo
 * body:
 * {
 *   "checkpoint_id": 4,
 *   "estado": "queued",
 *   "liberado_em": null,
 *   "question_topic": null,
 *   "respondido_em": null,
 *   "texto": "pergunta",
 *   "tour_id": 2
 * }
 *
 * Responde:
 * {
 *   "data": {
 *     "id": 2,
 *     "pergunta_id": 58,
 *     "respondido_por_tipo": "modelo",
 *     "respondido_por_usuario": null,
 *     "texto": "...",
 *     "criado_em": "..."
 *   },
 *   "message": "Question processed and answered successfully"
 * }
 */
export async function askModelo(
  payload: ModeloRequest
): Promise<Resposta> {
  console.log("[askModelo] POST /v1/modelo payload:", payload);

  const response = await modelClient.post("/v1/modelo", payload);

  console.log("[askModelo] /v1/modelo response:", response.data);

  return response.data.data as Resposta;
}
