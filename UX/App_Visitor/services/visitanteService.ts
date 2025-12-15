import { apiClient } from "@/api/client";

export interface TourVisitanteRel {
  tour_id: number;
  visitante_id: number;
}

export interface Visitante {
  email: string | null;
  nome: string | null;
  telefone: string | null;
}

export async function getTourVisitanteByTourId(
  tourId: number
): Promise<TourVisitanteRel | null> {
  try {
    console.log(`[visitanteService] GET /v1/tour-visitante/tour/${tourId}`);
    const response = await apiClient.get(`/v1/tour-visitante/tour/${tourId}`);

    return response.data?.data ?? null;
  } catch (error: any) {
    if (error?.response?.status === 404) {
      console.log(
        `[visitanteService] Nenhum vínculo tour-visitante para tourId=${tourId}`
      );
      return null;
    }
    console.error(
      "[visitanteService] Erro ao buscar tour-visitante por tourId:",
      error
    );
    throw error;
  }
}

export async function getVisitanteById(
  visitanteId: number
): Promise<Visitante | null> {
  try {
    console.log(`[visitanteService] GET /v1/visitante/${visitanteId}`);
    const response = await apiClient.get(`/v1/visitante/${visitanteId}`);

    return response.data?.data ?? null;
  } catch (error: any) {
    if (error?.response?.status === 404) {
      console.log(
        `[visitanteService] Visitante não encontrado: id=${visitanteId}`
      );
      return null;
    }
    console.error(
      "[visitanteService] Erro ao buscar visitante por id:",
      error
    );
    throw error;
  }
}
