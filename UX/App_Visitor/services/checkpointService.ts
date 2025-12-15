// services/checkpointService.ts
import { apiClient } from "@/api/client";
import { TOUR_ENDPOINTS } from "@/api/endpoints";

export interface Checkpoint {
  id: number;
  tour_id: number;
  tipo: string;
  ordem: number;
  status: string;
  inicio_previsto: string | null;
  inicio_real: string | null;
  fim_real: string | null;
}

export const checkpointService = {
  async getByTourId(tourId: number): Promise<Checkpoint[]> {
    // Ajusta aqui para bater com SEU backend real:
    // Se no Swagger você usa /v1/checkpoints?tour_id=,
    // troque TOUR_ENDPOINTS.CHECKPOINTS por "/checkpoints"
    const res = await apiClient.get<{ data: Checkpoint[] }>(
      TOUR_ENDPOINTS.CHECKPOINTS, // '/tour/checkpoints'
      {
        params: { tour_id: tourId },
      }
    );

    return res.data.data;
  },

  getCurrent(checkpoints: Checkpoint[]): Checkpoint | null {
    if (!checkpoints.length) return null;

    const pending = checkpoints.find((c) => c.status === "pending");
    if (pending) return pending;

    return [...checkpoints].sort((a, b) => a.ordem - b.ordem)[0];
  },
};
