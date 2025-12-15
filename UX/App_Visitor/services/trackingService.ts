// services/trackingService.ts
import { apiClient } from "@/api/client";

export interface RastreamentoRobo {
  checkpoint_id: number | null;
  progresso_pct: number | null;
  tour_id: number | null;
  waypoint: string | null;
}

export const trackingService = {
  async getByRoboId(roboId: number): Promise<RastreamentoRobo> {
    const response = await apiClient.get(`/rastreio-robo/${roboId}`);
    // Lembrando: API_BASE_URL já tem /v1, então aqui é só /rastreio-robo
    return response.data.data as RastreamentoRobo;
  },
};
