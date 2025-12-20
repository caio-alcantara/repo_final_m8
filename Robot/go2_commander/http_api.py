# go2_commander/http_api.py

from fastapi import FastAPI
from pydantic import BaseModel


class CheckpointRequest(BaseModel):
    name: str


class HoldRequest(BaseModel):
    enabled: bool


class MapRequest(BaseModel):
    name: str


def create_app(core):
    app = FastAPI(title="Go2 Commander HTTP API")

    @app.post("/start_checkpoint")
    def start_checkpoint(req: CheckpointRequest):
        ok, msg = core.start_checkpoint(req.name)
        return {"success": ok, "message": msg}

    @app.post("/start_next")
    def start_next():
        ok, msg = core.start_next()
        return {"success": ok, "message": msg}

    @app.post("/pause")
    def pause():
        ok, msg = core.pause()
        return {"success": ok, "message": msg}

    @app.post("/resume")
    def resume():
        ok, msg = core.resume()
        return {"success": ok, "message": msg}

    @app.post("/cancel")
    def cancel():
        ok, msg = core.cancel()
        return {"success": ok, "message": msg}

    @app.post("/set_hold")
    def set_hold(req: HoldRequest):
        ok, msg = core.set_hold(req.enabled)
        return {"success": ok, "message": msg}

    @app.post("/reset")
    def reset_counter():
        ok, msg = core.reset_counter()
        return {"success": ok, "message": msg}

    # 🔥 NEW: LOAD MAP
    @app.post("/load_map")
    def load_map(req: MapRequest):
        ok, msg = core.load_map(req.name)
        return {"success": ok, "message": msg}

    @app.get("/status")
    def status():
        return core.status()

    @app.post("/run_all")
    def run_all():
        """
        Executa todos os checkpoints em sequência:
        Navega → Toca áudio → Espera sucesso → Próximo checkpoint
        """
        ok, msg = core.run_all_checkpoints_async()
        return {"success": ok, "message": msg}

    @app.get("/is_busy")
    def is_busy():
        """Verifica se está navegando ou tocando áudio"""
        return {"busy": core.is_busy()}

    @app.post("/play_stop")
    def play_stop():
        """Toca o áudio de parada/despedida"""
        success = core.play_stop_audio()
        return {"success": success, "message": "Áudio de stop reproduzido" if success else "Falha ao reproduzir áudio de stop"}

    return app
