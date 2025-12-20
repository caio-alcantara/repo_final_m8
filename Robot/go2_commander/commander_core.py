import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from slam_toolbox.srv import DeserializePoseGraph
from waypoint_manager import WaypointManager
import os
import requests
import re
import time


class CommanderCore(Node):
    # Configuração da API de áudio
    AUDIO_API_HOST = "http://localhost:8080"
    AUDIO_API_TIMEOUT = 300  # 5 minutos de timeout para áudios longos
    AUDIO_API_POLL_INTERVAL = 0.5  # Intervalo de polling em segundos
    
    def __init__(self):
        super().__init__("go2_commander_core")

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.get_logger().info("Aguardando servidor de ação do Nav2...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 action server pronto!")

        self.deserialize_client = self.create_client(
            DeserializePoseGraph, "/slam_toolbox/deserialize_map"
        )

        self.wp = WaypointManager()

        self._lock = threading.RLock()
        self._current_checkpoint = None
        self._pending_goal_msg = None
        self._pending_checkpoint = None
        self._navigating = False
        self._goal_handle = None
        self._result_future = None
        self._hold_enabled = False
        self._last_message = ""
        self._playing_audio = False  # Flag para indicar que está tocando áudio
        self._audio_success = False  # Flag para indicar sucesso do áudio

        self.create_timer(0.2, self._nav_tick)

    def _get_checkpoint_number(self, checkpoint_name: str) -> int:
        """Extrai o número do checkpoint do nome (ex: 'checkpoint1' -> 1)"""
        match = re.search(r'(\d+)', checkpoint_name)
        if match:
            return int(match.group(1))
        return None

    def _call_audio_api(self, checkpoint_number: int) -> bool:
        """
        Chama a API de áudio para o checkpoint especificado.
        Retorna True se o áudio foi reproduzido com sucesso.
        """
        endpoint = f"{self.AUDIO_API_HOST}/checkpoint_{checkpoint_number}"
        return self._call_audio_endpoint(endpoint, f"checkpoint_{checkpoint_number}")

    def _call_stop_audio(self) -> bool:
        """
        Chama a API de áudio para tocar o áudio de parada/despedida.
        Retorna True se o áudio foi reproduzido com sucesso.
        """
        endpoint = f"{self.AUDIO_API_HOST}/stop"
        return self._call_audio_endpoint(endpoint, "stop")

    def _call_audio_endpoint(self, endpoint: str, name: str) -> bool:
        """
        Chama um endpoint da API de áudio e espera pela resposta de sucesso.
        Retorna True se o áudio foi reproduzido com sucesso.
        """
        try:
            self.get_logger().info(f"Chamando API de áudio: POST {endpoint}")
            
            response = requests.post(endpoint, timeout=self.AUDIO_API_TIMEOUT)
            
            if response.status_code == 200:
                data = response.json()
                # Verifica se a resposta indica sucesso
                if data.get("status") == "success" or data.get("success") == True:
                    self.get_logger().info(f"Áudio do {name} reproduzido com sucesso")
                    return True
                else:
                    self.get_logger().warn(f"API de áudio retornou: {data}")
                    return False
            else:
                self.get_logger().error(f"Erro na API de áudio: HTTP {response.status_code}")
                return False
                
        except requests.exceptions.Timeout:
            self.get_logger().error(f"Timeout ao chamar API de áudio para {name}")
            return False
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f"Erro de conexão com API de áudio em {self.AUDIO_API_HOST}")
            return False
        except Exception as e:
            self.get_logger().error(f"Erro ao chamar API de áudio: {e}")
            return False

    def _play_checkpoint_audio_async(self, checkpoint_name: str):
        """
        Inicia a reprodução de áudio em uma thread separada para não bloquear o ROS.
        """
        checkpoint_number = self._get_checkpoint_number(checkpoint_name)
        
        if checkpoint_number is None:
            self.get_logger().warn(f"Não foi possível extrair número do checkpoint: {checkpoint_name}")
            with self._lock:
                self._playing_audio = False
                self._audio_success = False
            return
        
        def audio_thread():
            success = self._call_audio_api(checkpoint_number)
            with self._lock:
                self._playing_audio = False
                self._audio_success = success
                if success:
                    self._last_message = f"Áudio do {checkpoint_name} finalizado com sucesso"
                else:
                    self._last_message = f"Falha ao reproduzir áudio do {checkpoint_name}"
        
        with self._lock:
            self._playing_audio = True
            self._audio_success = False
        
        thread = threading.Thread(target=audio_thread, daemon=True)
        thread.start()

    def _play_stop_audio_async(self):
        """
        Inicia a reprodução do áudio de parada/despedida em uma thread separada.
        """
        def audio_thread():
            success = self._call_stop_audio()
            with self._lock:
                self._playing_audio = False
                self._audio_success = success
                if success:
                    self._last_message = "Áudio de parada finalizado com sucesso"
                else:
                    self._last_message = "Falha ao reproduzir áudio de parada"
        
        with self._lock:
            self._playing_audio = True
            self._audio_success = False
        
        thread = threading.Thread(target=audio_thread, daemon=True)
        thread.start()

    def play_stop_audio(self):
        """
        Toca o áudio de parada/despedida e espera completar.
        Método bloqueante.
        """
        self.get_logger().info("Tocando áudio de parada...")
        return self._call_stop_audio()

    def load_waypoints(self, filename: str):
        names = self.wp.load(filename)
        with self._lock:
            self._last_message = f"{len(names)} waypoints carregados"
        self.get_logger().info(self._last_message)
        return names

    def load_map(self, map_name: str):
        filename = f"{map_name}.posegraph"

        if not os.path.exists(filename):
            return False, f"Arquivo '{filename}' não encontrado"

        if not self.deserialize_client.wait_for_service(timeout_sec=2.0):
            return False, "Serviço SLAM Toolbox indisponível"

        req = DeserializePoseGraph.Request()
        req.filename = filename
        req.match_type = 1  # exact match

        future = self.deserialize_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            return False, "Falha ao carregar o mapa"

        with self._lock:
            self._last_message = f"Mapa '{map_name}' carregado com sucesso"

        return True, self._last_message

    def _pose_from_payload(self, payload) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(payload["position_x"])
        pose.pose.position.y = float(payload["position_y"])
        pose.pose.position.z = float(payload.get("position_z", 0.0))

        pose.pose.orientation.x = float(payload.get("orientation_x", 0.0))
        pose.pose.orientation.y = float(payload.get("orientation_y", 0.0))
        pose.pose.orientation.z = float(payload.get("orientation_z", 0.0))
        pose.pose.orientation.w = float(payload["orientation_w"])

        return pose

    def _start_nav_goal(self, pose: PoseStamped, name: str):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb
        )

        send_future.add_done_callback(self._goal_response_cb)
        self._pending_goal_msg = None
        self._pending_checkpoint = None

        self.get_logger().info(f"Enviado goal → {name}")

    def _goal_response_cb(self, future):
        """Callback: quando Nav2 aceita ou rejeita o goal"""
        with self._lock:
            self._goal_handle = future.result()

            if not self._goal_handle.accepted:
                self._navigating = False
                self._last_message = f"Navegação rejeitada pelo Nav2 ({self._current_checkpoint})"
                self.get_logger().error(self._last_message)
                return

            self.get_logger().info("Goal aceito pelo Nav2")

            self._result_future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(self._nav_result_cb)

    def _feedback_cb(self, feedback_msg):
        # Você pode processar progresso aqui se desejar
        pass

    def _nav_result_cb(self, future):
        """Callback quando o Nav2 termina a navegação"""
        with self._lock:
            result = future.result().result

            if result == None:
                self._last_message = "Erro ao obter resultado"
                self.get_logger().error(self._last_message)
                self._navigating = False
                return

            status = future.result().status
            checkpoint_name = self._current_checkpoint

            if status == 4:   # STATUS_SUCCEEDED
                self._last_message = f"Chegou em {checkpoint_name}, iniciando áudio..."
                self.get_logger().info(self._last_message)
                # Inicia reprodução de áudio em thread separada
                self._play_checkpoint_audio_async(checkpoint_name)

            elif status == 5:  # STATUS_ABORTED
                self._last_message = f"Navegação falhou ({checkpoint_name})"
                self.get_logger().error(self._last_message)

            elif status == 6:  # STATUS_CANCELED
                self._last_message = f"Navegação cancelada ({checkpoint_name})"
                self.get_logger().warn(self._last_message)

            self._navigating = False

    # =============================================================
    # TIMER: controle automático
    # =============================================================
    def _nav_tick(self):
        with self._lock:

            # iniciar goal pendente
            if self._pending_goal_msg is not None:
                self._start_nav_goal(
                    self._pending_goal_msg,
                    self._pending_checkpoint
                )
                return

    # =============================================================
    # PUBLIC API — igual ao seu SimpleCommander, mas usando Nav2
    # =============================================================
    def start_checkpoint(self, name: str):
        with self._lock:
            if self._hold_enabled:
                return False, "Hold enabled"

            if name not in self.wp.names:
                return False, f"Checkpoint '{name}' não encontrado"

            if self._navigating:
                return False, "Já está navegando"
            
            if self._playing_audio:
                return False, "Áudio ainda está sendo reproduzido"

            payload = self.wp.get(name)
            pose = self._pose_from_payload(payload)

            self._pending_goal_msg = pose
            self._pending_checkpoint = name
            self._current_checkpoint = name
            self._navigating = True

            self._last_message = f"Iniciando navegação → {name}"
            return True, self._last_message

    def start_next(self):
        with self._lock:
            if self._hold_enabled:
                return False, "Hold enabled"
            if self.wp.count() == 0:
                return False, "Nenhum waypoint carregado"

            if self._current_checkpoint in self.wp.names:
                idx = self.wp.names.index(self._current_checkpoint) + 1
            else:
                idx = 0

            if idx >= len(self.wp.names):
                return False, "Não há próximo waypoint"

            name = self.wp.names[idx]

        return self.start_checkpoint(name)

    # ---------------------------------------------------------
    # CANCELAR / PAUSAR / RESUMIR
    # ---------------------------------------------------------
    def pause(self):
        with self._lock:
            if not self._navigating:
                return False, "Não está navegando"

            if self._goal_handle:
                self._goal_handle.cancel_goal_async()

            self._navigating = False
            self._last_message = "Pausado"
            return True, "Paused"

    def cancel(self):
        with self._lock:
            if not self._navigating:
                return False, "Não está navegando"
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            self._navigating = False
            self._last_message = "Cancelado"
            return True, "Canceled"

    def resume(self):
        with self._lock:
            if self._hold_enabled:
                return False, "Hold enabled"
            if self._navigating:
                return False, "Já está navegando"
            if not self._current_checkpoint:
                return False, "Nenhum checkpoint para retomar"

        return self.start_checkpoint(self._current_checkpoint)

    def set_hold(self, enabled: bool):
        with self._lock:
            self._hold_enabled = enabled

            if enabled and self._navigating:
                if self._goal_handle:
                    self._goal_handle.cancel_goal_async()
                self._navigating = False
                self._last_message = "Hold ON → navegação parada"
                return True, self._last_message

            self._last_message = f"Hold set to {enabled}"
            return True, self._last_message

    def status(self) -> dict:
        with self._lock:
            return {
                "current_checkpoint": self._current_checkpoint or "",
                "navigating": self._navigating,
                "playing_audio": self._playing_audio,
                "audio_success": self._audio_success,
                "hold_enabled": self._hold_enabled,
                "last_message": self._last_message,
                "waypoints_count": self.wp.count(),
            }
    def reset_counter(self):
        with self._lock:
            self._current_checkpoint = None
            self._current_index = 0
            return True, "Checkpoint counter reset to 0."

    def is_busy(self) -> bool:
        """Retorna True se está navegando ou tocando áudio"""
        with self._lock:
            return self._navigating or self._playing_audio

    def run_all_checkpoints_async(self):
        """
        Executa todos os checkpoints em sequência:
        1. Navega até o waypoint
        2. Chama a API de áudio correspondente e ESPERA a resposta de sucesso
        3. Repete para o próximo waypoint
        4. Ao final ou ao interromper, toca áudio de stop
        """
        with self._lock:
            if self._navigating or self._playing_audio:
                return False, "Já está navegando ou tocando áudio"
            
            if self.wp.count() == 0:
                return False, "Nenhum waypoint carregado"
            
            if self._hold_enabled:
                return False, "Hold enabled"
        
        def run_sequence():
            waypoint_names = list(self.wp.names)
            total = len(waypoint_names)
            interrupted = False
            
            for idx, name in enumerate(waypoint_names):
                self.get_logger().info(f"=== Checkpoint {idx + 1}/{total}: {name} ===")
                
                # Verifica se hold foi ativado
                with self._lock:
                    if self._hold_enabled:
                        self._last_message = "Sequência interrompida por hold"
                        self.get_logger().warn(self._last_message)
                        interrupted = True
                        break
                
                # Inicia navegação
                ok, msg = self.start_checkpoint(name)
                if not ok:
                    self.get_logger().error(f"Falha ao iniciar navegação para {name}: {msg}")
                    continue
                
                # Espera navegação terminar
                while True:
                    with self._lock:
                        if not self._navigating:
                            break
                        if self._hold_enabled:
                            self._last_message = "Sequência interrompida por hold"
                            self.get_logger().warn(self._last_message)
                            interrupted = True
                            break
                    time.sleep(0.5)
                
                if interrupted:
                    break
                
                # Espera áudio terminar (a resposta do servidor Rust já é bloqueante)
                while True:
                    with self._lock:
                        if not self._playing_audio:
                            break
                        if self._hold_enabled:
                            self._last_message = "Sequência interrompida por hold"
                            self.get_logger().warn(self._last_message)
                            interrupted = True
                            break
                    time.sleep(0.5)
                
                if interrupted:
                    break
                
                # Verifica sucesso do áudio
                with self._lock:
                    if self._audio_success:
                        self.get_logger().info(f"Checkpoint {name} completado com sucesso!")
                    else:
                        self.get_logger().warn(f"Áudio do checkpoint {name} pode ter falhado, continuando...")
            
            # Toca áudio de stop ao final (seja interrompido ou completado)
            self.get_logger().info("=== Tocando áudio de parada/despedida ===")
            stop_success = self._call_stop_audio()
            
            if interrupted:
                with self._lock:
                    self._last_message = f"Sequência interrompida. Áudio de stop: {'OK' if stop_success else 'FALHOU'}"
            else:
                with self._lock:
                    self._last_message = f"Sequência de {total} checkpoints completada! Áudio de stop: {'OK' if stop_success else 'FALHOU'}"
            
            self.get_logger().info(self._last_message)
        
        thread = threading.Thread(target=run_sequence, daemon=True)
        thread.start()
        
        return True, f"Iniciando sequência de {self.wp.count()} checkpoints"

