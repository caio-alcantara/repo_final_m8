#!/usr/bin/env python3
"""
Cliente de teste para o WebSocket do backend Rust.
Simula o frontend para testar a integração completa.
"""

import asyncio
import json
import websockets
from datetime import datetime

WS_URL = "ws://localhost:8080/ws"

class BackendWebSocketClient:
    def __init__(self):
        self.ws = None
        
    async def connect(self):
        """Conecta ao backend Rust"""
        print(f"🔌 Conectando ao backend em {WS_URL}...")
        try:
            self.ws = await websockets.connect(WS_URL)
            print("✅ Conectado ao backend Rust!")
            return True
        except Exception as e:
            print(f"❌ Erro ao conectar: {e}")
            return False
    
    async def listen(self):
        """Escuta eventos do backend"""
        print("📡 Aguardando eventos do backend...\n")
        
        try:
            async for message in self.ws:
                data = json.loads(message)
                self.handle_event(data)
        except websockets.exceptions.ConnectionClosed:
            print("🔌 Conexão fechada pelo servidor")
        except Exception as e:
            print(f"❌ Erro ao receber mensagem: {e}")
    
    def handle_event(self, data):
        """Processa eventos recebidos do backend"""
        event = data.get("event")
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        print(f"[{timestamp}] 📥 Evento recebido: {event}")
        print(f"   Dados: {json.dumps(data, indent=2)}\n")
        
        if event == "robot_status":
            connected = "🟢 ONLINE" if data.get("robot_connected") else "🔴 OFFLINE"
            running = "⏳ EXECUTANDO" if data.get("is_running") else "⏸️ PARADO"
            checkpoint = data.get("current_checkpoint") or "Nenhum"
            
            print(f"   🤖 Robô: {connected}")
            print(f"   📊 Estado: {running}")
            print(f"   📍 Checkpoint atual: {checkpoint}\n")
            
        elif event == "checkpoint_started":
            tipo = data.get("tipo")
            ordem = data.get("ordem")
            inicio = data.get("inicio_real")
            
            print(f"   ▶️ CHECKPOINT INICIADO!")
            print(f"   📍 Local: {tipo}")
            print(f"   🔢 Ordem: {ordem}")
            print(f"   ⏰ Início: {inicio}\n")
            
        elif event == "checkpoint_completed":
            tipo = data.get("tipo")
            ordem = data.get("ordem")
            status = data.get("status")
            fim = data.get("fim_real")
            
            emoji = "✅" if status == "finished" else "⏭️"
            status_text = "CONCLUÍDO" if status == "finished" else "PULADO"
            
            print(f"   {emoji} CHECKPOINT {status_text}!")
            print(f"   📍 Local: {tipo}")
            print(f"   🔢 Ordem: {ordem}")
            print(f"   ⏰ Fim: {fim}\n")
            
        elif event == "emergency_stop":
            tipo = data.get("tipo") or "Desconhecido"
            ordem = data.get("ordem") or "?"
            
            print(f"   🛑 PARADA DE EMERGÊNCIA!")
            print(f"   📍 Checkpoint afetado: {tipo} (ordem {ordem})\n")
            
        elif event == "error":
            message = data.get("message")
            print(f"   ❌ ERRO: {message}\n")
    
    async def send_command(self, action):
        """Envia comando para o backend"""
        if not self.ws:
            print("❌ WebSocket não conectado")
            return
        
        command = {"action": action}
        print(f"📤 Enviando comando: {action}")
        
        try:
            await self.ws.send(json.dumps(command))
            print(f"✅ Comando '{action}' enviado com sucesso\n")
        except Exception as e:
            print(f"❌ Erro ao enviar comando: {e}\n")
    
    async def close(self):
        """Fecha a conexão"""
        if self.ws:
            await self.ws.close()
            print("👋 Desconectado do backend")


async def interactive_mode():
    """Modo interativo para testar comandos"""
    client = BackendWebSocketClient()
    
    if not await client.connect():
        return
    
    # Task para receber eventos em background
    listen_task = asyncio.create_task(client.listen())
    
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("🎮 Modo Interativo - Comandos disponíveis:")
    print("   - play    : Executar próximo checkpoint")
    print("   - stop    : Parar robô")
    print("   - status  : Consultar status do robô")
    print("   - quit    : Sair")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")
    
    try:
        while True:
            cmd = await asyncio.get_event_loop().run_in_executor(
                None, 
                input, 
                "Comando > "
            )
            
            cmd = cmd.strip().lower()
            
            if cmd == "quit":
                print("\n👋 Encerrando...")
                break
            elif cmd in ["play", "stop", "status"]:
                await client.send_command(cmd if cmd != "status" else "get_status")
            elif cmd == "":
                continue
            else:
                print(f"❌ Comando desconhecido: {cmd}\n")
    
    except KeyboardInterrupt:
        print("\n\n👋 Interrompido pelo usuário")
    finally:
        listen_task.cancel()
        await client.close()


async def automated_test():
    """Teste automatizado completo"""
    client = BackendWebSocketClient()
    
    if not await client.connect():
        return
    
    # Task para receber eventos
    listen_task = asyncio.create_task(client.listen())
    
    print("🧪 TESTE AUTOMATIZADO INICIADO\n")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")
    
    try:
        # Aguardar conexão estabilizar
        await asyncio.sleep(2)
        
        # 1. Consultar status inicial
        print("📊 TESTE 1: Consultando status inicial...")
        await client.send_command("get_status")
        await asyncio.sleep(3)
        
        # 2. Tentar executar checkpoint
        print("▶️ TESTE 2: Executando checkpoint...")
        await client.send_command("play")
        await asyncio.sleep(5)
        
        # 3. Parar execução
        print("🛑 TESTE 3: Parando robô...")
        await client.send_command("stop")
        await asyncio.sleep(3)
        
        # 4. Consultar status final
        print("📊 TESTE 4: Consultando status final...")
        await client.send_command("get_status")
        await asyncio.sleep(3)
        
        print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print("✅ TESTE AUTOMATIZADO CONCLUÍDO")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")
        
    except KeyboardInterrupt:
        print("\n\n👋 Teste interrompido")
    finally:
        listen_task.cancel()
        await client.close()


async def listen_only():
    """Apenas escuta eventos (não envia comandos)"""
    client = BackendWebSocketClient()
    
    if not await client.connect():
        return
    
    print("👂 MODO ESCUTA - Aguardando eventos...\n")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")
    print("💡 Dica: Execute comandos no robô Python para ver os eventos aqui!\n")
    
    try:
        await client.listen()
    except KeyboardInterrupt:
        print("\n\n👋 Interrompido")
    finally:
        await client.close()


async def main():
    """Menu principal"""
    import sys
    
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        
        if mode == "interactive" or mode == "i":
            await interactive_mode()
        elif mode == "auto" or mode == "a":
            await automated_test()
        elif mode == "listen" or mode == "l":
            await listen_only()
        else:
            print(f"❌ Modo desconhecido: {mode}")
            print_usage()
    else:
        print_usage()


def print_usage():
    """Mostra instruções de uso"""
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("🧪 Cliente de Teste - WebSocket Backend Rust")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")
    print("Uso:")
    print("  python test_websocket.py <modo>\n")
    print("Modos disponíveis:")
    print("  interactive (i)  - Modo interativo com comandos")
    print("  auto (a)        - Teste automatizado completo")
    print("  listen (l)      - Apenas escuta eventos\n")
    print("Exemplos:")
    print("  python test_websocket.py interactive")
    print("  python test_websocket.py auto")
    print("  python test_websocket.py listen\n")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Encerrando...")
