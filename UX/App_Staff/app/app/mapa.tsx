import {
  View,
  StyleSheet,
  Platform,
  UIManager,
  Text,
  ScrollView,
} from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";
import { Navbar } from "@/components/navbar";
import { Header } from "@/components/header";
import { useState, useEffect, useRef } from "react";
import { Pergunta } from "@/components/Pergunta";
import Checkpoint from "@/components/Checkpoint";
import { AlertButton } from "@/components/AlertButton";
import AlertPopup from "@/components/AlertPopup";

if (
  Platform.OS === "android" &&
  UIManager.setLayoutAnimationEnabledExperimental
) {
  UIManager.setLayoutAnimationEnabledExperimental(true);
}

export type Tour = {
  codigo: string;
  responsavel: string;
  status: "scheduled" | "in_progress" | "paused" | "finished" | "cancelled";
  data: string;
  hora_inicio_prevista: string;
  hora_fim_prevista: string;
};

type WsPergunta = {
  id_pergunta?: number;
  texto_pergunta?: string;
  checkpoint?: number;
  texto_resposta?: string | null;
};

type WsMessage = {
  status?: string;
  tour_id?: number;
  codigo?: string;
  perguntas?: WsPergunta[];
};

const WS_URL = "ws://10.140.0.11:8080/v1/ws/tour/check";

export default function MapScreen() {
  const socketRef = useRef<WebSocket | null>(null);
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const [isNow, setIsNow] = useState(false);
  const [tourId, setTourId] = useState<number | null>(null);
  const [perguntas, setPerguntas] = useState<
    {
      id: number | undefined;
      pergunta: string;
      local: string;
      resposta: string;
    }[]
  >([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [alert, setAlert] = useState(false);

  const buildPayload = () => {
    const now = new Date();
    const horario_verificacao = now.toLocaleTimeString("pt-BR", {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      hour12: false,
    });
    const data_local = now.toISOString().split("T")[0];
    return { data_local, horario_verificacao };
  };

  const sendCurrentPayload = () => {
    const ws = socketRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    const payload = buildPayload();
    ws.send(JSON.stringify(payload));
  };

  const handleWsMessage = (raw: string) => {
    try {
      const parsed: WsMessage = JSON.parse(raw);
      if (parsed.status === "active") {
        setIsNow(true);
        setTourId(parsed.tour_id ?? null);
        const perguntasMapeadas =
          parsed.perguntas?.map((p) => ({
            id: p.id_pergunta,
            pergunta: p.texto_pergunta ?? "",
            local: p.checkpoint ? `Checkpoint ${p.checkpoint}` : "Checkpoint",
            resposta: p.texto_resposta ?? "Sem resposta ainda.",
          })) ?? [];
        setPerguntas(perguntasMapeadas);
        setError(null);
      } else if (parsed.status === "inactive") {
        setIsNow(false);
        setTourId(null);
        setPerguntas([]);
        setError(null);
      } else {
        setError("Resposta inesperada do servidor.");
        setIsNow(false);
        setTourId(null);
        setPerguntas([]);
      }
    } catch (err) {
      console.warn("Erro ao parsear mensagem WS", err);
      setError("Não foi possível interpretar a resposta do servidor.");
      setIsNow(false);
      setTourId(null);
      setPerguntas([]);
    }
  };

  const connectAndSend = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }

    setLoading(true);
    setError(null);

    try {
      // Fecha conexão anterior, se houver
      if (socketRef.current) {
        socketRef.current.close();
      }

      const ws = new WebSocket(WS_URL);
      socketRef.current = ws;

      ws.onopen = () => {
        sendCurrentPayload();
        intervalRef.current = setInterval(() => {
          sendCurrentPayload();
        }, 5000);
      };

      ws.onmessage = (event) => {
        handleWsMessage(event.data);
      };

      ws.onerror = () => {
        setError("Falha na conexão WebSocket.");
      };

      ws.onclose = () => {
        // noop
      };
    } catch (err) {
      console.error(err);
      setError("Não foi possível conectar ao WebSocket.");
    }
    setLoading(false);
  };

  useEffect(() => {
    connectAndSend();
    return () => {
      socketRef.current?.close();
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, []);

  //const tourAtual = getTourDoDia(filteredTours);

  return (
    <SafeAreaView style={styles.container}>
      <Header />

      <ScrollView
        style={{ width: "100%" }}
        contentContainerStyle={{
          flexDirection: "column",
          gap: 12,
          justifyContent: "center",
          alignItems: "center",
          paddingTop: 120,
          paddingBottom: 140,
        }}
        showsVerticalScrollIndicator={false}
        scrollIndicatorInsets={{ right: 1 }}
      >
        {loading ? (
          <Text style={{ color: "#FFF", fontSize: 16 }}>Carregando...</Text>
        ) : error ? (
          <View style={styles.status_atual}>
            <Text style={{ color: "#FFF" }}>{error}</Text>
          </View>
        ) : isNow ? (
          <View
            style={{
              width: "100%",
              alignItems: "center",
              justifyContent: "center",
            }}
          >
            <View style={styles.checkpointsContainer}>
              <Checkpoint id={1} status={"done"} label={"recepção"} />
              <Checkpoint id={2} status={"in_progress"} label={"auditório"} />
              <Checkpoint id={3} status={"not_started"} label={"ateliê"} />
              <Checkpoint id={4} status={"not_started"} label={"casinhas"} />
              <Checkpoint id={5} status={"not_started"} label={"LIA house"} />
            </View>

            <View style={styles.perguntasHeader}>
              <Text style={styles.text}>Perguntas Feitas</Text>
            </View>
            <View
              style={{
                width: "100%",
                justifyContent: "center",
                alignItems: "center",
                gap: 4,
                paddingTop: 15,
              }}
            >
              {perguntas.length === 0 ? (
                <Text style={{ color: "#FFF" }}>
                  Nenhuma pergunta registrada.
                </Text>
              ) : (
                perguntas.map((p) => (
                  <Pergunta
                    key={p.id ?? `${p.pergunta}-${p.local}`}
                    pergunta={p.pergunta}
                    local={p.local}
                    resposta={p.resposta}
                  />
                ))
              )}
            </View>
          </View>
        ) : (
          <View style={styles.status_atual}>
            <Text style={{ color: "#FFF" }}>
              Nenhum tour em andamento no momento.
            </Text>
          </View>
        )}
      </ScrollView>

      {alert && <AlertPopup onClose={() => setAlert(false)} tourId={tourId} />}

      {isNow && <AlertButton onOpen={() => setAlert(true)} />}
      <Navbar />
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#201A2C",
    justifyContent: "center",
    alignItems: "center",
  },

  tour: {
    flexDirection: "row",
    alignItems: "center",
    borderWidth: 1,
    borderRadius: 60,
    borderColor: "#FFBB00",
    paddingVertical: 9,
    paddingHorizontal: 18,
    maxHeight: 40,
    gap: 4,
  },

  status_atual: {
    borderWidth: 1,
    borderColor: "#402A78",
    borderRadius: 8,
    paddingVertical: 18,
    alignItems: "center",
    marginVertical: 12,
  },
  text: {
    fontSize: 16,
    fontWeight: "700",
    color: "#FFF",
  },
  perguntasHeader: {
    width: "95%",
    marginTop: 15,
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
  },
  checkpointsContainer: {
    marginVertical: 20,
    width: "95%",
    alignItems: "center",
    flexDirection: "row",
    flexWrap: "nowrap",
    justifyContent: "space-between",
  },
});
