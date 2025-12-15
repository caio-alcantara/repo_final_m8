// app/(tabs)/home.tsx
import React, { useEffect, useState } from "react";
import { Image, View, StyleSheet } from "react-native";
import { useLocalSearchParams } from "expo-router";

import Logo from "../../assets/images/logo-branca.png";
import ChatArea, { ChatMessage } from "../../components/chatArea";
import VoiceButton from "../../components/VoiceButton";
import Navbar from "@/components/navbar";

import { useTour } from "@/context/TourContext";
import {
  getHistoricoChat,
  getRespostaByPerguntaId,
  askModelo,
  Pergunta,
  Resposta,
} from "@/api/chatService";

export default function Home() {
  const { tourId: tourIdParam } = useLocalSearchParams<{ tourId?: string }>();

  const { tour } = useTour();

  const [messages, setMessages] = useState<ChatMessage[]>([]);

  const numericTourId: number | null =
    tour?.tourId ?? (tourIdParam ? Number(tourIdParam) : null);

  const numericCheckpointId: number | null = tour?.checkpointId ?? 4;

  const formatTime = (iso: string | null | undefined) => {
    if (!iso) {
      return new Date().toLocaleTimeString("pt-BR", {
        hour: "2-digit",
        minute: "2-digit",
      });
    }

    return new Date(iso).toLocaleTimeString("pt-BR", {
      hour: "2-digit",
      minute: "2-digit",
    });
  };

  const getCurrentTime = () =>
    new Date().toLocaleTimeString("pt-BR", {
      hour: "2-digit",
      minute: "2-digit",
    });

 
  useEffect(() => {
    const loadHistory = async () => {
      try {
        if (!numericTourId) {
          console.log(
            "⚠️ [CHAT] Sem tourId ainda, não dá pra carregar histórico."
          );
          return;
        }

        console.log("📜 [CHAT] Carregando histórico bruto (todas as perguntas)...");
        const historico: Pergunta[] = await getHistoricoChat();

        const msgs: ChatMessage[] = [];

        msgs.push({
          id: "welcome",
          text: "Oi! Eu sou a LIA! Tem alguma dúvida sobre o",
          time: getCurrentTime(),
          side: "left",
          renderMarkdown: true,
        });

        const perguntasDoTourAtual = historico.filter(
          (p) => p.tour_id === numericTourId
        );

        console.log(
          `📌 [CHAT] Encontradas ${perguntasDoTourAtual.length} perguntas do tour ${numericTourId}`
        );

        for (const pergunta of perguntasDoTourAtual) {
          msgs.push({
            id: `q-${pergunta.id}`,
            text: pergunta.texto,
            time: formatTime(pergunta.criado_em),
            side: "right",
          });

          try {
            const resposta: Resposta | null = await getRespostaByPerguntaId(
              pergunta.id
            );

            if (resposta) {
              msgs.push({
                id: `r-${resposta.id}`,
                text: resposta.texto,
                time: formatTime(resposta.criado_em),
                side: "left",
                renderMarkdown: true,
              });
            } else {
              msgs.push({
                id: `r-pending-${pergunta.id}`,
                text: "Essa pergunta ainda não foi respondida pela LIA.",
                time: formatTime(null),
                side: "left",
                status: "pending",
              });
            }
          } catch (err) {
            console.log(
              `⚠️ [CHAT] Erro ao buscar resposta da pergunta ${pergunta.id}:`,
              err
            );
            msgs.push({
              id: `r-error-${pergunta.id}`,
              text:
                "Tivemos um problema ao carregar a resposta desta pergunta. Ela pode aparecer aqui em breve.",
              time: formatTime(null),
              side: "left",
              status: "error",
            });
          }
        }

        setMessages(msgs);
      } catch (err) {
        console.error("Erro ao carregar histórico:", err);

        setMessages([
          {
            id: "error-load",
            text:
              "Não consegui carregar o histórico agora, mas você já pode me enviar perguntas normalmente.",
            time: getCurrentTime(),
            side: "left",
            status: "error",
          },
        ]);
      }
    };

    loadHistory();
  }, [numericTourId]);


  const sendQuestionToBackend = async (userText: string) => {
    console.log("🧠 [sendQuestionToBackend] Chamado com texto:", userText);
    console.log("🧩 [sendQuestionToBackend] IDs atuais:", {
      numericTourId,
      numericCheckpointId,
    });

    try {
      if (!numericTourId) {
        console.warn(
          "⚠️ [sendQuestionToBackend] Sem tourId, não tem como criar pergunta."
        );
        const errorMessage: ChatMessage = {
          id: `no-tour-${Date.now()}`,
          text:
            "Não encontrei o tour atual. Volte à tela inicial e entre novamente com o código, por favor. 🙏",
          time: getCurrentTime(),
          side: "left",
          status: "error",
        };
        setMessages((prev) => [...prev, errorMessage]);
        return;
      }

      // 1️⃣ tenta usar o checkpoint do contexto
      let checkpointIdToUse: number | null = numericCheckpointId;

      // 2️⃣ se for null, tenta inferir pelo histórico
      if (!checkpointIdToUse) {
        console.log(
          "[CHAT] checkpointId não definido, tentando inferir via histórico..."
        );
        try {
          const historico: Pergunta[] = await getHistoricoChat();
          const perguntasDoTourAtual = historico.filter(
            (p) => p.tour_id === numericTourId
          );

          if (perguntasDoTourAtual.length > 0) {
            const ultimaPergunta =
              perguntasDoTourAtual[perguntasDoTourAtual.length - 1];
            checkpointIdToUse = ultimaPergunta.checkpoint_id;
            console.log(
              "[CHAT] checkpoint_id inferido do histórico:",
              checkpointIdToUse
            );
          } else {
            console.log(
              "[CHAT] Nenhuma pergunta anterior encontrada para este tour."
            );
          }
        } catch (err) {
          console.error(
            "[CHAT] Erro ao tentar inferir checkpoint via histórico:",
            err
          );
        }
      }

      // 3️⃣ se mesmo assim não tiver checkpoint, avisa e não chama o backend
      if (!checkpointIdToUse) {
        console.warn(
          "⚠️ [sendQuestionToBackend] Continua sem checkpointId, não vou chamar o modelo."
        );
        const errorMessage: ChatMessage = {
          id: `no-checkpoint-${Date.now()}`,
          text:
            "Não encontrei o checkpoint atual para este tour. Avise um monitor, por favor. 🙏",
          time: getCurrentTime(),
          side: "left",
          status: "error",
        };
        setMessages((prev) => [...prev, errorMessage]);
        return;
      }

      // 4️⃣ Chama o modelo (porta 8000) - integração definitiva 🎯
      const resposta = await askModelo({
        tour_id: numericTourId,
        checkpoint_id: checkpointIdToUse,
        question_topic: null,
        texto: userText,
        estado: "queued",
        liberado_em: null,
        respondido_em: null,
      });

      console.log("✅ [sendQuestionToBackend] Resposta do modelo:", resposta);

      const botMessage: ChatMessage = {
        id: `bot-${Date.now()}`,
        text: resposta.texto,
        time: formatTime(resposta.criado_em),
        side: "left",
        renderMarkdown: true,
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      console.error("Erro ao falar com o backend/modelo:", error);

      const errorMessage: ChatMessage = {
        id: `error-${Date.now()}`,
        text:
          "Tive um problema para falar com a LIA agora. Pode tentar de novo daqui a pouco? 🙏",
        time: getCurrentTime(),
        side: "left",
        status: "error",
      };

      setMessages((prev) => [...prev, errorMessage]);
    }
  };

  const handleSendText = (text: string) => {
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      text,
      time: getCurrentTime(),
      side: "right",
    };

    setMessages((prev) => [...prev, userMessage]);
    sendQuestionToBackend(text);
  };

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Image source={Logo} style={styles.logo} resizeMode="contain" />
      </View>

      <View style={styles.body}>
        <View style={styles.leftPane}>
          <ChatArea messages={messages} />
        </View>

        <View style={styles.rightPane}>
          <VoiceButton onSendText={handleSendText} />
        </View>
      </View>

      <Navbar />
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#1E1730",
  },
  header: {
    alignItems: "center",
    marginTop: 40,
    marginBottom: 20,
  },
  logo: {
    width: 120,
    height: 100,
  },
  body: {
    flex: 1,
    flexDirection: "row",
    paddingHorizontal: 16,
    paddingBottom: 16,
  },
  leftPane: {
    flex: 1.5,
    marginRight: 12,
  },
  rightPane: {
    flex: 1,
    alignItems: "center",
    justifyContent: "center",
  },
});
