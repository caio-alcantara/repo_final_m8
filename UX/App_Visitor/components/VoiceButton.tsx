import React, { useRef, useEffect, useState } from "react";

import {
  View,
  TouchableOpacity,
  StyleSheet,
  Animated,
  Easing,
  TextInput,
  type TextInput as RNTextInput,
  Alert,
} from "react-native";

import { Ionicons } from "@expo/vector-icons";

import { Audio, type RecordingStatus, type Recording } from "expo-av";

import * as FileSystem from "expo-file-system/legacy";

const STT_WS_URL = "ws://10.140.0.11:5000/stt";

const BARS_COUNT = 24;

const MIN_HEIGHT = 6;

const MAX_EXTRA_HEIGHT = 50;

const BAR_CONFIGS = Array.from({ length: BARS_COUNT }, (_, i) => {
  const t = i / (BARS_COUNT - 1);

  const base = Math.sin(t * Math.PI);

  return {
    base,

    delay: i * 60,
  };
});

function SiriWaveform({ level }: { level: number }) {
  const barAnims = useRef(BAR_CONFIGS.map(() => new Animated.Value(0))).current;

  useEffect(() => {
    barAnims.forEach((anim, index) => {
      const { delay } = BAR_CONFIGS[index];

      Animated.loop(
        Animated.sequence([
          Animated.delay(delay),

          Animated.timing(anim, {
            toValue: 1,

            duration: 900,

            easing: Easing.inOut(Easing.sin),

            useNativeDriver: true,
          }),

          Animated.timing(anim, {
            toValue: 0,

            duration: 900,

            easing: Easing.inOut(Easing.sin),

            useNativeDriver: true,
          }),
        ])
      ).start();
    });
  }, [barAnims]);

  return (
    <View style={styles.waveformContainer}>
      {barAnims.map((anim, index) => {
        const cfg = BAR_CONFIGS[index];

        const baseHeight =
          MIN_HEIGHT + cfg.base * MAX_EXTRA_HEIGHT * (0.3 + level * 0.7);

        const scaleY = anim.interpolate({
          inputRange: [0, 1],

          outputRange: [0.5, 1.5],
        });

        return (
          <Animated.View
            key={index}
            style={[
              styles.waveBar,

              {
                height: baseHeight,

                transform: [{ scaleY }],
              },
            ]}
          />
        );
      })}
    </View>
  );
}

type InputMode = "voice" | "text";

type VoiceButtonProps = {
  onSendText?: (text: string) => void;
};

export default function VoiceButton({ onSendText }: VoiceButtonProps) {
  const [level, setLevel] = useState(0);

  const recordingRef = useRef<Recording | null>(null);

  const [isRecording, setIsRecording] = useState(false);

  const [hasPermission, setHasPermission] = useState<boolean | null>(null);

  const [inputMode, setInputMode] = useState<InputMode>("voice");

  const [text, setText] = useState("");

  const [isTranscribing, setIsTranscribing] = useState(false);

  const inputRef = useRef<RNTextInput | null>(null);

  const modeAnim = useRef(new Animated.Value(0)).current;

  useEffect(() => {
    (async () => {
      const { status } = await Audio.requestPermissionsAsync();

      setHasPermission(status === "granted");
    })();
  }, []);

  const animateMode = (toValue: 0 | 1) => {
    Animated.timing(modeAnim, {
      toValue,

      duration: 220,

      easing: Easing.out(Easing.quad),

      useNativeDriver: true,
    }).start();
  };

  const startRecording = async () => {
    if (hasPermission === false) {
      Alert.alert(
        "Permissão negada",

        "Ative o microfone nas configurações para usar a entrada por voz."
      );

      return;
    }

    try {
      await Audio.setAudioModeAsync({
        allowsRecordingIOS: true,

        playsInSilentModeIOS: true,
      });

      const { recording } = await Audio.Recording.createAsync(
        Audio.RecordingOptionsPresets.HIGH_QUALITY,

        (status: RecordingStatus) => {
          if (!status.isRecording) return;

          const anyStatus = status as any;

          const metering = anyStatus.metering;

          if (typeof metering === "number") {
            const minDb = -60;

            const clamped = Math.max(metering, minDb);

            const normalized = (clamped - minDb) / -minDb;

            setLevel(normalized);
          } else {
            // fallback se metering não existir (alguns Android)

            setLevel((prev) => {
              const next = prev + (Math.random() * 0.4 - 0.2);

              return Math.max(0, Math.min(1, next));
            });
          }
        },

        100
      );

      recordingRef.current = recording;

      setIsRecording(true);
    } catch (e) {
      console.error("Erro ao iniciar gravação:", e);
    }
  };

  const transcribeAudio = async (uri: string): Promise<string | null> => {
    try {
      // 1) Ler o arquivo como base64 (sem usar EncodingType)

      const base64Audio = await FileSystem.readAsStringAsync(uri, {
        encoding: "base64" as any, // evita o acesso FileSystem.EncodingType.Base64
      });

      console.log("📦 Áudio em base64, tamanho:", base64Audio.length);

      // 2) Enviar via WebSocket e esperar resposta

      const texto = await new Promise<string | null>((resolve, reject) => {
        try {
          console.log("🌐 Conectando ao WebSocket STT:", STT_WS_URL);

          const ws = new WebSocket(STT_WS_URL);

          ws.onopen = () => {
            console.log("✅ WebSocket conectado, enviando áudio...");

            ws.send(base64Audio);
          };

          ws.onmessage = (event) => {
            console.log("📨 Resposta do servidor STT:", event.data);

            const msg = String(event.data || "").trim();

            ws.close();

            resolve(msg || null);
          };

          ws.onerror = (event) => {
            console.error("❌ Erro no WebSocket STT:", event);

            ws.close();

            reject(new Error("Erro no WebSocket STT"));
          };

          ws.onclose = (event) => {
            console.log("🔒 WebSocket fechado:", event.code, event.reason);
          };
        } catch (err) {
          console.error("Erro ao criar WebSocket:", err);

          reject(err);
        }
      });

      return texto;
    } catch (e) {
      console.error("Erro ao transcrever áudio:", e);

      return null;
    }
  };

  const stopRecording = async () => {
    const recording = recordingRef.current;

    if (!recording) return;

    try {
      await recording.stopAndUnloadAsync();

      const uri = recording.getURI();

      setIsRecording(false);

      setLevel(0);

      if (uri && onSendText) {
        setIsTranscribing(true);

        const transcript = await transcribeAudio(uri);

        setIsTranscribing(false);

        if (transcript && transcript.trim().length > 0) {
          onSendText(transcript.trim()); // 👉 aparece como mensagem do usuário
        } else {
          console.log("Nenhuma transcrição retornada.");
        }
      }
    } catch (e) {
      console.warn("Erro ao parar gravação:", e);

      setIsRecording(false);

      setIsTranscribing(false);

      setLevel(0);
    } finally {
      recordingRef.current = null;
    }
  };

  const toggleRecording = () => {
    if (isTranscribing) return; // evita apertar enquanto transcreve

    if (isRecording) {
      stopRecording();
    } else {
      startRecording();
    }
  };

  const handleMainPress = () => {
    if (inputMode === "voice") {
      toggleRecording();
    } else {
      inputRef.current?.focus();
    }
  };

  const handleModeToggle = () => {
    if (inputMode === "voice") {
      if (isRecording) stopRecording();

      setInputMode("text");

      animateMode(1);

      setTimeout(() => inputRef.current?.focus(), 250);
    } else {
      setInputMode("voice");

      animateMode(0);

      setLevel(0);
    }
  };

  const handleSend = () => {
    const trimmed = text.trim();

    if (!trimmed) return;

    if (onSendText) {
      onSendText(trimmed); // texto digitado
    }

    setText("");
  };

  const micIconStyle = {
    opacity: modeAnim.interpolate({
      inputRange: [0, 1],

      outputRange: [1, 0],
    }),

    transform: [
      {
        translateY: modeAnim.interpolate({
          inputRange: [0, 1],

          outputRange: [0, -6],
        }),
      },
    ],
  };

  const textIconStyle = {
    opacity: modeAnim.interpolate({
      inputRange: [0, 1],

      outputRange: [0, 1],
    }),

    transform: [
      {
        translateY: modeAnim.interpolate({
          inputRange: [0, 1],

          outputRange: [6, 0],
        }),
      },
    ],
  };

  return (
    <View style={styles.container}>
      <View style={styles.switchWrapper}>
        <Animated.View
          style={[
            styles.modeToggleWrapper,

            {
              transform: [],
            },
          ]}
        >
          <TouchableOpacity
            style={[
              styles.modeToggle,

              inputMode === "text" && styles.modeToggleActive,
            ]}
            onPress={handleModeToggle}
            disabled={isTranscribing}
          >
            <Ionicons
              name={inputMode === "voice" ? "text-outline" : "mic-outline"}
              size={18}
              color="#fff"
            />
          </TouchableOpacity>
        </Animated.View>

        <TouchableOpacity
          style={styles.button}
          onPress={handleMainPress}
          disabled={isTranscribing}
        >
          <Animated.View style={[styles.iconLayer, micIconStyle]}>
            <Ionicons
              name={isRecording ? "mic" : "mic-outline"}
              size={38}
              color="#fff"
            />
          </Animated.View>

          <Animated.View style={[styles.iconLayer, textIconStyle]}>
            <Ionicons
              name="chatbubble-ellipses-outline"
              size={34}
              color="#fff"
            />
          </Animated.View>
        </TouchableOpacity>
      </View>

      {inputMode === "voice" && !isTranscribing && (
        <View style={styles.waveWrapper}>
          <SiriWaveform level={level} />
        </View>
      )}

      {inputMode === "text" && (
        <View style={styles.inputWrapper}>
          <TextInput
            ref={inputRef}
            style={styles.input}
            value={text}
            onChangeText={setText}
            placeholder="Digite sua mensagem..."
            placeholderTextColor="#C7C1E0"
            returnKeyType="send"
            onSubmitEditing={handleSend}
          />

          <TouchableOpacity style={styles.sendButton} onPress={handleSend}>
            <Ionicons name="send" size={20} color="#fff" />
          </TouchableOpacity>
        </View>
      )}

      {isTranscribing && (
        <View style={{ marginTop: 12 }}>
          <Ionicons name="ellipsis-horizontal" size={24} color="#C7C1E0" />
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    justifyContent: "center",

    alignItems: "center",

    transform: [{ scale: 1.6 }],
  },

  switchWrapper: {
    position: "relative",

    justifyContent: "center",

    alignItems: "center",
  },

  button: {
    width: 80,

    height: 80,

    borderRadius: 40,

    backgroundColor: "#6A40C4",

    alignItems: "center",

    justifyContent: "center",

    borderWidth: 2,

    borderColor: "rgba(255,255,255,0.2)",

    overflow: "hidden",
  },

  iconLayer: {
    position: "absolute",

    width: "100%",

    height: "100%",

    alignItems: "center",

    justifyContent: "center",
  },

  modeToggleWrapper: {
    position: "absolute",

    right: -12,

    bottom: -10,
  },

  modeToggle: {
    width: 40,

    height: 40,

    borderRadius: 20,

    backgroundColor: "#4D2AA8",

    alignItems: "center",

    justifyContent: "center",

    borderWidth: 1,

    borderColor: "rgba(255,255,255,0.4)",
  },

  modeToggleActive: {
    backgroundColor: "#7A52FF",
  },

  waveWrapper: {
    marginTop: 30,
  },

  waveformContainer: {
    flexDirection: "row",

    alignItems: "center",

    height: MAX_EXTRA_HEIGHT + MIN_HEIGHT,
  },

  waveBar: {
    width: 4,

    marginHorizontal: 2,

    borderRadius: 999,

    backgroundColor: "#9B6DFF",
  },

  inputWrapper: {
    marginTop: 18,

    flexDirection: "row",

    width: 300,

    height: 50,

    alignItems: "center",

    paddingHorizontal: 14,

    paddingVertical: 10,

    borderRadius: 999,

    backgroundColor: "rgba(38, 26, 92, 0.9)",

    borderWidth: 1,

    borderColor: "rgba(255,255,255,0.18)",
  },

  input: {
    flex: 1,
    color: "#fff",
    fontSize: 14,
    paddingVertical: 0,
    marginRight: 8,
  },

  sendButton: {
    paddingHorizontal: 4,
    paddingVertical: 2,
  },
});
