// app/(tabs)/chatIntro.tsx
import React from "react";
import { Image, StyleSheet, Text, TouchableOpacity, View } from "react-native";
import { useRouter } from "expo-router";
import { BlurView } from "expo-blur";
import { useTour } from "@/context/TourContext";
import Logo from "../../assets/images/logo-branca.png";

export default function ChatIntro() {
  const router = useRouter();
  const { tour } = useTour();

  const visitorName = tour?.visitorName || "Visitante";

  const handleStartChat = () => {
    // 👉 Depois daqui ele vai direto pro chat e não volta pra intro
    router.push("/(tabs)/home");
  };

  return (
    <View style={styles.container}>
      {/* Header com logo */}
      <View style={styles.header}>
        <Image source={Logo} style={styles.logo} resizeMode="contain" />
      </View>

      {/* Conteúdo central */}
      <View style={styles.overlay}>
        <View style={styles.card}>
          <Text style={styles.smallTitle}>Bem-vindo ao tour do Inteli</Text>

          <Text style={styles.bigTitle}>Olá, {visitorName}! 👋</Text>

          <Text style={styles.text}>
            Aqui você poderá conversar com a{" "}
            <Text style={styles.highlight}>LIA</Text>, a inteligência do nosso
            robô.
          </Text>

          <Text style={styles.text}>
            Você pode enviar suas perguntas{" "}
            <Text style={styles.highlight}>por voz</Text> ou{" "}
            <Text style={styles.highlight}>por texto</Text> usando o botão
            principal da tela.
          </Text>

          <Text style={[styles.text, { marginTop: 8 }]}>
            As respostas serão mostradas ao longo do tour, principalmente ao
            final de cada etapa. Fique à vontade para testar e tirar suas
            dúvidas!
          </Text>

          <TouchableOpacity style={styles.buttonWrapper} onPress={handleStartChat}>
            <BlurView intensity={40} tint="dark" style={styles.buttonBlur}>
              <Text style={styles.buttonText}>Começar a usar o chat</Text>
            </BlurView>
          </TouchableOpacity>

          <Text style={styles.footerText}>
            Você verá essa tela só agora, no início do tour. Depois, poderá
            acessar o chat diretamente.
          </Text>
        </View>
      </View>
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
  overlay: {
    flex: 1,
    justifyContent: "center",
    alignItems: "center",
    backgroundColor: "rgba(30, 23, 48, 0.9)",
  },
  card: {
    width: "70%",
    maxWidth: 800,
    backgroundColor: "rgba(30, 23, 48, 0.95)",
    borderRadius: 24,
    paddingVertical: 40,
    paddingHorizontal: 32,
    alignItems: "center",
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.08)",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 6 },
    shadowOpacity: 0.3,
    shadowRadius: 10,
    elevation: 10,
  },
  smallTitle: {
    color: "rgba(255,255,255,0.7)",
    fontSize: 16,
    marginBottom: 8,
  },
  bigTitle: {
    color: "#fff",
    fontSize: 28,
    fontWeight: "700",
    marginBottom: 16,
    textAlign: "center",
  },
  text: {
    color: "rgba(255,255,255,0.9)",
    fontSize: 16,
    textAlign: "center",
    marginTop: 4,
  },
  highlight: {
    color: "#B794FF",
    fontWeight: "600",
  },
  buttonWrapper: {
    marginTop: 32,
    width: "70%",
    maxWidth: 360,
    borderRadius: 999,
    overflow: "hidden",
  },
  buttonBlur: {
    paddingVertical: 14,
    paddingHorizontal: 24,
    borderRadius: 999,
    backgroundColor: "rgba(106, 64, 196, 0.9)",
    borderWidth: 1,
    borderColor: "rgba(255,255,255,0.18)",
    alignItems: "center",
    justifyContent: "center",
  },
  buttonText: {
    color: "#fff",
    fontSize: 18,
    fontWeight: "700",
  },
  footerText: {
    marginTop: 18,
    color: "rgba(255,255,255,0.6)",
    fontSize: 13,
    textAlign: "center",
  },
});
