import { StatusBar } from "expo-status-bar";
import React from "react";
import { Image, StyleSheet, Text, TouchableOpacity, View } from "react-native";
import { Ionicons } from "@expo/vector-icons";
import { useRouter } from "expo-router";

const Logo = require("../../assets/images/logo-branca.png");

export default function Confirmacao() {
  const router = useRouter();
  
  const handleConfirm = () => {
    // Aqui você pode adicionar a lógica para continuar o tour
    console.log("Tour confirmado para próxima etapa");
    router.push("/(tabs)/mapa");
  };

  return (
    <>
      <StatusBar hidden />
      <View style={styles.container}>
        {/* Logo do Inteli no topo */}
        <View style={styles.header}>
          <Image source={Logo} style={styles.logo} resizeMode="contain" />
        </View>

        {/* Conteúdo principal */}
        <View style={styles.content}>
          {/* Ícone de checkpoint */}
          <View style={styles.iconContainer}>
            <View style={styles.iconCircle}>
              <Ionicons name="checkmark-circle" size={120} color="#00E676" />
            </View>
          </View>

          {/* Texto de confirmação */}
          <Text style={styles.title}>Checkpoint Alcançado!</Text>
          <Text style={styles.subtitle}>
            Já podemos prosseguir para a próxima etapa do tour?
          </Text>

          {/* Botão de confirmação */}
          <TouchableOpacity style={styles.confirmButton} onPress={handleConfirm}>
            <Text style={styles.confirmButtonText}>Sim, vamos continuar!</Text>
            <Ionicons name="arrow-forward" size={24} color="#FFFFFF" />
          </TouchableOpacity>
        </View>
      </View>
    </>
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
  content: {
    flex: 1,
    alignItems: "center",
    justifyContent: "center",
    paddingHorizontal: 30,
    paddingBottom: 60,
  },
  iconContainer: {
    marginBottom: 40,
  },
  iconCircle: {
    backgroundColor: "rgba(0, 230, 118, 0.1)",
    borderRadius: 100,
    padding: 20,
    borderWidth: 3,
    borderColor: "#00E676",
    shadowColor: "#00E676",
    shadowOffset: { width: 0, height: 0 },
    shadowOpacity: 0.6,
    shadowRadius: 20,
    elevation: 10,
  },
  title: {
    fontSize: 28,
    fontWeight: "bold",
    color: "#FFFFFF",
    textAlign: "center",
    marginBottom: 15,
  },
  subtitle: {
    fontSize: 20,
    color: "#FFFFFF",
    textAlign: "center",
    marginBottom: 50,
    lineHeight: 28,
  },
  confirmButton: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    backgroundColor: "#6A40C4",
    paddingVertical: 18,
    paddingHorizontal: 40,
    borderRadius: 30,
    shadowColor: "#6A40C4",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.5,
    shadowRadius: 10,
    elevation: 8,
    gap: 10,
    minWidth: 280,
  },
  confirmButtonText: {
    fontSize: 18,
    fontWeight: "bold",
    color: "#FFFFFF",
  },
  infoText: {
    fontSize: 14,
    color: "#9E9E9E",
    textAlign: "center",
    marginTop: 30,
    fontStyle: "italic",
  },
});
