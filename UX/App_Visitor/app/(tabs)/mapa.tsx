import Navbar from "@/components/navbar";
import { StatusBar } from "expo-status-bar";
import React, { useEffect, useState } from "react";
import { Animated, Image, StyleSheet, Text, View } from "react-native";

const Logo = require("../../assets/images/logo-branca.png");
const MapaTerreo = require("../../assets/images/mapa_terreo.png");

// Estado do checkpoint
type CheckpointState = "unvisited" | "visited" | "visiting";

// Tipo do checkpoint
interface Checkpoint {
  id: number;
  x: number; // Coordenada X em % da largura da imagem
  y: number; // Coordenada Y em % da altura da imagem
  state: CheckpointState;
}

// Mock de checkpoints - ajuste as coordenadas x e y conforme necessário
const MOCK_CHECKPOINTS: Checkpoint[] = [
  { id: 1, x: 77, y: 50, state: "visited" },
  { id: 2, x: 50, y: 47, state: "visited" },
  { id: 3, x: 40, y: 38, state: "visiting" },
  { id: 4, x: 14.5, y: 58, state: "unvisited" },
  { id: 5, x: 5, y: 28, state: "unvisited" },
];

export default function Mapa() {
  const [checkpoints, setCheckpoints] = useState<Checkpoint[]>(MOCK_CHECKPOINTS);

  return (
    <>
      <StatusBar hidden />
      <View style={styles.container}>
        {/* Logo do Inteli no topo */}
        <View style={styles.header}>
          <Image source={Logo} style={styles.logo} resizeMode="contain" />
        </View>

        {/* Título */}
        <Text style={styles.title}>Acompanhamento em tempo real do seu tour</Text>

        {/* Container do mapa */}
        <View style={styles.mapContainer}>
          <View style={styles.mapWrapper}>
            {/* Imagem do mapa */}
            <Image source={MapaTerreo} style={styles.mapImage} resizeMode="contain" />

            {/* Checkpoints sobrepostos */}
            {checkpoints.map((checkpoint) => (
              <CheckpointMarker
                key={checkpoint.id}
                x={checkpoint.x}
                y={checkpoint.y}
                state={checkpoint.state}
              />
            ))}
          </View>
        </View>

        {/* Navbar */}
        <Navbar />
      </View>
    </>
  );
}

// Componente de marcador de checkpoint
function CheckpointMarker({ x, y, state }: { x: number; y: number; state: CheckpointState }) {
  const [pulseAnim] = useState(new Animated.Value(1));
  const [glowAnim] = useState(new Animated.Value(1));

  // Animação de pulso apenas para o estado "visiting"
  useEffect(() => {
    if (state === "visiting") {
      const pulse = Animated.loop(
        Animated.sequence([
          Animated.timing(pulseAnim, {
            toValue: 1.15,
            duration: 800,
            useNativeDriver: true,
          }),
          Animated.timing(pulseAnim, {
            toValue: 1,
            duration: 800,
            useNativeDriver: true,
          }),
        ])
      );
      pulse.start();
      return () => pulse.stop();
    } else {
      pulseAnim.setValue(1);
    }
  }, [state]);

  // Animação do glow (halo piscante) apenas para o estado "visiting"
  useEffect(() => {
    if (state === "visiting") {
      const glow = Animated.loop(
        Animated.sequence([
          Animated.timing(glowAnim, {
            toValue: 1.3,
            duration: 800,
            useNativeDriver: true,
          }),
          Animated.timing(glowAnim, {
            toValue: 1,
            duration: 800,
            useNativeDriver: true,
          }),
        ])
      );
      glow.start();
      return () => glow.stop();
    } else {
      glowAnim.setValue(1);
    }
  }, [state]);

  // Define a cor baseada no estado
  const getColor = () => {
    switch (state) {
      case "visited":
        return "#00E676"; // Verde neon
      case "visiting":
        return "#00E676"; // Verde neon
      case "unvisited":
        return "#9E9E9E"; // Cinza mais claro
      default:
        return "#9E9E9E";
    }
  };

  const getBorderColor = () => {
    switch (state) {
      case "visited":
        return "#00E676";
      case "visiting":
        return "#00E676"; // Verde neon
      case "unvisited":
        return "#FFFFFF";
      default:
        return "#FFFFFF";
    }
  };

  return (
    <>
      {/* Halo/Glow effect */}
      {state !== "unvisited" && (
        <Animated.View
          style={[
            styles.checkpointGlow,
            {
              left: `${x}%`,
              top: `${y}%`,
              opacity: state === "visiting" ? 0.5 : 0.6,
              transform: [{ scale: state === "visiting" ? glowAnim : 1 }],
              backgroundColor: "#00E676",
            },
          ]}
        />
      )}
      
      {/* Checkpoint principal */}
      <Animated.View
        style={[
          styles.checkpoint,
          {
            left: `${x}%`,
            top: `${y}%`,
            backgroundColor: getColor(),
            borderColor: getBorderColor(),
            transform: [{ scale: state === "visiting" ? pulseAnim : 1 }],
          },
        ]}
      >
        {/* Ponto interno para mais contraste */}
        <View style={styles.checkpointInner} />
      </Animated.View>

      {/* Label "Você está aqui" apenas para visiting */}
      {state === "visiting" && (
        <View
          style={[
            styles.labelContainer,
            {
              left: `${x}%`,
              top: `${y}%`,
            },
          ]}
        >
          {/* Texto */}
          <View style={styles.labelBubble}>
            <Text style={styles.labelText}>Você está aqui</Text>
          </View>
        </View>
      )}
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
  title: {
    fontSize: 24,
    fontWeight: "bold",
    color: "#FFFFFF",
    textAlign: "center",
    marginBottom: 10,
    marginTop: -20,
    paddingHorizontal: 20,
  },
  mapContainer: {
    flex: 1,
    alignItems: "center",
    justifyContent: "center",
    paddingHorizontal: 20,
    paddingBottom: 200, // Espaço para a navbar
    paddingTop: 10,
  },
  mapWrapper: {
    position: "relative",
    width: "100%",
    aspectRatio: 1, 
    maxWidth: 1000,
    maxHeight: 700,
  },
  mapImage: {
    width: "100%",
    height: "100%",
  },
  checkpoint: {
    position: "absolute",
    width: 40,
    height: 40,
    borderRadius: 20,
    borderWidth: 5,
    borderColor: "#FFFFFF",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 6 },
    shadowOpacity: 0.8,
    shadowRadius: 12,
    elevation: 15,
    marginLeft: -20,
    marginTop: -20,
    justifyContent: "center",
    alignItems: "center",
  },
  checkpointInner: {
    width: 12,
    height: 12,
    borderRadius: 6,
    backgroundColor: "#FFFFFF",
  },
  checkpointGlow: {
    position: "absolute",
    width: 60,
    height: 60,
    borderRadius: 30,
    marginLeft: -30,
    marginTop: -30,
  },
  labelContainer: {
    position: "absolute",
    flexDirection: "column",
    alignItems: "center",
    marginTop: -65,
    marginLeft: -60,
  },
  labelBubble: {
    backgroundColor: "#6A40C4",
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 20,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.3,
    shadowRadius: 4,
    elevation: 5,
  },
  labelText: {
    color: "#FFFFFF",
    fontSize: 14,
    fontWeight: "bold",
    textAlign: "center",
  },
});