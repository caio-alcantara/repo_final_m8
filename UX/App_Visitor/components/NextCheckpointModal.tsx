import React, { useEffect, useRef } from "react";
import {
  Modal,
  View,
  Text,
  StyleSheet,
  TouchableOpacity,
  Animated,
  Easing,
  Dimensions,
} from "react-native";
import { MaterialCommunityIcons } from "@expo/vector-icons";
import { BlurView } from "expo-blur"; // Opcional: Se quiser blur no fundo

const { width } = Dimensions.get("window");

interface NextCheckpointModalProps {
  visible: boolean;
  onConfirm: () => void;
  onCancel: () => void;
  nextLocationName?: string; // Opcional: Nome do próximo lugar
}

export default function NextCheckpointModal({
  visible,
  onConfirm,
  onCancel,
  nextLocationName = "Próxima parada",
}: NextCheckpointModalProps) {
  // Animações
  const scaleAnim = useRef(new Animated.Value(0)).current;
  const opacityAnim = useRef(new Animated.Value(0)).current;
  const iconPulse = useRef(new Animated.Value(1)).current;

  // Efeito de Entrada (Pop-up)
  useEffect(() => {
    if (visible) {
      Animated.parallel([
        Animated.spring(scaleAnim, {
          toValue: 1,
          friction: 6,
          tension: 50,
          useNativeDriver: true,
        }),
        Animated.timing(opacityAnim, {
          toValue: 1,
          duration: 200,
          useNativeDriver: true,
        }),
      ]).start();
    } else {
      // Reset para quando abrir de novo
      scaleAnim.setValue(0);
      opacityAnim.setValue(0);
    }
  }, [visible]);

  // Efeito de Pulso no Ícone (Loop Infinito)
  useEffect(() => {
    const pulseLoop = Animated.loop(
      Animated.sequence([
        Animated.timing(iconPulse, {
          toValue: 1.2,
          duration: 1000,
          easing: Easing.inOut(Easing.ease),
          useNativeDriver: true,
        }),
        Animated.timing(iconPulse, {
          toValue: 1,
          duration: 1000,
          easing: Easing.inOut(Easing.ease),
          useNativeDriver: true,
        }),
      ])
    );

    if (visible) {
      pulseLoop.start();
    } else {
      pulseLoop.stop();
      iconPulse.setValue(1);
    }
  }, [visible]);

  if (!visible) return null;

  return (
    <Modal transparent visible={visible} animationType="none" statusBarTranslucent>
      <View style={styles.overlay}>
        {/* Fundo escurecido com Fade */}
        <Animated.View style={[styles.backdrop, { opacity: opacityAnim }]} />

        {/* O Modal em si */}
        <Animated.View
          style={[
            styles.modalContainer,
            { transform: [{ scale: scaleAnim }], opacity: opacityAnim },
          ]}
        >
          {/* Ícone Animado no Topo */}
          <View style={styles.iconContainer}>
            <Animated.View style={{ transform: [{ scale: iconPulse }] }}>
              <MaterialCommunityIcons
                name="map-marker-path"
                size={50}
                color="#8141C2"
              />
            </Animated.View>
          </View>

          <Text style={styles.title}>Vamos prosseguir?</Text>
          
          <Text style={styles.message}>
            O robô está pronto para nos levar até:{"\n"}
            <Text style={styles.highlightText}>{nextLocationName}</Text>
          </Text>

          <View style={styles.buttonRow}>
            {/* Botão Esperar */}
            <TouchableOpacity
              style={styles.cancelButton}
              onPress={onCancel}
              activeOpacity={0.7}
            >
              <Text style={styles.cancelText}>Ainda não</Text>
            </TouchableOpacity>

            {/* Botão Prosseguir */}
            <TouchableOpacity
              style={styles.confirmButton}
              onPress={onConfirm}
              activeOpacity={0.8}
            >
              <Text style={styles.confirmText}>Vamos lá!</Text>
              <MaterialCommunityIcons name="arrow-right" size={20} color="#FFF" />
            </TouchableOpacity>
          </View>
        </Animated.View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: {
    flex: 1,
    justifyContent: "center",
    alignItems: "center",
    zIndex: 1000,
  },
  backdrop: {
    ...StyleSheet.absoluteFillObject,
    backgroundColor: "rgba(0, 0, 0, 0.75)",
  },
  modalContainer: {
    width: width * 0.85,
    backgroundColor: "#1E1730", // Cor de fundo do seu app
    borderRadius: 24,
    padding: 24,
    alignItems: "center",
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.1)", // Borda sutil
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 10 },
    shadowOpacity: 0.5,
    shadowRadius: 20,
    elevation: 20,
  },
  iconContainer: {
    width: 80,
    height: 80,
    borderRadius: 40,
    backgroundColor: "rgba(129, 65, 194, 0.15)", // Roxo translúcido
    justifyContent: "center",
    alignItems: "center",
    marginBottom: 20,
    borderWidth: 1,
    borderColor: "rgba(129, 65, 194, 0.3)",
  },
  title: {
    fontSize: 22,
    fontWeight: "800",
    color: "#FFFFFF",
    marginBottom: 10,
    textAlign: "center",
  },
  message: {
    fontSize: 16,
    color: "#CFCFCF",
    textAlign: "center",
    marginBottom: 30,
    lineHeight: 24,
  },
  highlightText: {
    color: "#8141C2", // Roxo destaque
    fontWeight: "bold",
    fontSize: 18,
  },
  buttonRow: {
    flexDirection: "row",
    justifyContent: "space-between",
    width: "100%",
    gap: 12,
  },
  cancelButton: {
    flex: 1,
    paddingVertical: 14,
    borderRadius: 14,
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.2)",
    alignItems: "center",
    justifyContent: "center",
  },
  cancelText: {
    color: "#CFCFCF",
    fontWeight: "600",
    fontSize: 16,
  },
  confirmButton: {
    flex: 1.5, // Botão de confirmar um pouco maior
    flexDirection: "row",
    paddingVertical: 14,
    borderRadius: 14,
    backgroundColor: "#8141C2",
    alignItems: "center",
    justifyContent: "center",
    gap: 8,
    shadowColor: "#8141C2",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.4,
    shadowRadius: 8,
    elevation: 8,
  },
  confirmText: {
    color: "#FFFFFF",
    fontWeight: "bold",
    fontSize: 16,
  },
});