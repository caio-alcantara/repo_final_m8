import { useState } from 'react';
import {
  Animated,
  View,
  StyleSheet,
  Platform,
  UIManager,
  Text,
  TouchableOpacity,
  Image,
  Pressable,
  ActivityIndicator,
  Alert,
} from "react-native";
import { Header } from "@/components/header";
import { alertasService } from "@/services/api";
import SuccessPopup from "@/components/SuccessPopup";
const AlertButton = require("@/assets/images/alert-button.png");

if (Platform.OS === "android" && UIManager.setLayoutAnimationEnabledExperimental) {
  UIManager.setLayoutAnimationEnabledExperimental(true);
}

type Props = {
  onClose: () => void;
  tourId: number | null;
};

export default function AlertPopup({ onClose, tourId }: Props) {
  const [scaleAnim] = useState(new Animated.Value(1));
  const [sending, setSending] = useState(false);
  const [showSuccess, setShowSuccess] = useState(false);

  const handleAlertPress = async () => {
    Animated.sequence([
      Animated.timing(scaleAnim, {
        toValue: 0.9,
        duration: 100,
        useNativeDriver: true,
      }),
      Animated.timing(scaleAnim, {
        toValue: 1,
        duration: 100,
        useNativeDriver: true,
      }),
    ]).start();

    if (!tourId) {
      Alert.alert("Tour", "Nenhum tour selecionado.");
      return;
    }

    setSending(true);
    try {
      await alertasService.create({
        tour_id: tourId,
        origem: 'manager',
        nivel: 'medio',
        mensagem: 'Alerta enviado pelo app do staff',
        autor_usuario_id: 2,
        resolvido_em: null,
      });
      setShowSuccess(true);
    } catch (error) {
      console.error(error);
      setShowSuccess(false);
      Alert.alert("Erro", "Não foi possível enviar o alerta.");
    } finally {
      setSending(false);
    }
  };

  return (
    <View style={styles.overlay}>
      <View style={styles.container}>
        <Header />
        <View style={styles.content}>
          <TouchableOpacity activeOpacity={0.8} onPress={handleAlertPress} disabled={sending}>
            <Animated.View
              style={[
                styles.buttonCircle,
                { transform: [{ scale: scaleAnim }] }
              ]}
            >
              {sending ? (
                <ActivityIndicator size="large" color="#FFF" />
              ) : (
                <Image source={AlertButton} style={styles.alertButton} resizeMode="contain" />
              )}
            </Animated.View>
          </TouchableOpacity>

          <Text style={styles.title}>DESEJA SOLICITAR A EMERGÊNCIA?</Text>

          <Text style={styles.description}>
            Ao clicar neste ícone, o tour será interrompido e a equipe Inteli será acionada
          </Text>

          <Pressable style={styles.button} onPress={onClose} disabled={sending}>
            <Text style={{ color: "white" }}>Voltar</Text>
          </Pressable>
        </View>
      </View>

      {showSuccess && (
        <SuccessPopup
          onClose={() => {
            setShowSuccess(false);
            onClose();
          }}
        />
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  overlay: {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    height: "100%",
    justifyContent: "flex-start",
    alignItems: "center",
    backgroundColor: "rgba(0, 0, 0, 0.3)",
  },
  container: {
    flex: 1,
    backgroundColor: "#201A2C",
    paddingTop: 64,
    justifyContent: "center",
    alignItems: "center",
  },
  content: {
    flex: 1,
    alignItems: "center",
    justifyContent: "center",
    paddingHorizontal: 40,
    paddingBottom: 60,
  },
  buttonCircle: {
    width: 280,
    height: 280,
    borderRadius: 140,
    backgroundColor: "#6440C4",
    alignItems: "center",
    justifyContent: "center",
    marginBottom: 40,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 8 },
    shadowOpacity: 0.4,
    shadowRadius: 12,
    elevation: 10,
  },
  alertButton: {
    width: 180,
    height: 180,
  },
  title: {
    fontSize: 24,
    fontWeight: "bold",
    color: "#FFFFFF",
    textAlign: "center",
    marginBottom: 20,
    letterSpacing: 1,
  },
  description: {
    fontSize: 24,
    color: "#FFFFFF",
    textAlign: "center",
    lineHeight: 28,
    opacity: 0.95,
    fontWeight: "500",
    paddingHorizontal: 20,
  },
  button: {
    borderRadius: 30,
    backgroundColor: "#6440C4",
    alignItems: "center",
    justifyContent: "center",
    marginTop: 80,
    paddingHorizontal: 30,
    paddingVertical: 10
  }
});
