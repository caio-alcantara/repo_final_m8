import { View, Text, Pressable, StyleSheet } from "react-native";

type Props = {
  onClose: () => void;
};

export default function SuccessPopup({ onClose }: Props) {
  return (
    <View style={styles.successOverlay}>
      <View style={styles.successCard}>
        <Text style={styles.successTitle}>Alerta enviado</Text>
        <Text style={styles.successDescription}>
          A equipe foi notificada. Aguarde instruções.
        </Text>
        <Pressable style={styles.successButton} onPress={onClose}>
          <Text style={styles.successButtonText}>Fechar</Text>
        </Pressable>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  successOverlay: {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: "rgba(0,0,0,0.6)",
    justifyContent: "center",
    alignItems: "center",
    padding: 24,
  },
  successCard: {
    backgroundColor: "#2B2340",
    borderRadius: 16,
    padding: 20,
    width: "85%",
    maxWidth: 360,
    borderWidth: 1,
    borderColor: "#5C3DA9",
    gap: 12,
    alignItems: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 8 },
    shadowOpacity: 0.35,
    shadowRadius: 14,
    elevation: 10,
  },
  successTitle: {
    color: "#FFFFFF",
    fontSize: 18,
    fontWeight: "700",
    textAlign: "center",
  },
  successDescription: {
    color: "#E0D9FF",
    fontSize: 14,
    textAlign: "center",
    lineHeight: 20,
  },
  successButton: {
    backgroundColor: "#6440C4",
    borderRadius: 10,
    paddingHorizontal: 20,
    paddingVertical: 10,
    marginTop: 4,
  },
  successButtonText: {
    color: "#FFFFFF",
    fontWeight: "700",
    fontSize: 14,
  },
});
