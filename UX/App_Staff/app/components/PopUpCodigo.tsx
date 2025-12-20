import { View, Text, TouchableOpacity, StyleSheet, Modal, Image } from "react-native";
import { useRouter } from "expo-router";

interface Props {
  visible: boolean;
  onClose: () => void;
  onConfirm: () => void;
  codigo: string;
}

export default function PopUpCodigo({ visible, onClose, onConfirm, codigo }: Props) {
  const router = useRouter();

  return (
    <Modal visible={visible} transparent animationType="fade">
      <View style={styles.overlay}>
        <View style={styles.container}>

          <View style={styles.header}>
            <Text style={styles.title}>Código do tour</Text>

            <TouchableOpacity onPress={onClose}>
              <Image source={require("@/assets/images/icons/X.png")} />
            </TouchableOpacity>
          </View>

          <Text style={styles.text}>
            Coloque o código abaixo no aplicativo do dispositivo móvel dos visitantes
            para que o tour inicie de forma automática.
          </Text>

          <View style={styles.codeBox}>
            <Text style={styles.codeText}>{codigo || "--"}</Text>
          </View>

          <Text style={styles.text}>
            Quando iniciado, você pode ir para a tela do mapa para acompanhar o
            progresso no primeiro andar do robô.
          </Text>

          <View style={styles.footer}>
            <TouchableOpacity onPress={onClose}>
              <Text style={styles.cancelText}>Cancelar</Text>
            </TouchableOpacity>

            <TouchableOpacity
              style={styles.confirmButton}
              onPress={() => {
                onClose();
                router.push("/mapa");
              }}
            >
              <Image
                source={require("@/assets/images/icons/iconMap.png")}
                style={{ marginRight: 6 }}
              />
              <Text style={styles.confirmText}>Ver mapa</Text>
            </TouchableOpacity>
          </View>

        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: {
    flex: 1,
    backgroundColor: "rgba(0,0,0,0.6)",
    justifyContent: "center",
    alignItems: "center",
  },

  container: {
    width: "85%",
    backgroundColor: "#402A78F2",
    padding: 20,
    borderRadius: 18,
    borderColor: "white",
    borderWidth: 1.5,
  },

  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 12,
  },

  title: {
    color: "white",
    fontSize: 20,
    fontWeight: "bold",
  },

  text: {
    color: "white",
    marginBottom: 16,
  },

  codeBox: {
    width: "100%",
    height: 130,
    borderWidth: 2,
    borderColor: "#C7B6FF",
    borderRadius: 14,
    justifyContent: "center",
    alignItems: "center",
    backgroundColor: "rgba(19, 26, 41, 0.5)",
    marginBottom: 10,
  },

  codeText: {
    fontSize: 64,
    fontWeight: "bold",
    color: "#855EDE",
    fontFamily:"Arial",
  },

  footer: {
    flexDirection: "row",
    justifyContent: "space-around",
    alignItems: "center",
    marginTop: 10,
  },

  cancelText: {
    color: "white",
    fontSize: 16,
  },

  confirmButton: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#855EDE",
    paddingHorizontal: 14,
    paddingVertical: 10,
    borderRadius: 20,
  },

  confirmText: {
    color: "white",
    fontWeight: "bold",
    fontSize: 14,
  },
});
