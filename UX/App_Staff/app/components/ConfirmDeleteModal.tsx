import { Modal, View, Text, Pressable, StyleSheet } from "react-native";

type Props = {
  visible: boolean;
  tourLabel?: string;
  isDeleting?: boolean;
  onCancel: () => void;
  onConfirm: () => void;
};

export function ConfirmDeleteModal({ visible, tourLabel, isDeleting = false, onCancel, onConfirm }: Props) {
  return (
    <Modal transparent visible={visible} animationType="fade">
      <View style={styles.overlay}>
        <View style={styles.box}>
          <Text style={styles.title}>Excluir tour</Text>
          <Text style={styles.subtitle}>
            Tem certeza que deseja deletar o tour {tourLabel ? `#${tourLabel}` : ""}? Essa ação não pode ser desfeita.
          </Text>

          <View style={styles.actions}>
            <Pressable style={[styles.button, styles.cancel]} onPress={onCancel} disabled={isDeleting}>
              <Text style={styles.cancelText}>Cancelar</Text>
            </Pressable>
            <Pressable style={[styles.button, styles.delete]} onPress={onConfirm} disabled={isDeleting}>
              <Text style={styles.deleteText}>{isDeleting ? "Deletando..." : "Deletar"}</Text>
            </Pressable>
          </View>
        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: {
    flex: 1,
    backgroundColor: "rgba(0,0,0,0.5)",
    justifyContent: "center",
    alignItems: "center",
    padding: 24,
  },
  box: {
    width: "100%",
    borderRadius: 16,
    backgroundColor: "white",
    padding: 20,
    gap: 12,
  },
  title: {
    fontSize: 18,
    fontWeight: "700",
    color: "#1F1F1F",
  },
  subtitle: {
    fontSize: 14,
    color: "#3C3C3C",
  },
  actions: {
    flexDirection: "row",
    justifyContent: "flex-end",
    gap: 12,
    marginTop: 8,
  },
  button: {
    paddingVertical: 10,
    paddingHorizontal: 16,
    borderRadius: 10,
  },
  cancel: {
    borderWidth: 1,
    borderColor: "#DDD",
  },
  delete: {
    backgroundColor: "#E65050",
  },
  cancelText: {
    color: "#444",
    fontWeight: "600",
  },
  deleteText: {
    color: "#FFF",
    fontWeight: "700",
  },
});
