import { Pressable, View, Text, StyleSheet } from "react-native";
import { Feather, MaterialIcons } from "@expo/vector-icons";

type Props = {
  label: string;
  icon: React.ComponentProps<typeof Feather>["name"];
  onPress?: () => void;
  showChevron?: boolean;
};

export function MenuItem({ label, icon, onPress, showChevron = false }: Props) {
  return (
    <Pressable onPress={onPress} style={({ pressed }) => [styles.container, pressed && styles.pressed]}>
      <View style={styles.iconLabel}>
        <Feather name={icon} size={22} color="#855EDE" />
        <Text style={styles.label}>{label}</Text>
      </View>
      {showChevron && <MaterialIcons name="chevron-right" size={22} color="#9EA0AB" />}
    </Pressable>
  );
}

const styles = StyleSheet.create({
  container: {
    width: "100%",
    paddingVertical: 14,
    paddingHorizontal: 12,
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    borderRadius: 14,
  },
  pressed: {
    backgroundColor: "rgba(133, 94, 222, 0.08)",
  },
  iconLabel: {
    flexDirection: "row",
    alignItems: "center",
    gap: 10,
  },
  label: {
    color: "#F3F4F6",
    fontSize: 16,
    fontWeight: "600",
  },
});
