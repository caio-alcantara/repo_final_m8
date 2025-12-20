import { View, Image, StyleSheet } from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

export function Header() {
  const insets = useSafeAreaInsets();

  return (
    <View style={[styles.header, { top: insets.top + 8 }]}>
      <Image source={require("../assets/images/logo_inteli.png")} />
    </View>
  );
}

const styles = StyleSheet.create({
  header: {
    position: "absolute",
    left: 0,
    right: 0,
    alignItems: "center",
    zIndex: 10,
    pointerEvents: "none",
  },
});
