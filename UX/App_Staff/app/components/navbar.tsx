import {
  View,
  Pressable,
  Text,
  StyleSheet,
  LayoutAnimation,
} from "react-native";
import { Feather, Octicons, AntDesign } from "@expo/vector-icons";
import { useRouter, usePathname } from "expo-router";
import { useSafeAreaInsets } from "react-native-safe-area-context";

export function Navbar() {
  const pathname = usePathname();
  const router = useRouter();
  const insets = useSafeAreaInsets();

  const isActive = (route: string) => pathname === route;

  const handlePress = (route: string) => {
    LayoutAnimation.configureNext(LayoutAnimation.Presets.easeInEaseOut);
    router.push(route as any);
  };

  return (
    <View
      style={[
        styles.navbar,
        {
          bottom: Math.max(insets.bottom + 10, 16),
        },
      ]}
    >
      <Pressable onPress={() => handlePress("/")}>
        <View style={[styles.item, isActive("/") && styles.itemActive]}>
          <Octicons name="workflow" size={22} color="#FFF" />
          {isActive("/") && <Text style={styles.text}>Tours</Text>}
        </View>
      </Pressable>

      <Pressable onPress={() => handlePress("/mapa")}>
        <View style={[styles.item, isActive("/mapa") && styles.itemActive]}>
          <Feather name="map" size={20} color="#FFF" />
          {isActive("/mapa") && <Text style={styles.text}>Mapa</Text>}
        </View>
      </Pressable>

      <Pressable onPress={() => handlePress("/menu")}>
        <View style={[styles.item, isActive("/menu") && styles.itemActive]}>
          <Octicons name="three-bars" size={22} color="#FFF" />
          {isActive("/menu") && <Text style={styles.text}>Menu</Text>}
        </View>
      </Pressable>

      {/* Rota WS Test removida */}
    </View>
  );
}

const styles = StyleSheet.create({
  navbar: {
    flexDirection: "row",
    justifyContent: "space-between",
    backgroundColor: "rgba(92, 61, 169, 0.9)",
    borderRadius: 50,
    width: "95%",
    position: "absolute",
    alignSelf: "center",
    padding: 12,
  },
  item: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
    paddingVertical: 10,
    paddingHorizontal: 16,
    borderRadius: 24,
  },
  itemActive: {
    backgroundColor: "#5C3DA9",
  },
  text: {
    color: "#FFF",
    fontWeight: "600",
    fontSize: 14,
  },
});
