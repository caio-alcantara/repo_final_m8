import { Entypo, FontAwesome5, Ionicons } from "@expo/vector-icons";
import AntDesign from "@expo/vector-icons/AntDesign";
import { BlurView } from "expo-blur";
import { usePathname, useRouter } from "expo-router";
import React, { useEffect, useRef } from "react";
import { StyleSheet, TouchableOpacity, View, Animated, Dimensions } from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

const { width } = Dimensions.get("window");
const BAR_WIDTH = width * 0.85;
const TAB_WIDTH = BAR_WIDTH / 5;

export default function Navbar() {
  const pathname = usePathname();
  const router = useRouter();
  const insets = useSafeAreaInsets(); // ✅ Hook dentro do componente

  const slideAnim = useRef(new Animated.Value(0)).current;

  const routes = ["mapa", "emergencia", "home", "faq", "menu"];
  useEffect(() => {
    let activeIndex = routes.findIndex((route) => pathname.includes(route));
    if (activeIndex === -1) activeIndex = 2;

    Animated.spring(slideAnim, {
      toValue: activeIndex * TAB_WIDTH,
      useNativeDriver: true,
      tension: 60,
      friction: 8,
    }).start();
  }, [pathname]);

  const isActive = (route: string) => pathname.includes(route);
  const isHome = isActive("home");

  const IconeCentro = isHome ? (
    <Ionicons name="home" size={38} color="#fff" />
  ) : (
    <Ionicons name="mic-outline" size={38} color="#fff" />
  );

  const showNavbarRoutes = ["home", "mapa", "emergencia", "faq", "menu"];
  const shouldShowNavbar = showNavbarRoutes.some((route) => pathname.includes(route));
  if (!shouldShowNavbar) return null;

  return (
    <View style={[styles.container, { bottom: 25 + insets.bottom }]}>
      <BlurView intensity={80} tint="dark" style={[styles.barDimensions, styles.barBackground]} />

      <View style={[styles.barDimensions, styles.barContent]}>
        <Animated.View
          style={[
            styles.liquidGlassCursor,
            { transform: [{ translateX: slideAnim }] },
            { opacity: isHome ? 0 : 1 },
          ]}
        />

        <TouchableOpacity
          style={styles.tabItem}
          onPress={() => router.navigate("/(tabs)/mapa")}
          delayPressIn={0}
          activeOpacity={0.9}
        >
          <Ionicons name="map-outline" size={28} color="#fff" />
        </TouchableOpacity>

        <TouchableOpacity
          style={styles.tabItem}
          onPress={() => router.navigate("/(tabs)/emergencia")}
          delayPressIn={0}
          activeOpacity={0.9}
        >
          <AntDesign name="alert" size={28} color="#fff" />
        </TouchableOpacity>

        <View style={styles.tabItem}>
          <View style={styles.centerButton}>
            <TouchableOpacity
              style={styles.micButton}
              onPress={() => router.navigate("/(tabs)/home")}
              delayPressIn={0}
              activeOpacity={0.9}
            >
              {isHome && <View style={styles.roundGlassOverlay} />}
              {IconeCentro}
            </TouchableOpacity>
          </View>
        </View>

        <TouchableOpacity
          style={styles.tabItem}
          onPress={() => router.navigate("/(tabs)/faq")}
          delayPressIn={0}
          activeOpacity={0.9}
        >
          <FontAwesome5 name="question-circle" size={26} color="#fff" />
        </TouchableOpacity>

        <TouchableOpacity
          style={styles.tabItem}
          onPress={() => router.navigate("/(tabs)/menu")}
          delayPressIn={0}
          activeOpacity={0.9}
        >
          <Entypo name="menu" size={30} color="#fff" />
        </TouchableOpacity>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    position: "absolute",
    width: "100%",
    alignItems: "center",
    zIndex: 999,
    justifyContent: "center",
  },
  barDimensions: {
    width: "85%",
    height: 70,
    borderRadius: 50,
  },
  barBackground: {
    position: "absolute",
    backgroundColor: "rgba(81, 61, 160, 0.25)",
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.15)",
    overflow: "hidden",
  },
  barContent: {
    flexDirection: "row",
    alignItems: "center",
    overflow: "visible",
    backgroundColor: "transparent",
  },
  tabItem: {
    flex: 1,
    height: "100%",
    alignItems: "center",
    justifyContent: "center",
    zIndex: 2,
  },
  liquidGlassCursor: {
    position: "absolute",
    left: 0,
    top: 0,
    bottom: 0,
    width: TAB_WIDTH,
    backgroundColor: "rgba(255, 255, 255, 0.15)",
    borderRadius: 50,
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.3)",
    zIndex: 1,
  },
  centerButton: {
    top: -15,
    alignSelf: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.3,
    shadowRadius: 6,
    elevation: 8,
    zIndex: 1,
  },
  micButton: {
    width: 150,
    height: 150,
    borderRadius: 100,
    backgroundColor: "#6A40C4",
    alignItems: "center",
    justifyContent: "center",
    borderWidth: 2,
    borderColor: "rgba(255,255,255,0.2)",
  },
  roundGlassOverlay: {
    position: "absolute",
    width: "100%",
    height: "100%",
    borderRadius: 100,
    backgroundColor: "rgba(255, 255, 255, 0.15)",
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.3)",
  },
});
