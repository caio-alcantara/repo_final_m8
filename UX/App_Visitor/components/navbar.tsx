import { Entypo, FontAwesome5, Ionicons } from "@expo/vector-icons";
import AntDesign from '@expo/vector-icons/AntDesign';
import { BlurView } from "expo-blur";
import { usePathname, useRouter } from "expo-router";
import React from "react";
import { StyleSheet, TouchableOpacity, View } from "react-native";

export default function Navbar() {
  const pathname = usePathname();
  const router = useRouter();
  
  const isHome = pathname === "/home";

  // define o ícone central uma vez, fora do JSX
  const IconeCentro = isHome ? (
    <Ionicons name="home" size={38} color="#fff" />
  ) : (
    <Ionicons name="mic-outline" size={38} color="#fff" />
  );

  return (
    <View style={styles.container}>
      <BlurView intensity={80} tint="dark" style={styles.bar}>
        <TouchableOpacity 
          style={styles.iconButton}
          onPress={() => router.push("/(tabs)/mapa")}
        >
          <Ionicons name="map-outline" size={28} color="#fff" />
        </TouchableOpacity>

        <TouchableOpacity 
          style={styles.iconButton}
          onPress={() => router.push("/(tabs)/emergencia")}
        >
          <AntDesign name="alert" size={28} color="#fff" />
        </TouchableOpacity>
        
        <View style={styles.centerButton}>
          <TouchableOpacity style={styles.micButton}
          onPress={() => router.push("/(tabs)/home")}
          >
            {IconeCentro} {/* aqui vai o componente, não a string */}
          </TouchableOpacity>
        </View>

        <TouchableOpacity 
          style={styles.iconButton}
          onPress={() => router.push("/(tabs)/faq")}
        >
          <FontAwesome5 name="question-circle" size={26} color="#fff" />
        </TouchableOpacity>

        <TouchableOpacity 
          style={styles.iconButton}
          onPress={() => router.push("/(tabs)/menu")}
        >
          <Entypo name="menu" size={30} color="#fff" />
        </TouchableOpacity>
      </BlurView>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    position: "absolute",
    bottom: 25,
    width: "100%",
    alignItems: "center",
  },
  bar: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    width: "85%",
    height: 70,
    borderRadius: 50,
    backgroundColor: "rgba(81, 61, 160, 0.25)",
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.15)",
    paddingHorizontal: 25,
  },
  iconButton: {
    padding: 8,
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
});
