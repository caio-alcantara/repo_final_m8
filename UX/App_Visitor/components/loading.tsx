import React from "react";
import { View, StyleSheet, Platform, ActivityIndicator } from "react-native";

export default function Loading() {
  // ✅ Web: fallback simples (evita o bundler tentar resolver lottie-react-native web)
  if (Platform.OS === "web") {
    return (
      <View style={styles.container}>
        <ActivityIndicator size="large" color="#FFFFFF" />
      </View>
    );
  }

  // ✅ Mobile: carrega lottie apenas aqui (import dinâmico)
  const LottieView = require("lottie-react-native").default;

  return (
    <View style={styles.container}>
      <LottieView
        source={require("../assets/animations/loading.json")}
        autoPlay
        loop
        style={styles.animation}
      />
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    ...StyleSheet.absoluteFillObject,
    backgroundColor: "rgba(30, 23, 48, 0.9)",
    justifyContent: "center",
    alignItems: "center",
    zIndex: 999,
  },
  animation: {
    width: 200,
    height: 200,
  },
});
