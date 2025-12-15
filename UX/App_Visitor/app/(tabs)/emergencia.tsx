import Navbar from "@/components/navbar";
import { StatusBar } from "expo-status-bar";
import React, { useState } from "react";
import { Animated, Image, StyleSheet, Text, TouchableOpacity, View } from "react-native";

const Logo = require("../../assets/images/logo-branca.png");
const AlertButton = require("../../assets/images/alert-button.png");

export default function Emergencia() {
  const [scaleAnim] = useState(new Animated.Value(1));

  const handleAlertPress = () => {
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

    console.log("🚨 EMERGÊNCIA ACIONADA - Tour interrompido, equipe Inteli notificada");
  };

  return (
    <>
      <StatusBar hidden />
      <View style={styles.container}>
        <View style={styles.header}>
          <Image source={Logo} style={styles.logo} resizeMode="contain" />
        </View>

        <View style={styles.content}>
          <TouchableOpacity 
            activeOpacity={0.8}
            onPress={handleAlertPress}
          >
            <Animated.View 
              style={[
                styles.buttonCircle,
                { transform: [{ scale: scaleAnim }] }
              ]}
            >
              <Image source={AlertButton} style={styles.alertButton} resizeMode="contain" />
            </Animated.View>
          </TouchableOpacity>

          <Text style={styles.title}>DESEJA SOLICITAR A EMERGÊNCIA?</Text>

          <Text style={styles.description}>
            Ao clicar neste ícone, o tour será interrompido e a equipe Inteli será acionada
          </Text>
        </View>

        <Navbar />
      </View>
    </>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#1E1730",
  },
  header: {
    alignItems: "center",
    marginTop: 40,
    marginBottom: 20,
  },
  logo: {
    width: 120,
    height: 100,
  },
  content: {
    flex: 1,
    alignItems: "center",
    justifyContent: "center",
    paddingHorizontal: 40,
    paddingBottom: 60, 
    marginTop: -40, 
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
});
