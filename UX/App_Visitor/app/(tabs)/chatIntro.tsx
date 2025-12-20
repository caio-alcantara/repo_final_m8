import React, { useEffect, useRef } from "react";
import {
  Image,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
  Animated,
  Dimensions,
  StatusBar,
  SafeAreaView,
  Platform,
  Easing
} from "react-native";
import { useRouter } from "expo-router";
import { Ionicons } from "@expo/vector-icons";
import { useTour } from "@/context/TourContext";
import Logo from "../../assets/images/logo-branca.png";

const { width } = Dimensions.get("window");

export default function ChatIntro() {
  const router = useRouter();
  const { tour } = useTour();
  const visitorName = tour?.visitorName || "Visitante";

  // --- ANIMAÇÕES ---
  const fadeAnim = useRef(new Animated.Value(0)).current;
  const slideAnim = useRef(new Animated.Value(50)).current;
  const buttonScale = useRef(new Animated.Value(1)).current;
  
  // Animação da mãozinha (Rotação)
  const waveAnim = useRef(new Animated.Value(0)).current;

  useEffect(() => {
    // 1. Entrada dos elementos (Fade + Slide)
    Animated.parallel([
      Animated.timing(fadeAnim, {
        toValue: 1,
        duration: 800,
        useNativeDriver: true,
      }),
      Animated.timing(slideAnim, {
        toValue: 0,
        duration: 800,
        useNativeDriver: true,
      }),
    ]).start();

    // 2. Loop da mãozinha acenando
    Animated.loop(
      Animated.sequence([
        // Balança para a esquerda
        Animated.timing(waveAnim, {
          toValue: -1, 
          duration: 150,
          easing: Easing.linear,
          useNativeDriver: true,
        }),
        // Balança para a direita
        Animated.timing(waveAnim, {
          toValue: 1, 
          duration: 150,
          easing: Easing.linear,
          useNativeDriver: true,
        }),
        // Esquerda
        Animated.timing(waveAnim, {
          toValue: -1, 
          duration: 150,
          easing: Easing.linear,
          useNativeDriver: true,
        }),
        // Centro (pausa)
        Animated.timing(waveAnim, {
          toValue: 0, 
          duration: 150,
          easing: Easing.linear,
          useNativeDriver: true,
        }),
        // Pausa de 2 segundos antes de acenar de novo
        Animated.delay(2000) 
      ])
    ).start();
  }, []);

  // Interpolação da rotação da mão (-20 graus a +20 graus)
  const waveRotation = waveAnim.interpolate({
    inputRange: [-1, 1],
    outputRange: ['-20deg', '20deg']
  });

  const handleStartChat = () => {
    Animated.sequence([
      Animated.timing(buttonScale, {
        toValue: 0.95,
        duration: 100,
        useNativeDriver: true,
      }),
      Animated.timing(buttonScale, {
        toValue: 1,
        duration: 100,
        useNativeDriver: true,
      }),
    ]).start(() => {
      router.push("/(tabs)/home");
    });
  };

  return (
    <View style={styles.container}>
      <StatusBar barStyle="light-content" backgroundColor="#1E1730" />
      
      <SafeAreaView style={styles.safeArea}>
        
        {/* HEADER */}
        <Animated.View 
          style={[
            styles.header, 
            { opacity: fadeAnim, transform: [{ translateY: slideAnim }] }
          ]}
        >
          <Image source={Logo} style={styles.logo} resizeMode="contain" />
        </Animated.View>

        {/* CONTEÚDO */}
        <View style={styles.contentContainer}>
          
          <Animated.View 
            style={[
              styles.textWrapper,
              { opacity: fadeAnim, transform: [{ translateY: slideAnim }] }
            ]}
          >
            <Text style={styles.eyebrow}>TUDO PRONTO</Text>
            
            {/* TÍTULO COM A MÃOZINHA ANIMADA AO LADO */}
            <View style={styles.titleRow}>
              <Text style={styles.greetingTitle}>Olá, {visitorName}!</Text>
              <Animated.View style={{ transform: [{ rotate: waveRotation }] }}>
                <Text style={styles.greetingTitle}> 👋</Text>
              </Animated.View>
            </View>

            <View style={styles.divider} />

            <Text style={styles.bodyText}>
              Aqui você poderá conversar com a <Text style={styles.highlight}>LIA</Text>, 
              a inteligência artificial do nosso robô.
            </Text>

            <Text style={styles.bodyText}>
              Envie perguntas por <Text style={styles.highlight}>voz</Text> ou <Text style={styles.highlight}>texto</Text>. 
              As respostas serão dadas ao longo do seu tour pelo Inteli.
            </Text>

          </Animated.View>

          {/* BOTÃO */}
          <Animated.View 
            style={[
              styles.footer, 
              { opacity: fadeAnim, transform: [{ scale: buttonScale }] }
            ]}
          >
            <TouchableOpacity 
              style={styles.primaryButton} 
              onPress={handleStartChat}
              activeOpacity={0.8}
            >
              <Text style={styles.buttonText}>Começar Conversa</Text>
              <Ionicons name="chatbubble-ellipses-outline" size={24} color="#FFF" style={{ marginLeft: 10 }} />
            </TouchableOpacity>

            <Text style={styles.disclaimer}>
              Toque para iniciar a interação com a LIA
            </Text>
          </Animated.View>

        </View>
      </SafeAreaView>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    // Cor sólida mais clara (Roxo Inteli Dark) conforme solicitado
    backgroundColor: "#1E1730", 
  },
  safeArea: {
    flex: 1,
    paddingTop: Platform.OS === "android" ? 40 : 0,
    justifyContent: "space-between",
  },
  header: {
    alignItems: "center",
    marginTop: 20,
  },
  logo: {
    width: 100,
    height: 40,
  },
  contentContainer: {
    flex: 1,
    justifyContent: "center",
    paddingHorizontal: 32,
    paddingBottom: 40,
  },
  textWrapper: {
    alignItems: "flex-start",
    marginBottom: 40,
  },
  eyebrow: {
    color: "#8B2CF5",
    fontSize: 14,
    fontWeight: "700",
    letterSpacing: 2,
    marginBottom: 10,
    textTransform: "uppercase",
  },
  // Container para alinhar Texto + Emoji na mesma linha
  titleRow: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 20,
    flexWrap: 'wrap', // Garante que não quebre feio em telas pequenas
  },
  greetingTitle: {
    color: "#FFFFFF",
    fontSize: 36,
    fontWeight: "800",
    lineHeight: 42,
  },
  divider: {
    width: 40,
    height: 4,
    backgroundColor: "#8B2CF5",
    borderRadius: 2,
    marginBottom: 24,
  },
  bodyText: {
    color: "#E0E0E0",
    fontSize: 18,
    lineHeight: 28,
    fontWeight: "400",
    marginBottom: 20,
  },
  highlight: {
    color: "#B794FF",
    fontWeight: "700",
  },
  footer: {
    alignItems: "center",
    marginTop: 20,
  },
  primaryButton: {
    width: "100%",
    height: 64,
    backgroundColor: "#8B2CF5",
    borderRadius: 32,
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    shadowColor: "#8B2CF5",
    shadowOffset: { width: 0, height: 8 },
    shadowOpacity: 0.4,
    shadowRadius: 16,
    elevation: 10,
  },
  buttonText: {
    color: "#FFFFFF",
    fontSize: 18,
    fontWeight: "bold",
    textTransform: "uppercase",
    letterSpacing: 1,
  },
  disclaimer: {
    marginTop: 20,
    color: "rgba(255,255,255,0.4)",
    fontSize: 12,
    textAlign: "center",
  },
});