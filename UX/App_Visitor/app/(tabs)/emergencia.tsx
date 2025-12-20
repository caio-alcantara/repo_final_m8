import Navbar from "@/components/navbar";
import { StatusBar } from "expo-status-bar";
import React, { useRef, useState } from "react";
import { Animated, Easing, Image, StyleSheet, Text, TouchableWithoutFeedback, View } from "react-native";

const Logo = require("../../assets/images/logo-branca.png");

export default function Emergencia() {
  const glassOpenAnim = useRef(new Animated.Value(0)).current;
  const buttonPressAnim = useRef(new Animated.Value(1)).current;
  const [isActivating, setIsActivating] = useState(false);

  // Altura da caixa para cálculo do pivô
  const BOX_HEIGHT = 260; 
  const HALF_HEIGHT = BOX_HEIGHT / 2;

  const handleEmergencyPress = () => {
    if (isActivating) return; 
    setIsActivating(true);

    Animated.sequence([
      // 1. Abre o vidro (vai até 1)
      Animated.timing(glassOpenAnim, {
        toValue: 1,
        duration: 300,
        easing: Easing.out(Easing.back(1.5)), // Um leve balanço ao abrir
        useNativeDriver: true,
      }),
      // 2. Pressiona botão
      Animated.timing(buttonPressAnim, {
        toValue: 0.85,
        duration: 100,
        easing: Easing.inOut(Easing.ease),
        useNativeDriver: true,
      }),
      // 3. Solta botão
      Animated.timing(buttonPressAnim, {
        toValue: 1,
        duration: 100,
        easing: Easing.inOut(Easing.ease),
        useNativeDriver: true,
      }),
      Animated.delay(100).start(() => {
         console.log("🚨 EMERGÊNCIA ACIONADA");
         // Lógica de navegação aqui
      }),
      // 4. Fecha o vidro
      Animated.delay(1500),
      Animated.timing(glassOpenAnim, {
        toValue: 0,
        duration: 600,
        easing: Easing.bounce, 
        useNativeDriver: true,
      }),
    ]).start(() => {
      setIsActivating(false);
    });
  };

  // Rotação: De 0 a -100 graus (abre bem para cima)
  const glassRotation = glassOpenAnim.interpolate({
    inputRange: [0, 1],
    outputRange: ["0deg", "-110deg"], 
  });

  const textOpacity = glassOpenAnim.interpolate({
    inputRange: [0, 0.3],
    outputRange: [1, 0],
  });

  return (
    <>
      <StatusBar hidden />
      <View style={styles.container}>
        <View style={styles.header}>
          <Image source={Logo} style={styles.logo} resizeMode="contain" />
        </View>

        <View style={styles.content}>
          
          <TouchableWithoutFeedback onPress={handleEmergencyPress} disabled={isActivating}>
            {/* CONTAINER GERAL */}
            <View style={styles.emergencyBoxContainer}>
              
              {/* 1. A CAIXA (Fica por baixo) */}
              <View style={styles.boxBase}>
                <Animated.View 
                  style={[
                    styles.innerButton,
                    { transform: [{ scale: buttonPressAnim }] }
                  ]}
                >
                  <View style={styles.innerButtonHighlight} />
                </Animated.View>
              </View>

              {/* 2. O VIDRO (Fica por cima) */}
              {/* Container absoluto para garantir alinhamento */}
              <View style={styles.glassPositioner}>
                <Animated.View 
                  style={[
                    styles.glassCover,
                    { 
                      transform: [
                        { perspective: 1000 }, 
                        // --- TRUQUE DO PIVÔ NO TOPO ---
                        // 1. Move o centro para a borda superior
                        { translateY: -HALF_HEIGHT }, 
                        // 2. Rotaciona no eixo X
                        { rotateX: glassRotation },
                        // 3. Traz de volta para a posição original
                        { translateY: HALF_HEIGHT }
                      ]
                    }
                  ]}
                >
                  <View style={styles.glassReflection} />
                  <Animated.Text style={[styles.glassText, { opacity: textOpacity }]}>
                    EMERGÊNCIA
                  </Animated.Text>
                  
                  {/* Detalhe da dobradiça visual */}
                  <View style={styles.hingeDetailLeft} />
                  <View style={styles.hingeDetailRight} />
                  
                  <View style={styles.glassHandle} />
                </Animated.View>
              </View>

            </View>
          </TouchableWithoutFeedback>

          <Text style={styles.title}>DESEJA SOLICITAR A EMERGÊNCIA?</Text>
          <Text style={styles.description}>
            Ao acionar o dispositivo acima, o tour será interrompido e a equipe Inteli será notificada imediatamente.
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
    paddingHorizontal: 30,
    paddingBottom: 80,
    marginTop: -20,
  },
  
  // --- CAIXA ---
  emergencyBoxContainer: {
    width: 260,
    height: 260,
    marginBottom: 50,
    alignItems: 'center',
    justifyContent: 'center',
  },
  boxBase: {
    width: '100%',
    height: '100%',
    backgroundColor: '#750000', // Vermelho um pouco mais escuro para profundidade
    borderRadius: 20,
    borderWidth: 0, // Removi a borda aqui para o vidro cobrir tudo
    alignItems: 'center',
    justifyContent: 'center',
    // Sombra da caixa no chão
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 15 },
    shadowOpacity: 0.6,
    shadowRadius: 20,
    elevation: 20,
    // Borda interna falsa para profundidade
    borderBottomWidth: 10,
    borderBottomColor: '#4a0000',
  },
  innerButton: {
    width: 150,
    height: 150,
    borderRadius: 75,
    backgroundColor: '#D92424', 
    alignItems: 'center',
    justifyContent: 'center',
    borderWidth: 4,
    borderColor: '#b30000', // Borda escura no botão
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 5 },
    shadowOpacity: 0.4,
    shadowRadius: 6,
    elevation: 8,
  },
  innerButtonHighlight: {
    position: 'absolute',
    top: 25,
    width: 90,
    height: 40,
    backgroundColor: 'rgba(255,255,255,0.25)',
    borderRadius: 20,
  },

  // --- VIDRO ---
  glassPositioner: {
    position: 'absolute',
    top: 0,
    left: 0,
    width: 260,
    height: 260,
    zIndex: 10,
  },
  glassCover: {
    width: '100%',
    height: '100%',
    backgroundColor: 'rgba(200, 230, 255, 0.25)', // Vidro azulado leve
    borderRadius: 20,
    borderWidth: 2,
    borderColor: 'rgba(255,255,255,0.5)',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingVertical: 20,
    overflow: 'hidden',
    // Backface visibility garante que ele pareça sólido ao girar
    backfaceVisibility: 'visible',
  },
  glassReflection: {
    position: 'absolute',
    top: -60,
    left: -60,
    width: 350,
    height: 60,
    backgroundColor: 'rgba(255,255,255,0.15)',
    transform: [{ rotate: '35deg' }],
  },
  glassText: {
    color: 'rgba(255,255,255,0.9)',
    fontSize: 22,
    fontWeight: '800',
    letterSpacing: 3,
    marginTop: 85,
    textShadowColor: 'rgba(0,0,0,0.5)',
    textShadowOffset: {width: 0, height: 1},
    textShadowRadius: 4,
  },
  glassHandle: {
    width: 80,
    height: 6,
    backgroundColor: 'rgba(255,255,255,0.8)',
    borderRadius: 3,
    marginBottom: 5,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.3,
    shadowRadius: 2,
  },

  // Detalhes da dobradiça no topo (visual apenas)
  hingeDetailLeft: {
    position: 'absolute',
    top: 8,
    left: 20,
    width: 12,
    height: 12,
    borderRadius: 6,
    backgroundColor: 'rgba(255,255,255,0.4)',
  },
  hingeDetailRight: {
    position: 'absolute',
    top: 8,
    right: 20,
    width: 12,
    height: 12,
    borderRadius: 6,
    backgroundColor: 'rgba(255,255,255,0.4)',
  },
  
  // --- RODAPÉ ---
  title: {
    fontSize: 24,
    fontWeight: "bold",
    color: "#FFFFFF",
    textAlign: "center",
    marginBottom: 15,
    letterSpacing: 1,
  },
  description: {
    fontSize: 20,
    color: "#E0E0E0",
    textAlign: "center",
    lineHeight: 28,
    fontWeight: "400",
    paddingHorizontal: 20,
  },
});