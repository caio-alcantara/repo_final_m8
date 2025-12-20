import { Ionicons } from "@expo/vector-icons";
import { useRouter, useFocusEffect } from "expo-router"; // <--- 1. Importe useFocusEffect
import React, { useState, useRef, useEffect, useCallback } from "react"; // <--- 2. Importe useCallback
import {
  Image,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
  Animated,
  StatusBar,
  SafeAreaView,
  Platform,
  Dimensions
} from "react-native";
import Logo from "../../assets/images/logo-branca.png";

const imgEmergencia = require("../../assets/images/navbar.png");
const imgDistancia = require("../../assets/images/distancia.png");
const imgDuvidas = require("../../assets/images/ondas.png");
const imgConfirmacao = require("../../assets/images/ondas.png");

const { height } = Dimensions.get('window');

export default function Onboarding() {
  const router = useRouter();
  
  // Animações
  const fadeAnim = useRef(new Animated.Value(1)).current;
  const pulseAnim = useRef(new Animated.Value(1)).current;

  const [currentStep, setCurrentStep] = useState(0);

  // --- SOLUÇÃO DO BUG ---
  // Toda vez que essa tela ganha foco (aparece para o usuário), forçamos o reset.
  useFocusEffect(
    useCallback(() => {
      // 1. Garante que a tela esteja visível (opacidade 1)
      fadeAnim.setValue(1);
      
      // 2. Garante que o pulso esteja no tamanho original
      pulseAnim.setValue(1);

      // 3. (Opcional) Se quiser sempre voltar para o passo 0 ao entrar na tela:
      setCurrentStep(0);
    }, [])
  );
  // ----------------------

  const steps = [
    {
      title: "EM CASOS DE EMERGÊNCIA",
      subtitle: "A AJUDA PODERÁ SER SOLICITADA PELO APP",
      image: imgEmergencia,
      hasPulse: true,
      customStyles: {
        image: { maxHeight: 200, marginBottom: 20 },
        pulse: { top: "32%", left: "21%" },
        textBlock: { marginBottom: 120 },
        title: { fontSize: 48, lineHeight: 52 },
        subtitle: { fontSize: 24 },
      },
    },
    {
      title: "MANTENHA UMA DISTÂNCIA SEGURA",
      subtitle: "DE NO MÍNIMO 2 METROS DA LIA",
      image: imgDistancia,
      customStyles: {
        image: { maxHeight: 200 },
        textBlock: { marginBottom: 100 },
        title: { fontSize: 48, lineHeight: 52 },
        subtitle: { fontSize: 24 },
      },
    },
    {
      title: "TIRE SUAS DÚVIDAS POR ÁUDIO OU TEXTO",
      subtitle: "ELAS SERÃO RESPONDIDAS PELA LIA AO FINAL DE CADA ETAPA",
      image: imgDuvidas,
      customStyles: {
        image: { maxHeight: 150, width: "90%" },
        textBlock: { marginBottom: 110 },
        title: { fontSize: 42, lineHeight: 46 },
        subtitle: { fontSize: 22 },
      },
    },
    {
      title: "AO FINAL DE CADA ETAPAS CONFIRMAREMOS",
      subtitle: "SE VOCÊ DESEJA PROSSEGUIR",
      image: imgConfirmacao,
      customStyles: {
        image: { maxHeight: 150 },
        textBlock: { marginBottom: 130 },
        title: { fontSize: 45, lineHeight: 50 },
        subtitle: { fontSize: 26 },
      },
    },
  ];

  // Efeito de Pulso
  useEffect(() => {
    if (steps[currentStep] && steps[currentStep].hasPulse) {
      pulseAnim.setValue(1);
      // Cria a animação de loop
      const pulseLoop = Animated.loop(
        Animated.sequence([
          Animated.timing(pulseAnim, {
            toValue: 1.15,
            duration: 1000,
            useNativeDriver: true,
          }),
          Animated.timing(pulseAnim, {
            toValue: 1,
            duration: 1000,
            useNativeDriver: true,
          }),
        ])
      );
      
      pulseLoop.start();

      // Limpeza da animação ao mudar de passo ou desmontar
      return () => pulseLoop.stop();
    }
  }, [currentStep]);

  const handleNext = () => {
    // Fade Out
    Animated.timing(fadeAnim, {
      toValue: 0,
      duration: 200,
      useNativeDriver: true,
    }).start(() => {
      // Troca o passo ou navega
      if (currentStep < steps.length - 1) {
        setCurrentStep((prev) => prev + 1);
        
        // Fade In
        Animated.timing(fadeAnim, {
          toValue: 1,
          duration: 200,
          useNativeDriver: true,
        }).start();
      } else {
        router.push("/(tabs)/chatIntro");
      }
    });
  };

  const currentStepData = steps[currentStep];

  if (!currentStepData) {
    return null;
  }

  const currentStyles = currentStepData.customStyles || {};

  return (
    <View style={styles.container}>
      <StatusBar barStyle="light-content" backgroundColor="#161221" />
      <SafeAreaView style={styles.safeArea}>
        
        <View style={styles.header}>
          <Image source={Logo} style={styles.logo} resizeMode="contain" />
          <Text style={styles.headerSubtitle}>Tutorial</Text>
        </View>

        <Animated.View style={[styles.content, { opacity: fadeAnim }]}>
          
          <View style={styles.imageContainer}>
            <Image 
              source={steps[currentStep].image} 
              style={[styles.illustration, currentStyles.image]} 
              resizeMode="contain" 
            />

            {steps[currentStep].hasPulse && (
              <Animated.View
                style={[
                  styles.pulseTarget,
                  currentStyles.pulse, 
                  { transform: [{ scale: pulseAnim }] }
                ]}
              />
            )}
          </View>

          <View style={styles.footerContainer}>
            <View style={[styles.textBlock, currentStyles.textBlock]}>
              <Text style={[styles.titleText, currentStyles.title]}>
                {steps[currentStep].title}
              </Text>
              <Text style={[styles.subtitleText, currentStyles.subtitle]}>
                {steps[currentStep].subtitle}
              </Text>
            </View>

            <TouchableOpacity 
              style={styles.actionButton} 
              onPress={handleNext}
              activeOpacity={0.7}
            >
              <Ionicons name="chevron-forward" size={28} color="#FFF" />
            </TouchableOpacity>
          </View>
          
        </Animated.View>
      </SafeAreaView>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#1D182F",
  },
  safeArea: {
    flex: 1,
    paddingTop: Platform.OS === 'android' ? 30 : 0,
  },
  header: {
    alignItems: "center",
    marginTop: 10,
    height: height * 0.1, 
    justifyContent: 'flex-start',
  },
  logo: {
    width: 90,
    height: 35,
    marginBottom: 4,
  },
  headerSubtitle: {
    color: "#FFFFFF",
    fontSize: 14,
    fontWeight: "300",
    letterSpacing: 0.5,
  },
  content: {
    flex: 1,
  },
  imageContainer: {
    flex: 1, 
    justifyContent: "center",
    alignItems: "center",
    position: 'relative',
    paddingHorizontal: 20,
    paddingBottom: 0, 
  },
  illustration: {
    width: "100%",
    flex: 1, 
    maxHeight: 180, 
  },
  pulseTarget: {
    position: "absolute",
    top: "40%",  
    left: "25%", 
    width: 150,
    height: 150,
    borderRadius: 75,
    borderWidth: 2,
    borderColor: "rgba(255,255,255,0.8)",
    backgroundColor: "rgba(255,255,255,0.2)",
    zIndex: 10,
  },
  footerContainer: {
    flexDirection: "row",
    alignItems: "flex-end", 
    justifyContent: "space-between",
    paddingHorizontal: 32,
    paddingBottom: 100, 
    paddingLeft: 60, 
  },
  textBlock: {
    flex: 1,
    paddingRight: 15,
    marginBottom: 120, 
  },
  titleText: {
    color: "#FFFFFF",
    fontSize: 52,      
    lineHeight: 58,   
    fontWeight: "800",  
    textTransform: "uppercase",
    marginBottom: 8,
  },
  subtitleText: {
    color: "#E0E0E0",
    fontSize: 30,       
    fontWeight: "400",
    textTransform: "uppercase",
    lineHeight: 34, 
  },
  actionButton: {
    width: 120, 
    height: 120,
    borderRadius: 60,
    backgroundColor: "#8B2CF5",
    justifyContent: "center",
    alignItems: "center",
    shadowColor: "#8B2CF5",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.5,
    shadowRadius: 10,
    elevation: 8,
    marginBottom: 5, 
  },
});