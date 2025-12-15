import Navbar from "@/components/navbar";
import { Ionicons } from "@expo/vector-icons";
import { StatusBar } from "expo-status-bar";
import React, { useState } from "react";
import { Animated, Image, ScrollView, StyleSheet, Text, TouchableOpacity, View } from "react-native";

const Logo = require("../../assets/images/logo-branca.png");

interface FAQItem {
  id: number;
  question: string;
  answer: string;
}

const FAQ_DATA: FAQItem[] = [
  {
    id: 1,
    question: "Qual o tempo de duração do tour?",
    answer: "O tour dura em média 15 minutos, podendo variar de acordo com o ritmo do grupo e perguntas realizadas.",
  },
  {
    id: 2,
    question: "Posso pausar o tour para ir ao banheiro?",
    answer: "Caso precise pausar o tour, avise ao acompanhante que ele irá pausar o percurso para você.",
  },
  {
    id: 3,
    question: "Quantas pessoas podem participar do tour ao mesmo tempo?",
    answer: "O tour comporta grupos de até 4 ou 5 pessoas por vez para garantir uma experiência de qualidade.",
  },
  {
    id: 4,
    question: "Posso tirar fotos durante o tour?",
    answer: "Sim! Fique à vontade para registrar momentos durante o tour. Pedimos apenas que mantenha a distância segura do robô.",
  },
  {
    id: 5,
    question: "O tour é acessível para pessoas com mobilidade reduzida?",
    answer: "Sim! Todo o percurso foi planejado para ser totalmente acessível. Além disso, todas as respostas do robô também são transcritas na tela do aplicativo.",
  },
  {
    id: 6,
    question: "O que fazer em caso de emergência durante o tour?",
    answer: "Em caso de emergência, utilize o botão de emergência no aplicativo para solicitar ajuda imediata da equipe Inteli.",
  }, 
  {
    id: 7,
    question: "Como posso fazer uma pergunta ao robô durante o tour?",
    answer: "Durante o tour, você pode fazer perguntas ao robô utilizando o recurso de áudio no aplicativo. Basta clicar no ícone de microfone para gravar sua pergunta. Você também pode digitar sua pergunta por texto.",
  },
  {
    id: 8,
    question: "Quando o robô irá responder as minhas perguntas?",
    answer: "A LIA é programada para responder suas perguntas ao final de cada etapa do tour, garantindo que você receba as informações necessárias antes de prosseguir.",
  }
];

function FAQAccordion({ item }: { item: FAQItem }) {
  const [isOpen, setIsOpen] = useState(false);
  const [animation] = useState(new Animated.Value(0));

  const toggleAccordion = () => {
    const toValue = isOpen ? 0 : 1;
    
    Animated.timing(animation, {
      toValue,
      duration: 300,
      useNativeDriver: false,
    }).start();
    
    setIsOpen(!isOpen);
  };

  const heightInterpolate = animation.interpolate({
    inputRange: [0, 1],
    outputRange: [0, 200], // Ajuste conforme necessário
  });

  const rotateInterpolate = animation.interpolate({
    inputRange: [0, 1],
    outputRange: ['0deg', '180deg'],
  });

  return (
    <View style={styles.accordionContainer}>
      <TouchableOpacity 
        style={styles.accordionHeader}
        onPress={toggleAccordion}
        activeOpacity={0.8}
      >
        <View style={styles.questionContainer}>
          <Ionicons name="help-circle-outline" size={24} color="#8141C2" />
          <Text style={styles.questionText}>{item.question}</Text>
        </View>
        <Animated.View style={{ transform: [{ rotate: rotateInterpolate }] }}>
          <Ionicons name="chevron-down" size={24} color="#FFFFFF" />
        </Animated.View>
      </TouchableOpacity>

      <Animated.View 
        style={[
          styles.accordionContent,
          { 
            maxHeight: heightInterpolate,
            opacity: animation,
          }
        ]}
      >
        <View style={styles.answerContainer}>
          <Ionicons name="checkmark-circle" size={20} color="#00E676" />
          <Text style={styles.answerText}>{item.answer}</Text>
        </View>
      </Animated.View>
    </View>
  );
}

export default function FAQ() {
  return (
    <>
      <StatusBar hidden />
      <View style={styles.container}>
        {/* Logo do Inteli no topo */}
        <View style={styles.header}>
          <Image source={Logo} style={styles.logo} resizeMode="contain" />
        </View>

        {/* Título */}
        <Text style={styles.title}>Perguntas Frequentes</Text>
        <Text style={styles.subtitle}>Tire suas dúvidas sobre o tour</Text>

        {/* Lista de FAQs */}
        <ScrollView 
          style={styles.scrollView}
          contentContainerStyle={styles.scrollContent}
          showsVerticalScrollIndicator={false}
        >
          {FAQ_DATA.map((item) => (
            <FAQAccordion key={item.id} item={item} />
          ))}
          
          {/* Espaço extra no final para a navbar */}
          <View style={{ height: 120 }} />
        </ScrollView>

        {/* Navbar */}
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
  title: {
    fontSize: 28,
    fontWeight: "bold",
    color: "#FFFFFF",
    textAlign: "center",
    marginBottom: 8,
  },
  subtitle: {
    fontSize: 16,
    color: "#FFFFFF",
    textAlign: "center",
    marginBottom: 20,
    opacity: 0.8,
  },
  scrollView: {
    flex: 1,
    paddingHorizontal: 20,
  },
  scrollContent: {
    paddingBottom: 20,
  },
  accordionContainer: {
    marginBottom: 15,
    backgroundColor: "rgba(106, 64, 196, 0.15)",
    borderRadius: 15,
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.1)",
    overflow: "hidden",
  },
  accordionHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    padding: 18,
  },
  questionContainer: {
    flexDirection: "row",
    alignItems: "center",
    flex: 1,
    gap: 12,
  },
  questionText: {
    fontSize: 16,
    fontWeight: "600",
    color: "#FFFFFF",
    flex: 1,
  },
  accordionContent: {
    overflow: "hidden",
  },
  answerContainer: {
    flexDirection: "row",
    padding: 18,
    paddingTop: 0,
    gap: 10,
    alignItems: "flex-start",
  },
  answerText: {
    fontSize: 15,
    color: "#FFFFFF",
    lineHeight: 22,
    opacity: 0.9,
    flex: 1,
  },
});
