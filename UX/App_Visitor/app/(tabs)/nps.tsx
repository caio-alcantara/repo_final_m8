import React, { useState, useRef } from "react";
import {
  View,
  Text,
  StyleSheet,
  TouchableOpacity,
  Animated,
  Image,
  TextInput,
  KeyboardAvoidingView,
  Platform,
  ScrollView,
  Alert,
  Modal,
} from "react-native";
import { useRouter } from "expo-router";
import { MaterialCommunityIcons } from "@expo/vector-icons";

// Contexto e Imagens
import { useTour } from "@/context/TourContext";
import Logo from "../../assets/images/logo-branca.png";

const FEEDBACK_OPTIONS = [
  { id: 1, icon: "emoticon-dead", color: "#FF3B30", label: "Péssimo" },
  { id: 2, icon: "emoticon-sad", color: "#FF9500", label: "Ruim" },
  { id: 3, icon: "emoticon-neutral", color: "#FFCC00", label: "Regular" },
  { id: 4, icon: "emoticon-happy", color: "#34C759", label: "Bom" },
  { id: 5, icon: "emoticon-excited", color: "#00E676", label: "Incrível!" },
];

export default function NPS() {
  const router = useRouter();
  const { setTour } = useTour();

  const [selectedOption, setSelectedOption] = useState<number>(3);
  const [comment, setComment] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);
  
  const [showIntroModal, setShowIntroModal] = useState(true);

  // Animações
  const heroScale = useRef(new Animated.Value(1)).current;
  const heroOpacity = useRef(new Animated.Value(1)).current;
  const inputOpacity = useRef(new Animated.Value(1)).current; 

  const currentFeedback = FEEDBACK_OPTIONS.find((opt) => opt.id === selectedOption)!;

  const animateHero = () => {
    heroScale.setValue(0.8);
    heroOpacity.setValue(0.5);

    Animated.parallel([
      Animated.spring(heroScale, {
        toValue: 1,
        friction: 4,
        tension: 50,
        useNativeDriver: true,
      }),
      Animated.timing(heroOpacity, {
        toValue: 1,
        duration: 200,
        useNativeDriver: true,
      }),
    ]).start();
  };

  const handleSelect = (id: number) => {
    if (selectedOption !== id) {
      setSelectedOption(id);
      setTimeout(() => animateHero(), 0);
    }
  };

  const handleSubmit = async () => {
    setIsSubmitting(true);
    
    const payload = {
        score: selectedOption,
        feedback: comment,
    };
    console.log("🚀 Avaliação enviada:", payload);

    // Simulação de delay
    await new Promise(resolve => setTimeout(resolve, 1000));

    Alert.alert(
        "Obrigado!",
        "Sua opinião é fundamental para a evolução da LIA.",
        [
            {
                text: "Encerrar",
                onPress: () => {
                    // 1. Limpa o contexto (Logout do tour)
                    setTour(null);

                    // 2. CORREÇÃO DO ERRO POP_TO_TOP:
                    // Removemos o router.dismissAll() que causava o crash.
                    // Usamos apenas replace("/") para trocar a tela atual pela Home (Login).
                    // Se houver modais abertos, verificamos antes de fechar.
                    if (router.canDismiss()) {
                        router.dismissAll();
                    }
                    
                    // Vai para a raiz (Login) e substitui o histórico
                    router.replace("/");
                }
            }
        ]
    );
    setIsSubmitting(false);
  };

  return (
    <KeyboardAvoidingView 
      style={{ flex: 1, backgroundColor: "#1E1730" }} 
      behavior={Platform.OS === "ios" ? "padding" : "height"}
    >
      <ScrollView 
        contentContainerStyle={{ flexGrow: 1 }} 
        keyboardShouldPersistTaps="handled"
        style={{ backgroundColor: "#1E1730" }} 
      >
        <View style={styles.container}>
          
          <View style={styles.header}>
            <Image source={Logo} style={styles.logo} resizeMode="contain" />
          </View>

          <View style={styles.content}>
            <Text style={styles.subtitle}>
              Como você avalia sua experiência com a LIA?
            </Text>

            <View style={styles.heroContainer}>
              <Animated.View
                style={{
                  transform: [{ scale: heroScale }],
                  opacity: heroOpacity,
                  shadowColor: currentFeedback.color,
                  shadowOffset: { width: 0, height: 0 },
                  shadowOpacity: 0.6,
                  shadowRadius: 20,
                }}
              >
                <MaterialCommunityIcons
                  name={currentFeedback.icon as any}
                  size={120}
                  color={currentFeedback.color}
                />
              </Animated.View>

              <Text
                style={[
                  styles.heroText,
                  { color: currentFeedback.color },
                ]}
              >
                {currentFeedback.label}
              </Text>
            </View>

            {/* Seleção de Notas */}
            <View style={styles.selectionRow}>
              {FEEDBACK_OPTIONS.map((item) => {
                const isSelected = selectedOption === item.id;

                return (
                  <TouchableOpacity
                    key={item.id}
                    activeOpacity={0.7}
                    onPress={() => handleSelect(item.id)}
                    style={[
                      styles.emojiButton,
                      isSelected && {
                        backgroundColor: "rgba(255,255,255,0.1)",
                        borderColor: item.color,
                        transform: [{ scale: 1.1 }],
                        shadowColor: item.color,
                        shadowOpacity: 0.5,
                        shadowRadius: 10,
                        elevation: 5,
                        borderWidth: 10,
                      },
                    ]}
                  >
                    <MaterialCommunityIcons
                      name={item.icon as any}
                      size={32}
                      color={isSelected ? item.color : "rgba(255,255,255,0.3)"}
                    />
                  </TouchableOpacity>
                );
              })}
            </View>

            {/* Campo de Texto */}
            <Animated.View style={[styles.inputContainer, { opacity: inputOpacity }]}>
                <Text style={styles.inputLabel}>Gostaria de deixar um comentário? (Opcional)</Text>
                <TextInput 
                    style={styles.textInput}
                    placeholder="Digite aqui sua sugestão, elogio ou crítica..."
                    placeholderTextColor="rgba(255,255,255,0.4)"
                    multiline
                    textAlignVertical="top"
                    value={comment}
                    onChangeText={setComment}
                />
            </Animated.View>

            {/* Botão Enviar */}
            <TouchableOpacity
              style={[
                styles.submitButton,
                { backgroundColor: currentFeedback.color },
              ]}
              disabled={isSubmitting}
              onPress={handleSubmit}
            >
              <Text style={styles.submitText}>
                {isSubmitting ? "Enviando..." : "Enviar avaliação"}
              </Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* --- MODAL DE INSTRUÇÃO --- */}
        <Modal
          visible={showIntroModal}
          transparent={true}
          animationType="fade"
          statusBarTranslucent
        >
          <View style={styles.modalOverlay}>
            <View style={styles.modalContent}>
                <View style={styles.modalIconContainer}>
                    <MaterialCommunityIcons name="star-face" size={50} color="#8141C2" />
                </View>
                
                <Text style={styles.modalTitle}>Sua opinião importa!</Text>
                
                <Text style={styles.modalText}>
                    Queremos melhorar cada vez mais a LIA. {"\n\n"}
                    Por favor, use a escala de emojis para avaliar como foi sua experiência durante o tour pelo Inteli.
                </Text>

                <View style={styles.scaleExplanation}>
                    <View style={styles.scaleItem}>
                        <MaterialCommunityIcons name="emoticon-dead" size={24} color="#FF3B30" />
                        <Text style={styles.scaleLabel}>Péssimo</Text>
                    </View>
                    <View style={styles.arrowLine} />
                    <View style={styles.scaleItem}>
                        <MaterialCommunityIcons name="emoticon-excited" size={24} color="#00E676" />
                        <Text style={styles.scaleLabel}>Incrível</Text>
                    </View>
                </View>

                <TouchableOpacity 
                    style={styles.modalButton}
                    onPress={() => setShowIntroModal(false)}
                >
                    <Text style={styles.modalButtonText}>Avaliar agora</Text>
                </TouchableOpacity>
            </View>
          </View>
        </Modal>

      </ScrollView>
    </KeyboardAvoidingView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#1E1730",
    minHeight: "100%",
  },
  header: {
    alignItems: "center",
    marginTop: 60,
    marginBottom: -20,
  },
  logo: {
    width: 100,
    height: 80,
  },
  content: {
    flex: 1,
    alignItems: "center",
    paddingHorizontal: 24,
    paddingBottom: 40,
  },
  title: {
    fontSize: 26,
    fontWeight: "800",
    color: "#fff",
    marginBottom: 8,
    textAlign: "center",
    marginTop: 20,
  },
  subtitle: {
    marginTop: 30, 
    fontSize: 20,
    color: "#CFCFCF",
    textAlign: "center",
    marginBottom: 20,
    lineHeight: 22,
  },
  heroContainer: {
    height: 160,
    justifyContent: "center",
    alignItems: "center",
    marginBottom: 20,
  },
  heroText: {
    fontSize: 24,
    fontWeight: "700",
    marginTop: 16,
    textTransform: "uppercase",
    letterSpacing: 1.5,
  },
  selectionRow: {
    flexDirection: "row",
    justifyContent: "space-between",
    width: "100%",
    marginBottom: 20,
  },
  emojiButton: {
    width: 100,
    height: 100,
    borderRadius: 50,
    justifyContent: "center",
    alignItems: "center",
    backgroundColor: "rgba(255,255,255,0.03)",
    borderWidth: 10,
    borderColor: "rgba(255,255,255,0.1)",
    marginTop: 40, 
  },
  inputContainer: {
    width: "100%",
    marginBottom: 30,
  },
  inputLabel: {
    color: "#CFCFCF",
    fontSize: 14,
    marginBottom: 10,
    marginLeft: 4,
  },
  textInput: {
    width: "100%",
    height: 100,
    backgroundColor: "rgba(255,255,255,0.05)",
    borderRadius: 16,
    padding: 16,
    color: "#fff",
    fontSize: 16,
    borderWidth: 1,
    borderColor: "rgba(255,255,255,0.1)",
  },
  submitButton: {
    width: "100%",
    height: 56,
    borderRadius: 28,
    alignItems: "center",
    justifyContent: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.3,
    shadowRadius: 5,
    elevation: 6,
    marginBottom: 20,
  },

  submitText: {
    color: "#1E1730",
    fontWeight: "800",
    fontSize: 16,
    textTransform: "uppercase",
    letterSpacing: 1,
  },

  modalOverlay: {
    flex: 1,
    backgroundColor: "rgba(0,0,0,0.85)", 
    justifyContent: "center",
    alignItems: "center",
    padding: 20,
  },
  modalContent: {
    backgroundColor: "#272036",
    width: "100%",
    maxWidth: 340,
    borderRadius: 24,
    padding: 24,
    alignItems: "center",
    borderWidth: 1,
    borderColor: "rgba(255,255,255,0.1)",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 10 },
    shadowOpacity: 0.5,
    shadowRadius: 20,
    elevation: 10,
  },
  modalIconContainer: {
    width: 80,
    height: 80,
    borderRadius: 40,
    backgroundColor: "rgba(129, 65, 194, 0.15)",
    justifyContent: "center",
    alignItems: "center",
    marginBottom: 20,
  },
  modalTitle: {
    fontSize: 22,
    fontWeight: "800",
    color: "#fff",
    marginBottom: 12,
    textAlign: "center",
  },
  modalText: {
    fontSize: 15,
    color: "#CFCFCF",
    textAlign: "center",
    lineHeight: 22,
    marginBottom: 24,
  },
  scaleExplanation: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    width: "100%",
    marginBottom: 30,
    paddingHorizontal: 10,
  },
  scaleItem: {
    alignItems: "center",
    gap: 6,
  },
  scaleLabel: {
    color: "#fff",
    fontSize: 12,
    fontWeight: "600",
  },
  arrowLine: {
    flex: 1,
    height: 2,
    backgroundColor: "rgba(255,255,255,0.1)",
    marginHorizontal: 15,
  },
  modalButton: {
    width: "100%",
    backgroundColor: "#8141C2",
    paddingVertical: 16,
    borderRadius: 16,
    alignItems: "center",
  },
  modalButtonText: {
    color: "#fff",
    fontWeight: "700",
    fontSize: 16,
  },
});