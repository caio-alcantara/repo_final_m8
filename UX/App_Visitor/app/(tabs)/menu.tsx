import Navbar from "@/components/navbar";
import { Ionicons } from "@expo/vector-icons";
import { useRouter } from "expo-router";
import { StatusBar } from "expo-status-bar";
import React, { useState } from "react";
import { Image, Modal, ScrollView, StyleSheet, Text, TouchableOpacity, View, ActivityIndicator } from "react-native";

import { useTour } from "@/context/TourContext";
import { tourService } from "../../services/tourService";

const Logo = require("../../assets/images/logo-branca.png");
const APP_VERSION = "1.0.0";

export default function Menu() {
  const router = useRouter();
  const { tour } = useTour();

  const [showAboutModal, setShowAboutModal] = useState(false);
  const [showEndTourModal, setShowEndTourModal] = useState(false);
  const [isEnding, setIsEnding] = useState(false);

  const handleEndTour = () => {
    setShowEndTourModal(true);
  };

  const confirmEndTour = async () => {
    setIsEnding(true);

    try {
      if (tour?.tourId) {
        console.log("Encerrando tour:", tour.tourId);

        const currentTourData = await tourService.getTourById(tour.tourId);
        
        const { id, ...tourDataWithoutId } = currentTourData;

        await tourService.updateTour(tour.tourId, {
          ...tourDataWithoutId,
          status: 'completed',
          fim_real: new Date().toISOString()
        });

        console.log("Tour encerrado com sucesso!");
      }
    } catch (error) {
      console.error("Erro ao encerrar tour (prosseguindo para NPS):", error);
    } finally {
      setIsEnding(false);
      setShowEndTourModal(false);
      router.push("/nps");
    }
  };

  return (
    <>
      <StatusBar hidden />
      <View style={styles.container}>
        <View style={styles.header}>
          <Image source={Logo} style={styles.logo} resizeMode="contain" />
        </View>

        <Text style={styles.title}>Menu</Text>

        <View style={styles.content}>
          <TouchableOpacity
            style={styles.menuButton}
            onPress={() => setShowAboutModal(true)}
            activeOpacity={0.8}
          >
            <View style={styles.buttonContent}>
              <Ionicons name="information-circle-outline" size={32} color="#8141C2" />
              <Text style={styles.buttonText}>Sobre o App</Text>
            </View>
            <Ionicons name="chevron-forward" size={24} color="#FFFFFF" />
          </TouchableOpacity>

          <TouchableOpacity
            style={[styles.menuButton, styles.endTourButton]}
            onPress={handleEndTour}
            activeOpacity={0.8}
          >
            <View style={styles.buttonContent}>
              <Ionicons name="exit-outline" size={32} color="#FF4B4B" />
              <Text style={[styles.buttonText, styles.endTourText]}>Encerrar Tour</Text>
            </View>
            <Ionicons name="chevron-forward" size={24} color="#FF4B4B" />
          </TouchableOpacity>
        </View>

        <View style={styles.versionContainer}>
          <Text style={styles.versionText}>Versão {APP_VERSION}</Text>
        </View>

        <Modal
          visible={showAboutModal}
          transparent={true}
          animationType="fade"
          onRequestClose={() => setShowAboutModal(false)}
        >
          <View style={styles.modalOverlay}>
            <View style={styles.modalContent}>
              {/* Cabeçalho do modal */}
              <View style={styles.modalHeader}>
                <Text style={styles.modalTitle}>Sobre o App</Text>
                <TouchableOpacity onPress={() => setShowAboutModal(false)}>
                  <Ionicons name="close-circle" size={32} color="#8141C2" />
                </TouchableOpacity>
              </View>

              <ScrollView
                style={styles.modalScroll}
                showsVerticalScrollIndicator={false}
              >
                <View style={styles.modalLogoContainer}>
                  <Image source={Logo} style={styles.modalLogo} resizeMode="contain" />
                </View>

                <View style={styles.modalSection}>
                  <View style={styles.sectionHeader}>
                    <Ionicons name="bulb-outline" size={24} color="#8141C2" />
                    <Text style={styles.sectionTitle}>O Projeto</Text>
                  </View>
                  <Text style={styles.modalText}>
                    Programação de um robô autônomo para realizar tours interativas e apresentar o campus do Inteli de forma dinâmica e inesquecível.
                  </Text>
                </View>

                <View style={styles.modalSection}>
                  <View style={styles.sectionHeader}>
                    <Ionicons name="flag-outline" size={24} color="#8141C2" />
                    <Text style={styles.sectionTitle}>Objetivo</Text>
                  </View>
                  <Text style={styles.modalText}>
                    Tornar a experiência de conhecer o campus mais dinâmica para potenciais alunos, utilizando IA generativa e robótica para criar uma apresentação única do espaço físico, metodologia de ensino e processo seletivo.
                  </Text>
                </View>

                <View style={styles.modalSection}>
                  <View style={styles.sectionHeader}>
                    <Ionicons name="school-outline" size={24} color="#8141C2" />
                    <Text style={styles.sectionTitle}>Sobre o Inteli</Text>
                  </View>
                  <Text style={styles.modalText}>
                    Somos o Instituto de Tecnologia e Liderança. Nosso modelo educacional forma grandes lideranças através do desenvolvimento de projetos reais, integrando computação, negócios e liderança de forma prática, colaborativa e criativa.
                  </Text>
                </View>

                <View style={styles.modalSection}>
                  <View style={styles.sectionHeader}>
                    <Text style={styles.sectionTitle}>Versão</Text>
                  </View>
                  <Text style={styles.modalText}>
                    Versão 1.0.0.
                    </Text>
                    <Text style={styles.modalText}>
                      Direitos reservados pelo Instituto de Tecnologia e Liderança - Inteli
                    </Text>
                </View>

                <View style={{ height: 20 }} />
              </ScrollView>
            </View>
          </View>
        </Modal>

        <Modal
          visible={showEndTourModal}
          transparent={true}
          animationType="fade"
          onRequestClose={() => {
             if(!isEnding) setShowEndTourModal(false);
          }}
        >
          <View style={styles.modalOverlay}>
            <View style={styles.confirmModalContent}>
              <View style={styles.alertIconContainer}>
                <Ionicons name="warning" size={80} color="#FF4B4B" />
              </View>

              <Text style={styles.confirmTitle}>Encerrar Tour</Text>

              <Text style={styles.confirmMessage}>
                Tem certeza que deseja encerrar o tour? Você voltará para a tela inicial.
              </Text>

              <View style={styles.confirmButtons}>
                <TouchableOpacity
                  style={[styles.confirmButton, styles.cancelButton]}
                  onPress={() => {
                    if (!isEnding) {
                      console.log("Cancelado");
                      setShowEndTourModal(false);
                    }
                  }}
                  activeOpacity={0.8}
                  disabled={isEnding}
                >
                  <Text style={styles.cancelButtonText}>Cancelar</Text>
                </TouchableOpacity>

                <TouchableOpacity
                  style={[styles.confirmButton, styles.endButton]}
                  onPress={confirmEndTour}
                  activeOpacity={0.8}
                  disabled={isEnding}
                >
                  {isEnding ? (
                    <ActivityIndicator size="small" color="#FFF" />
                  ) : (
                    <Text style={styles.endButtonText}>Encerrar</Text>
                  )}
                </TouchableOpacity>
              </View>
            </View>
          </View>
        </Modal>
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
    marginBottom: 40,
  },
  content: {
    flex: 1,
    paddingHorizontal: 30,
    paddingTop: 20,
    gap: 20,
  },
  menuButton: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    backgroundColor: "rgba(106, 64, 196, 0.15)",
    padding: 20,
    borderRadius: 15,
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.1)",
  },
  endTourButton: {
    backgroundColor: "rgba(255, 75, 75, 0.15)",
    borderColor: "rgba(255, 75, 75, 0.3)",
  },
  buttonContent: {
    flexDirection: "row",
    alignItems: "center",
    gap: 15,
  },
  buttonText: {
    fontSize: 18,
    fontWeight: "600",
    color: "#FFFFFF",
  },
  endTourText: {
    color: "#FF4B4B",
  },
  versionContainer: {
    paddingBottom: 120,
    alignItems: "center",
  },
  versionText: {
    fontSize: 14,
    color: "#FFFFFF",
    opacity: 0.5,
  },
  modalOverlay: {
    flex: 1,
    backgroundColor: "rgba(0, 0, 0, 0.8)",
    justifyContent: "center",
    alignItems: "center",
    padding: 20,
  },
  modalContent: {
    backgroundColor: "#1E1730",
    borderRadius: 20,
    width: "100%",
    maxWidth: 600,
    maxHeight: "85%",
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.1)",
  },
  modalHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    padding: 20,
    borderBottomWidth: 1,
    borderBottomColor: "rgba(255, 255, 255, 0.1)",
  },
  modalTitle: {
    fontSize: 24,
    fontWeight: "bold",
    color: "#FFFFFF",
  },
  modalScroll: {
    padding: 20,
  },
  modalLogoContainer: {
    alignItems: "center",
    marginBottom: 25,
  },
  modalLogo: {
    width: 100,
    height: 80,
  },
  modalSection: {
    marginBottom: 25,
  },
  sectionHeader: {
    flexDirection: "row",
    alignItems: "center",
    gap: 10,
    marginBottom: 12,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: "bold",
    color: "#FFFFFF",
  },
  modalText: {
    fontSize: 15,
    color: "#FFFFFF",
    lineHeight: 24,
    opacity: 0.9,
  },
  boldText: {
    fontWeight: "bold",
    color: "#8141C2",
  },

  confirmModalContent: {
    backgroundColor: "#1E1730",
    borderRadius: 20,
    width: "90%",
    maxWidth: 450,
    padding: 30,
    borderWidth: 1,
    borderColor: "rgba(255, 75, 75, 0.3)",
    alignItems: "center",
  },
  alertIconContainer: {
    marginBottom: 20,
  },
  confirmTitle: {
    fontSize: 26,
    fontWeight: "bold",
    color: "#FFFFFF",
    marginBottom: 15,
    textAlign: "center",
  },
  confirmMessage: {
    fontSize: 16,
    color: "#FFFFFF",
    lineHeight: 24,
    opacity: 0.9,
    textAlign: "center",
    marginBottom: 30,
  },
  confirmButtons: {
    flexDirection: "row",
    gap: 15,
    width: "100%",
  },
  confirmButton: {
    flex: 1,
    paddingVertical: 15,
    borderRadius: 12,
    alignItems: "center",
  },
  cancelButton: {
    backgroundColor: "rgba(106, 64, 196, 0.2)",
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.2)",
  },
  cancelButtonText: {
    color: "#FFFFFF",
    fontSize: 16,
    fontWeight: "600",
  },
  endButton: {
    backgroundColor: "#FF4B4B",
  },
  endButtonText: {
    color: "#FFFFFF",
    fontSize: 16,
    fontWeight: "bold",
  },
});