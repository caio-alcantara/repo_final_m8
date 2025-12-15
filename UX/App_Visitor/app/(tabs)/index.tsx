// app/index.tsx (ou app/login.tsx)
import { Manrope_700Bold } from "@expo-google-fonts/manrope/700Bold";
import { useFonts } from "@expo-google-fonts/manrope/useFonts";
import { Image } from "expo-image";
import { router } from "expo-router";
import { useState } from "react";
import {
  Alert,
  ActivityIndicator,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";

// Imagens
import Auditorio from "../../assets/images/Login/Auditório.jpg";
import Casinhas from "../../assets/images/Login/Casinhas.jpg";
import Frente from "../../assets/images/Login/Frente.jpg";
import Letreiro from "../../assets/images/Login/Letreiro.jpeg";
import Pessoas from "../../assets/images/Login/Pessoas.jpeg";
import Refeitorio from "../../assets/images/Login/Refeitorio.jpg";
import Logo from "../../assets/images/logo-branca.png";

import { useTour } from "@/context/TourContext";

// Componentes e serviços
import AccessCodeInput from "../../components/code";
import { tourService } from "../../services/tourService";
import { checkpointService } from "../../services/checkpointService";
import {
  getTourVisitanteByTourId,
  getVisitanteById,
} from "../../services/visitanteService";

function randomPhoto(max: number) {
  return Math.floor(Math.random() * max);
}

const photos = [Casinhas, Auditorio, Frente, Letreiro, Pessoas, Refeitorio];
const numberRandom = randomPhoto(photos.length);
const photo = photos[numberRandom];

export default function HomeScreen() {
  const [code, setCode] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  const { setTour } = useTour();

  const [fontsLoaded] = useFonts({
    Manrope_700Bold,
  });

  const handleLogin = async () => {
    if (!code || code.trim().length === 0) {
      Alert.alert("❌ Código incorreto", "Por favor, digite o código de acesso.");
      return;
    }

    setIsLoading(true);

    try {
      console.log("🔐 Tentando login com código:", code);

      // 1) Login do tour
      const data = await tourService.loginByCode(code);
      const tour = data.tour;

      console.log("✅ Login bem-sucedido!");
      console.log("📋 Tour:", tour);
      console.log("👥 Visitantes (loginByCode):", data.visitantes);

      // 2) Buscar checkpoints do tour
      let checkpointId: number | null = null;
      try {
        const checkpoints = await checkpointService.getByTourId(tour.id!);
        const currentCheckpoint = checkpointService.getCurrent(checkpoints);

        console.log("📍 Checkpoints:", checkpoints);
        console.log("✅ Checkpoint atual:", currentCheckpoint);

        checkpointId = currentCheckpoint?.id ?? null;
      } catch (err: any) {
        console.log(
          "⚠️ Erro ao buscar checkpoints. Seguindo sem checkpoint:",
          err?.message
        );
        checkpointId = null;
      }

      // 3) Resolver nome do visitante
      let visitorName = "Visitante";

      try {
        // GET /v1/tour-visitante/tour/{tourId}
        const rel = await getTourVisitanteByTourId(tour.id!);
        if (rel?.visitante_id) {
          // GET /v1/visitante/{id}
          const visitante = await getVisitanteById(rel.visitante_id);
          if (visitante?.nome) {
            visitorName = visitante.nome;
          }
        } else {
          console.log(
            "[login] Nenhum vínculo tour-visitante retornado, vou tentar fallback do loginByCode."
          );
        }
      } catch (err) {
        console.log(
          "[login] Erro ao buscar tour-visitante/visitante, usando fallback:",
          err
        );
      }

      // Fallback: usar visitantes vindos do loginByCode, se existirem
      if (
        visitorName === "Visitante" &&
        Array.isArray(data.visitantes) &&
        data.visitantes.length > 0
      ) {
        visitorName = data.visitantes[0].nome || "Visitante";
      }

      console.log("🙋‍♀️ visitorName resolvido como:", visitorName);

      // 4) Salvar no contexto global
      setTour({
        tourId: tour.id ?? null,
        checkpointId,
        visitorName,
      });

      // 5) Navegar para onboarding / intro do chat
      router.push({
        pathname: "/(tabs)/onboarding",
        params: {
          tourId: tour.id?.toString() || "",
          tourCode: tour.codigo,
          tourTitle: tour.titulo || "Tour",
          visitorName: visitorName,
          visitorCount: data.visitantes?.length?.toString() ?? "1",
        },
      });
    } catch (error: any) {
      console.error("❌ Erro no login:", error);

      let errorMessage = "Por favor, verifique o código de acesso.";

      if (error?.message === "Tour não encontrado com o código fornecido") {
        errorMessage = "Não encontramos um tour com este código.";
      } else if (error?.message?.includes("Erro de conexão")) {
        errorMessage = "Verifique sua conexão com a internet.";
      }

      Alert.alert("❌ Código incorreto", errorMessage);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <View style={styles.container}>
      <Image
        source={photo}
        style={StyleSheet.absoluteFillObject}
        contentFit="cover"
        transition={1000}
      />

      <View style={styles.overlay}>
        <TouchableOpacity style={styles.loginButton}>
          <Image
            source={Logo}
            style={styles.logo}
            contentFit="contain"
            transition={1000}
          />

          <AccessCodeInput value={code} onChangeText={setCode} />

          <TouchableOpacity
            style={styles.button}
            onPress={handleLogin}
            disabled={isLoading}
          >
            {isLoading ? (
              <ActivityIndicator color="#fff" size="small" />
            ) : (
              <Text style={styles.buttonText}>Entrar</Text>
            )}
          </TouchableOpacity>
        </TouchableOpacity>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1 },
  text: {
    fontFamily: "Manrope_700Bold",
    color: "#fff",
  },
  overlay: {
    flex: 1,
    justifyContent: "center",
    alignItems: "center",
    backgroundColor: "rgba(39, 32, 54, 0.6)",
  },
  loginButton: {
    position: "absolute",
    bottom: 20,
    alignSelf: "center",
    backgroundColor: "#272036dc",
    borderRadius: 35,
    padding: 20,
    width: "40%",
    height: "40%",
  },
  logo: {
    width: 120,
    height: 100,
    alignSelf: "center",
    bottom: 10,
  },
  button: {
    backgroundColor: "#8141C2",
    borderRadius: 20,
    width: "32%",
    height: "25%",
    borderWidth: 0,
    justifyContent: "center",
    alignItems: "center",
    alignSelf: "center",
  },
  buttonText: {
    color: "#fff",
    fontFamily: "Manrope_700Bold",
  },
});
