import { View, StyleSheet, Platform, UIManager, ScrollView, Text } from "react-native"
import { Navbar } from "@/components/navbar";
import { Header } from "@/components/header";
import CardTour from "@/components/CardTour";
import { ConfirmDeleteModal } from "@/components/ConfirmDeleteModal";
import DateSelector from "@/components/DateSelector";
import { AddTourIcon } from "@/components/AddTourIcon";
import { useEffect, useState } from "react";
import { AddTourPopup } from "@/components/AddTourPopup";
import { tourService, type Tour as ApiTour } from "@/services/api";

if (Platform.OS === "android" && UIManager.setLayoutAnimationEnabledExperimental) {
  UIManager.setLayoutAnimationEnabledExperimental(true);
}

export type Tour = {
  id?: number;
  codigo: string;
  responsavel: string;
  responsavel_id?: number | null;
  status: "scheduled" | "in_progress" | "paused" | "finished" | "cancelled";
  data: string;
  hora_inicio_prevista: string;
  hora_fim_prevista: string;
  titulo?: string | null;
  robo_id?: number;
};

const initialTours: Tour[] = [
  {
    "codigo": "A1B2C3D4",
    "responsavel": "João Pereira",
    "status": "scheduled",
    "data": "25/11/2025",
    "hora_inicio_prevista": "09:00",
    "hora_fim_prevista": "10:00"
  },
  {
    "codigo": "E5F6G7H8",
    "responsavel": "Mariana Souza",
    "status": "in_progress",
    "data": "17/11/2025",
    "hora_inicio_prevista": "10:30",
    "hora_fim_prevista": "11:15"
  },
  {
    "codigo": "I9J0K1L2",
    "responsavel": "Lucas Andrade",
    "status": "paused",
    "data": "16/11/2025",
    "hora_inicio_prevista": "11:00",
    "hora_fim_prevista": "11:45"
  },
  {
    "codigo": "M3N4O5P6",
    "responsavel": "Fernanda Costa",
    "status": "finished",
    "data": "14/11/2025",
    "hora_inicio_prevista": "13:00",
    "hora_fim_prevista": "14:00"
  },
  {
    "codigo": "Q7R8S9T0",
    "responsavel": "Ricardo Lima",
    "status": "cancelled",
    "data": "20/11/2025",
    "hora_inicio_prevista": "14:30",
    "hora_fim_prevista": "15:30"
  },
  {
    "codigo": "U1V2W3X4",
    "responsavel": "Ana Bezerra",
    "status": "scheduled",
    "data": "27/11/2025",
    "hora_inicio_prevista": "08:00",
    "hora_fim_prevista": "09:00"
  },
  {
    "codigo": "Y5Z6A7B8",
    "responsavel": "Gabriel Nunes",
    "status": "in_progress",
    "data": "17/11/2025",
    "hora_inicio_prevista": "15:00",
    "hora_fim_prevista": "15:45"
  },
  {
    "codigo": "C9D0E1F2",
    "responsavel": "Carla Moura",
    "status": "finished",
    "data": "15/11/2025",
    "hora_inicio_prevista": "16:00",
    "hora_fim_prevista": "17:00"
  },
  {
    "codigo": "G3H4I5J6",
    "responsavel": "Pedro Alves",
    "status": "paused",
    "data": "17/11/2025",
    "hora_inicio_prevista": "09:30",
    "hora_fim_prevista": "10:15"
  },
  {
    "codigo": "K7L8M9N0",
    "responsavel": "Julia Fernandes",
    "status": "scheduled",
    "data": "28/11/2025",
    "hora_inicio_prevista": "11:30",
    "hora_fim_prevista": "12:30"
  },
  {
    "codigo": "O1P2Q3R4",
    "responsavel": "Tiago Ramos",
    "status": "finished",
    "data": "13/11/2025",
    "hora_inicio_prevista": "13:30",
    "hora_fim_prevista": "14:20"
  },
  {
    "codigo": "S5T6U7V8",
    "responsavel": "Larissa Rocha",
    "status": "cancelled",
    "data": "19/11/2025",
    "hora_inicio_prevista": "15:45",
    "hora_fim_prevista": "16:30"
  },
  {
    "codigo": "W9X0Y1Z2",
    "responsavel": "André Martins",
    "status": "in_progress",
    "data": "17/11/2025",
    "hora_inicio_prevista": "10:00",
    "hora_fim_prevista": "11:00"
  },
  {
    "codigo": "A3B4C5D6",
    "responsavel": "Patrícia Gomes",
    "status": "scheduled",
    "data": "26/11/2025",
    "hora_inicio_prevista": "08:30",
    "hora_fim_prevista": "09:15"
  },
  {
    "codigo": "E7F8G9H0",
    "responsavel": "Rafael Tavares",
    "status": "finished",
    "data": "12/11/2025",
    "hora_inicio_prevista": "17:00",
    "hora_fim_prevista": "17:45"
  }
]

function normalizeStatus(status: string | undefined): Tour["status"] {
  const allowed: Tour["status"][] = ["scheduled", "in_progress", "paused", "finished", "cancelled"];
  return allowed.includes(status as Tour["status"]) ? (status as Tour["status"]) : "scheduled";
}

function formatApiDate(apiDate: string) {
  if (!apiDate) return "";
  const parts = apiDate.split("-");
  if (parts.length === 3) {
    const [year, month, dayWithTime] = parts;
    const day = dayWithTime.split("T")[0];
    return `${day.padStart(2, "0")}/${month.padStart(2, "0")}/${year}`;
  }

  const parsed = new Date(apiDate);
  return Number.isNaN(parsed.getTime()) ? apiDate : parsed.toLocaleDateString("pt-BR");
}

function formatTime(value: string | null) {
  if (!value) return "--:--";
  const [hour, minute] = value.split(":");
  const safeMinute = minute?.split(".")[0];
  return `${hour?.padStart(2, "0") ?? "--"}:${safeMinute?.padStart(2, "0") ?? "--"}`;
}

function mapApiTourToUi(tour: ApiTour): Tour {
  return {
    id: tour.id,
    codigo: tour.codigo,
    responsavel: tour.responsavel_id ? `Responsável #${tour.responsavel_id}` : "Não informado",
    responsavel_id: tour.responsavel_id,
    status: normalizeStatus(tour.status),
    data: formatApiDate(tour.data_local),
    hora_inicio_prevista: formatTime(tour.hora_inicio_prevista),
    hora_fim_prevista: formatTime(tour.hora_fim_prevista),
    titulo: tour.titulo,
    robo_id: tour.robo_id,
  };
}

export default function HomeScreen() {
  const [tours, setTours] = useState(initialTours);
  const [openPopup, setOpenPopup] = useState(false);
  const [selectedDate, setSelectedDate] = useState(new Date());
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [deleteTarget, setDeleteTarget] = useState<Tour | null>(null);
  const [isDeleting, setIsDeleting] = useState(false);

  function formatDate(date: Date) {
    return date.toLocaleDateString("pt-BR");
  }

  useEffect(() => {
    async function fetchTours() {
      setIsLoading(true);
      setError(null);

      try {
        const response = await tourService.list();
        const mapped = response.data.map(mapApiTourToUi);
        setTours(mapped);
      } catch (err) {
        console.error("Erro ao buscar tours", err);
        setError("Não foi possível carregar os tours agora.");
      } finally {
        setIsLoading(false);
      }
    }

    fetchTours();
  }, []);

  function addTour(newTour: Tour) {
    setTours(prev => [...prev, newTour]);
  }

  function updateTour(updatedTour: Tour) {
    setTours(prev =>
      prev.map(tour => {
        const same = updatedTour.id ? tour.id === updatedTour.id : tour.codigo === updatedTour.codigo;
        return same ? { ...tour, ...updatedTour } : tour;
      })
    );
  }

  const filteredTours = tours.filter(
    tour => tour.data === formatDate(selectedDate)
  );

  async function confirmDelete() {
    if (!deleteTarget) return;
    setIsDeleting(true);
    try {
      if (deleteTarget.id) {
        await tourService.remove(deleteTarget.id);
      }
      setTours(prev => prev.filter(t => (deleteTarget.id ? t.id !== deleteTarget.id : t.codigo !== deleteTarget.codigo)));
    } catch (err) {
      console.error("Erro ao deletar tour", err);
      setError("Não foi possível deletar o tour.");
    } finally {
      setIsDeleting(false);
      setDeleteTarget(null);
    }
  }

  return (
    <View style={styles.container}>
      <Header />

      <DateSelector onDateChange={setSelectedDate} />

      {openPopup && (
        <AddTourPopup
          onClose={() => setOpenPopup(false)}
          addTour={addTour}
        />
      )}

      <ScrollView
        style={styles.cards}
        contentContainerStyle={{ alignItems: "center", gap: 24, marginTop: 50 }}
        showsVerticalScrollIndicator={false}
      >
        {isLoading && (
          <Text style={{ color: "white" }}>Carregando tours...</Text>
        )}

        {error && !isLoading && (
          <Text style={{ color: "white" }}>{error}</Text>
        )}

        {!isLoading && !filteredTours.length && !error && (
          <Text style={{ color: "white" }}>Nenhum tour para esta data.</Text>
        )}

        {!isLoading && filteredTours.map((tour) => (
          <CardTour
            key={tour.codigo}
            {...tour}
            onUpdateTour={updateTour}
            onDelete={() => setDeleteTarget(tour)}
          />
        ))}
      </ScrollView>

      <AddTourIcon onOpen={() => setOpenPopup(true)} />
      <ConfirmDeleteModal
        visible={!!deleteTarget}
        tourLabel={deleteTarget?.codigo}
        isDeleting={isDeleting}
        onCancel={() => setDeleteTarget(null)}
        onConfirm={confirmDelete}
      />
      <Navbar />
    </View>
  );
}


const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#201A2C",
    paddingTop: 64,
    justifyContent: "center",
    alignItems: "center",
  },

  cards: {
    width: "100%",
    flexGrow: 1,
    maxHeight: "65%"
  },
});
