import { useEffect, useState } from "react";
import { StyleSheet, View, Text, Pressable, ScrollView, Alert } from "react-native";
import MaterialIcons from "@expo/vector-icons/MaterialIcons";
import type { Tour } from "@/app/(tabs)/index";
import { FormInput } from "./FormInput";
import { DatePickerField } from "./DatePickerField";
import { TimePickerField } from "./TimePickerField";
import { StatePickerField } from "./StatePickerField";
import { CompanionSection } from "./CompanionSection";
import { tourService, visitanteService, tourVisitanteService, type Usuario } from "@/services/api";

type Props = {
  onClose: () => void;
  updateTour: (tour: Tour) => void;
  tour: Tour;
};

// Mock de responsáveis enquanto não há rota de usuários
const mockUsuarios: Usuario[] = [
  { id: 1, nome: "João Silva", email: "joao@example.com" },
  { id: 2, nome: "Maria Souza", email: "maria@example.com" },
];

export function EditTourPopup({ onClose, updateTour, tour }: Props) {
  const [form, setForm] = useState({
    roboId: "",
    titulo: tour?.titulo ?? "",
    data: new Date(),
    horaInicioPrevista: tour?.hora_inicio_prevista ?? "",
    horaFimPrevista: tour?.hora_fim_prevista ?? "",
    status: (tour?.status as Tour["status"]) ?? "scheduled",
    nomeVisitante: "",
    emailVisitante: "",
    perfilvisitante: "",
    estado: "",
    cpf: "",
    telefone: "",
    cidade: "",
    acompanhante: false,
    nomeAcompanhante: "",
    cpfAcompanhante: "",
  });

  const [responsavelSelecionado, setResponsavelSelecionado] = useState<{ id: number; nome: string | null } | null>(null);
  const [usuarios, setUsuarios] = useState<Usuario[]>([]);
  const [showResponsavelList, setShowResponsavelList] = useState(false);
  const [loadingUsuarios, setLoadingUsuarios] = useState(false);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isLoadingData, setIsLoadingData] = useState(false);
  const [initialForm, setInitialForm] = useState<typeof form | null>(null);
  const [hasChanges, setHasChanges] = useState(false);
  const [visitanteId, setVisitanteId] = useState<number | null>(null);

  function parseDate(value: string | Date | null | undefined) {
    if (!value) return new Date();
    if (value instanceof Date) return value;
    // suporta formatos "dd/mm/yyyy" e "yyyy-mm-dd"
    if (value.includes("/")) {
      const [day, month, year] = value.split("/");
      return new Date(parseInt(year, 10), parseInt(month, 10) - 1, parseInt(day, 10));
    }
    const [year, month, day] = value.split("-"); // "2025-11-17"
    return new Date(parseInt(year, 10), parseInt(month, 10) - 1, parseInt(day, 10));
  }

  function formatHora(value: string | null) {
    if (!value) return "";
    const [h, m] = value.split(":");
    return `${h?.padStart(2, "0") ?? ""}:${m?.split(".")[0]?.padStart(2, "0") ?? ""}`;
  }

  // Preencher o formulário com os dados reais do tour e visitante
  useEffect(() => {
    async function hydrateData() {
      if (!tour?.id) {
        // Sem id, usamos os dados que já vieram no card
        const baseForm = {
          roboId: tour?.robo_id ? String(tour.robo_id) : "",
          titulo: tour?.titulo ?? "",
          data: parseDate(tour?.data),
          horaInicioPrevista: tour?.hora_inicio_prevista ?? "",
          horaFimPrevista: tour?.hora_fim_prevista ?? "",
          status: tour?.status ?? "scheduled",
          nomeVisitante: "",
          emailVisitante: "",
          perfilvisitante: "",
          estado: "",
          cpf: "",
          telefone: "",
          cidade: "",
          acompanhante: false,
          nomeAcompanhante: "",
          cpfAcompanhante: "",
        };
        setForm(baseForm);
        setInitialForm(baseForm);
        return;
      }

      setIsLoadingData(true);
      try {
        const [tourResp, visitRelResp] = await Promise.all([
          tourService.getById(tour.id),
          tourVisitanteService.listByTour(tour.id),
        ]);

        const visitanteId = visitRelResp.data[0]?.visitante_id;
        const visitanteResp = visitanteId ? await visitanteService.getById(visitanteId) : null;
        if (visitanteId) setVisitanteId(visitanteId);

        const baseForm = {
          roboId: tourResp.data.robo_id?.toString() ?? "",
          titulo: tourResp.data.titulo ?? "",
          data: parseDate(tourResp.data.data_local ?? tour.data),
          horaInicioPrevista: formatHora(tourResp.data.hora_inicio_prevista) || "",
          horaFimPrevista: formatHora(tourResp.data.hora_fim_prevista) || "",
          status: tourResp.data.status ?? "scheduled",
          nomeVisitante: visitanteResp?.data.nome ?? "",
          emailVisitante: visitanteResp?.data.email ?? "",
          perfilvisitante: "",
          estado: "",
          cpf: "",
          telefone: visitanteResp?.data.telefone ?? "",
          cidade: "",
          acompanhante: false,
          nomeAcompanhante: "",
          cpfAcompanhante: "",
        };

        setForm(baseForm);
        setInitialForm(baseForm);

        const respId = tourResp.data.responsavel_id ?? tour.responsavel_id;
        if (respId) {
          setResponsavelSelecionado({ id: respId, nome: `Responsável #${respId}` });
        } else {
          const mockResp = mockUsuarios.find(u => u.nome === tour.responsavel);
          if (mockResp) {
            setResponsavelSelecionado({ id: mockResp.id as number, nome: mockResp.nome });
          }
        }
      } catch (error) {
        console.error("Erro ao carregar dados do tour para edição", error);
        Alert.alert("Erro", "Não foi possível carregar os dados do tour.");
      } finally {
        setIsLoadingData(false);
      }
    }

    hydrateData();
  }, [tour]);

  // Detecta alterações para habilitar/desabilitar o botão de salvar
  useEffect(() => {
    if (!initialForm) return;
    const normalize = (value: any) => {
      if (value instanceof Date) return value.toISOString();
      return value ?? "";
    };

    const changed = Object.keys(initialForm).some(key => normalize((form as any)[key]) !== normalize((initialForm as any)[key]));
    setHasChanges(changed);
  }, [form, initialForm]);

  function updateField(field: string, value: any) {
    setForm((prev) => ({ ...prev, [field]: value }));
  }

  useEffect(() => {
    setLoadingUsuarios(true);
    setUsuarios(mockUsuarios);
    setLoadingUsuarios(false);
  }, []);

  function toIsoDate(date: Date) {
    return date.toISOString().split("T")[0];
  }

  function timeWithSeconds(value: string) {
    if (!value) return "";
    const parts = value.split(":");
    if (parts.length === 2) return `${parts[0].padStart(2, "0")}:${parts[1].padStart(2, "0")}:00`;
    if (parts.length >= 3)
      return `${parts[0].padStart(2, "0")}:${parts[1].padStart(2, "0")}:${parts[2].slice(0, 2).padStart(2, "0")}`;
    return value;
  }

  function normalizeStatusInput(value: string) {
    const allowed: Tour["status"][] = ["scheduled", "in_progress", "paused", "finished", "cancelled"];
    return allowed.includes(value as Tour["status"]) ? (value as Tour["status"]) : "scheduled";
  }

  async function handleSubmit() {
    if (isSubmitting) return;

    if (!tour?.id) {
      Alert.alert("Erro", "Não foi possível identificar o tour para editar.");
      return;
    }

    if (!form.nomeVisitante || !form.emailVisitante || !form.telefone) {
      Alert.alert("Campos obrigatórios", "Preencha nome, email e telefone do visitante.");
      return;
    }

    if (!form.horaInicioPrevista || !form.horaFimPrevista) {
      Alert.alert("Campos obrigatórios", "Preencha os horários do tour.");
      return;
    }

    if (!responsavelSelecionado && !tour.responsavel_id) {
      Alert.alert("Responsável", "Selecione um responsável para o tour.");
      return;
    }

    setIsSubmitting(true);

    try {
      if (visitanteId) {
        await visitanteService.update(visitanteId, {
          nome: form.nomeVisitante,
          email: form.emailVisitante,
          telefone: form.telefone,
        });
      }

      const payload = {
        codigo: tour.codigo,
        data_local: toIsoDate(form.data),
        hora_inicio_prevista: timeWithSeconds(form.horaInicioPrevista),
        hora_fim_prevista: timeWithSeconds(form.horaFimPrevista),
        responsavel_id: responsavelSelecionado?.id ?? tour.responsavel_id ?? null,
        robo_id: form.roboId ? Number(form.roboId) : tour.robo_id ?? 1,
        status: normalizeStatusInput(form.status),
        titulo: form.titulo || tour.titulo || "Tour",
        inicio_real: null,
        fim_real: null,
      };

      const resp = await tourService.update(tour.id, payload as any);
      const apiTour = resp.data;

      const updatedTour: Tour = {
        id: apiTour.id ?? tour.id,
        codigo: apiTour.codigo ?? tour.codigo,
        responsavel: responsavelSelecionado?.nome ?? `Responsável #${apiTour.responsavel_id ?? tour.responsavel_id ?? ""}`,
        responsavel_id: apiTour.responsavel_id ?? tour.responsavel_id,
        status: normalizeStatusInput(apiTour.status),
        data: form.data.toLocaleDateString("pt-BR"),
        hora_inicio_prevista: formatHora(apiTour.hora_inicio_prevista) || form.horaInicioPrevista,
        hora_fim_prevista: formatHora(apiTour.hora_fim_prevista) || form.horaFimPrevista,
        titulo: apiTour.titulo ?? tour.titulo,
        robo_id: apiTour.robo_id ?? tour.robo_id,
      };

      updateTour(updatedTour);
      setInitialForm(form);
      setHasChanges(false);
      onClose();
    } catch (error) {
      console.error(error);
      Alert.alert("Erro", "Não foi possível atualizar o tour.");
    } finally {
      setIsSubmitting(false);
    }
  }

  const handleHoraInicioChange = (hora: string) => {
    updateField("horaInicioPrevista", hora);

    const [horas, minutos] = hora.split(":");
    const inicio = new Date();
    inicio.setHours(parseInt(horas, 10), parseInt(minutos, 10));

    const fim = new Date(inicio.getTime());
    fim.setHours(fim.getHours() + 1);

    const horaFim = fim.toLocaleTimeString("pt-BR", {
      hour: "2-digit",
      minute: "2-digit",
      hour12: false,
    });

    updateField("horaFimPrevista", horaFim);
  };

  return (
    <View style={styles.overlay}>
      <View style={styles.edit_tour_popup}>
        <View style={styles.topo}>
          <Text style={styles.title}>Editar tour</Text>
          <Pressable onPress={onClose}>
            <MaterialIcons name="close" size={20} color="black" />
          </Pressable>
        </View>

        <ScrollView contentContainerStyle={{ paddingBottom: 20 }}>
          {/* Informações gerais */}
          <View style={styles.bloco_input}>
            <View style={[styles.input_section, { width: "95%" }]}>
              <Text style={styles.label}>Staff responsável</Text>
              <Pressable onPress={() => setShowResponsavelList((prev) => !prev)} style={styles.selectBox}>
                <Text style={{ fontSize: 14 }}>
                  {responsavelSelecionado?.nome || (loadingUsuarios ? "Carregando..." : "Selecione o responsável")}
                </Text>
              </Pressable>
              {showResponsavelList && (
                <View style={styles.dropdown}>
                  {usuarios.map((user) => (
                    <Pressable
                      key={user.id}
                      style={styles.dropdownItem}
                      onPress={() => {
                        setResponsavelSelecionado({ id: user.id as number, nome: user.nome });
                        setShowResponsavelList(false);
                      }}
                    >
                      <Text>{user.nome || `Usuário #${user.id}`}</Text>
                    </Pressable>
                  ))}
                </View>
              )}
            </View>

            <FormInput
              label="Robô ID"
              value={form.roboId}
              onChangeText={(text) => updateField("roboId", text)}
              width="48%"
              keyboardType="numeric"
            />

            <DatePickerField label="Data" value={form.data} onChange={(date) => updateField("data", date)} width="48%" />

            <TimePickerField
              label="Horário inicial"
              value={form.horaInicioPrevista}
              onChange={handleHoraInicioChange}
              width="48%"
              testID="timePickerInicio"
            />

            <TimePickerField
              label="Horário final"
              value={form.horaFimPrevista}
              onChange={(hora) => updateField("horaFimPrevista", hora)}
              width="48%"
              testID="timePickerFim"
            />

            <FormInput label="Título" value={form.titulo} onChangeText={(text) => updateField("titulo", text)} width="95%" />
          </View>

          {/* Visitantes */}
          <Text style={[styles.title, { paddingBottom: 8 }]}>Visitantes</Text>

          <View style={styles.bloco_input}>
            <FormInput label="Nome" value={form.nomeVisitante} onChangeText={(text) => updateField("nomeVisitante", text)} width="95%" />

            <FormInput
              label="E-mail"
              value={form.emailVisitante}
              onChangeText={(text) => updateField("emailVisitante", text)}
              width="95%"
            />

            <FormInput
              label="Perfil"
              value={form.perfilvisitante}
              onChangeText={(text) => updateField("perfilvisitante", text)}
              width="95%"
            />

            <FormInput label="CPF" value={form.cpf} onChangeText={(text) => updateField("cpf", text)} width="48%" />

            <FormInput label="Telefone" value={form.telefone} onChangeText={(text) => updateField("telefone", text)} width="48%" />

            <StatePickerField label="Estado" value={form.estado} onChange={(value) => updateField("estado", value)} width="48%" />

            <FormInput label="Cidade" value={form.cidade} onChangeText={(text) => updateField("cidade", text)} width="48%" />
          </View>

          <CompanionSection
            hasCompanion={form.acompanhante}
            companionName={form.nomeAcompanhante}
            companionCpf={form.cpfAcompanhante}
            onToggleCompanion={() => updateField("acompanhante", !form.acompanhante)}
            onChangeCompanionName={(text) => updateField("nomeAcompanhante", text)}
            onChangeCompanionCpf={(text) => updateField("cpfAcompanhante", text)}
          />

          {/* Botão de Editar */}
          <View style={styles.buttonContainer}>
            <Pressable 
              style={[
                styles.editButton,
                (!hasChanges || isSubmitting || isLoadingData) && styles.editButtonDisabled
              ]} 
              onPress={handleSubmit}
              disabled={!hasChanges || isSubmitting || isLoadingData}
            >
              <Text style={styles.editButtonText}>
                {isSubmitting ? "Salvando..." : isLoadingData ? "Carregando..." : "Editar tour"}
              </Text>
            </Pressable>
          </View>
        </ScrollView>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  overlay: {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    justifyContent: "flex-start",
    alignItems: "center",
    backgroundColor: "rgba(0, 0, 0, 0.3)",
    zIndex: 1000,
  },
  edit_tour_popup: {
    width: "90%",
    borderRadius: 20,
    backgroundColor: "white",
    marginTop: 60,
    elevation: 10,
    padding: 16,
    zIndex: 1001,
  },
  title: {
    fontSize: 16,
    fontWeight: "700",
    color: "#404040",
  },
  topo: {
    display: "flex",
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
  },
  bloco_input: {
    flexDirection: "row",
    flexWrap: "wrap",
    justifyContent: "space-between",
    gap: 8,
    marginTop: 25,
  },
  input_section: {
    display: "flex",
    flexDirection: "column",
    justifyContent: "space-between",
    borderWidth: 1,
    borderColor: "#E5E5E5",
    borderRadius: 12,
    paddingHorizontal: 10,
    paddingVertical: 8,
    marginBottom: 12,
  },
  label: {
    color: "rgba(19, 26, 41, 0.48)",
    fontSize: 12,
  },
  selectBox: {
    borderWidth: 1,
    borderColor: "#E5E5E5",
    borderRadius: 12,
    paddingHorizontal: 10,
    paddingVertical: 12,
    marginTop: 8,
    backgroundColor: "#F8F8F8",
  },
  dropdown: {
    borderWidth: 1,
    borderColor: "#E5E5E5",
    borderRadius: 12,
    marginTop: 8,
    backgroundColor: "#FFF",
    maxHeight: 160,
  },
  dropdownItem: {
    paddingVertical: 10,
    paddingHorizontal: 12,
    borderBottomWidth: 1,
    borderBottomColor: "#EEE",
  },
  buttonContainer: {
    marginTop: 20,
    alignItems: "center",
  },
  editButton: {
    backgroundColor: "#9747FF",
    borderRadius: 12,
    paddingVertical: 14,
    paddingHorizontal: 40,
    minWidth: 200,
    alignItems: "center",
  },
  editButtonDisabled: {
    backgroundColor: "#D0B3FF",
  },
  editButtonText: {
    color: "white",
    fontSize: 16,
    fontWeight: "600",
  },
});
