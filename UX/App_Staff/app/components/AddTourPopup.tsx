import { useEffect, useState } from "react";
import { StyleSheet, View, Text, Pressable, ScrollView, Alert, KeyboardAvoidingView, Platform } from "react-native";
import { Picker } from "@react-native-picker/picker";
import Feather from "@expo/vector-icons/Feather";
import MaterialIcons from "@expo/vector-icons/MaterialIcons";
import type { Tour } from "@/app/(tabs)/index";
import { FormInput } from "./FormInput";
import { DatePickerField } from "./DatePickerField";
import { TimePickerField } from "./TimePickerField";
import { StatePickerField } from "./StatePickerField";
import { CompanionSection } from "./CompanionSection";
import { acompanhanteService, tourService, visitanteService, tourVisitanteService, type Usuario } from "@/services/api";

type Props = {
  onClose: () => void;
  addTour: (tour: Tour) => void;
};

// Mock de responsáveis enquanto não há rota de usuários
const mockUsuarios: Usuario[] = [
  { id: 1, nome: "João Silva", email: "joao@example.com" },
  { id: 2, nome: "Maria Souza", email: "maria@example.com" },
];

export function AddTourPopup({ onClose, addTour }: Props) {
  const [form, setForm] = useState({
    roboId: "",
    titulo: "",
    data: new Date(),
    horaInicioPrevista: "",
    horaFimPrevista: "",
    status: "scheduled",
    nomeVisitante: "",
    emailVisitante: "",
    perfilvisitante: "student",
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

  function updateField(field: string, value: any) {
    setForm((prev) => ({ ...prev, [field]: value }));
  }

  function generateCode() {
    const letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    const numbers = "0123456789";
    const all = letters + numbers;

    // Garante pelo menos uma letra e um número, depois preenche o restante e embaralha.
    const base = [
      letters[Math.floor(Math.random() * letters.length)],
      numbers[Math.floor(Math.random() * numbers.length)],
    ];
    while (base.length < 6) {
      base.push(all[Math.floor(Math.random() * all.length)]);
    }

    // Fisher–Yates simples para embaralhar
    for (let i = base.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [base[i], base[j]] = [base[j], base[i]];
    }

    return base.join("");
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

  function normalizeStatusFromApi(value: string | undefined) {
    if (value === "completed") return "finished";
    if (value === "scheduled" || value === "in_progress" || value === "cancelled") return value;
    return "scheduled";
  }

  async function handleSubmit() {
    if (isSubmitting) return;

    if (!form.nomeVisitante || !form.emailVisitante || !form.telefone) {
      Alert.alert("Campos obrigatórios", "Preencha nome, email e telefone do visitante.");
      return;
    }

    if (!form.horaInicioPrevista || !form.horaFimPrevista) {
      Alert.alert("Campos obrigatórios", "Preencha os horários do tour.");
      return;
    }

    if (!responsavelSelecionado) {
      Alert.alert("Responsável", "Selecione um responsável para o tour.");
      return;
    }

    if (form.acompanhante && (!form.nomeAcompanhante || !form.cpfAcompanhante)) {
      Alert.alert("Acompanhante", "Preencha nome e CPF do acompanhante.");
      return;
    }

    setIsSubmitting(true);
    const codigo = generateCode();

    try {
      const visitanteResp = await visitanteService.create({
        email: form.emailVisitante || null,
        nome: form.nomeVisitante || null,
        telefone: form.telefone || null,
        perfil: (form.perfilvisitante as "student" | "executive") || "student",
        estado: form.estado || null,
        cidade: form.cidade || null,
        cpf: form.cpf || null,
      });

      const visitanteId = visitanteResp.data.id;
      if (!visitanteId) throw new Error("ID do visitante não retornado");

      const tourResp = await tourService.create({
        codigo,
        data_local: toIsoDate(form.data),
        hora_inicio_prevista: timeWithSeconds(form.horaInicioPrevista),
        hora_fim_prevista: timeWithSeconds(form.horaFimPrevista),
        responsavel_id: responsavelSelecionado.id,
        robo_id: Number(form.roboId) || 1,
        status: "scheduled",
        titulo: form.titulo || "Tour",
      });

      const tourId = tourResp.data.id;
      if (!tourId) throw new Error("ID do tour não retornado");

      await tourVisitanteService.create({
        tour_id: tourId,
        visitante_id: visitanteId,
      });

      if (form.acompanhante) {
        await acompanhanteService.create({
          visitante_id: visitanteId,
          nome: form.nomeAcompanhante || null,
          cpf: form.cpfAcompanhante || null,
        });
      }

      const newTour: Tour = {
        codigo: tourResp.data.codigo ?? codigo,
        responsavel: responsavelSelecionado?.nome ?? `Responsável #${tourResp.data.responsavel_id ?? ""}`,
        status: normalizeStatusFromApi(tourResp.data.status),
        data: form.data.toLocaleDateString("pt-BR"),
        hora_inicio_prevista: tourResp.data.hora_inicio_prevista?.slice(0, 5) ?? form.horaInicioPrevista,
        hora_fim_prevista: tourResp.data.hora_fim_prevista?.slice(0, 5) ?? form.horaFimPrevista,
      };

      addTour(newTour);
      onClose();
    } catch (error) {
      console.error(error);
      Alert.alert("Erro", "Não foi possível cadastrar o tour.");
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
      <KeyboardAvoidingView
        behavior={Platform.OS === "ios" ? "padding" : "height"}
        style={styles.keyboardAvoid}
      >
        <View style={styles.add_tour_popup}>
          <View style={styles.topo}>
            <Text style={styles.title}>Cadastrar novo tour</Text>
            <Pressable onPress={onClose}>
              <MaterialIcons name="close" size={20} color="black" />
            </Pressable>
          </View>

          <ScrollView
            contentContainerStyle={styles.scrollContent}
            keyboardShouldPersistTaps="handled"
          >
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

            <View style={[styles.input_section, { width: "95%" }]}>
              <Text style={styles.label}>Perfil</Text>
              <View style={styles.pickerContainer}>
                <Picker
                  selectedValue={form.perfilvisitante}
                  onValueChange={(value) => updateField("perfilvisitante", value)}
                >
                  <Picker.Item label="Estudante" value="student" />
                  <Picker.Item label="Executivo" value="executive" />
                </Picker>
              </View>
            </View>

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
          <View style={styles.buttonContainer}>
            <Pressable
              style={[
                styles.submitButton,
                isSubmitting && styles.submitButtonDisabled,
              ]}
              onPress={handleSubmit}
              disabled={isSubmitting}
            >
              <Text style={styles.submitButtonText}>
                {isSubmitting ? "Salvando..." : "Criar tour"}
              </Text>
            </Pressable>
          </View>
          </ScrollView>
        </View>
      </KeyboardAvoidingView>
    </View>
  );
}

const styles = StyleSheet.create({
  overlay: {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    height: "100%",
    justifyContent: "flex-start",
    alignItems: "center",
    backgroundColor: "rgba(0, 0, 0, 0.3)",
    zIndex: 2000,
  },
  keyboardAvoid: {
    width: "100%",
    alignItems: "center",
  },
  add_tour_popup: {
    width: "90%",
    borderRadius: 20,
    backgroundColor: "white",
    marginTop: 60,
    elevation: 6,
    padding: 16,
    zIndex: 2001,
    maxHeight: "95%",
  },
  scrollContent: {
    paddingBottom: 20,
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
  pickerContainer: {
    marginTop: 8,
    borderWidth: 1,
    borderColor: "#E5E5E5",
    borderRadius: 12,
    backgroundColor: "#F8F8F8",
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
    marginTop: 12,
    alignItems: "center",
  },
  submitButton: {
    backgroundColor: "#9747FF",
    borderRadius: 12,
    paddingVertical: 14,
    paddingHorizontal: 32,
    minWidth: 220,
    alignItems: "center",
  },
  submitButtonDisabled: {
    opacity: 0.7,
  },
  submitButtonText: {
    color: "white",
    fontSize: 16,
    fontWeight: "600",
  },
});
