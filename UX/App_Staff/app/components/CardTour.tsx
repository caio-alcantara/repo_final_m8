import { StyleSheet, View, Text, Pressable, Alert } from 'react-native'
import React, { useState } from "react";
import Feather from '@expo/vector-icons/Feather'
import MaterialIcons from '@expo/vector-icons/MaterialIcons'
import PopUpInfosGerais from "@/components/PopUpInfosGerais";
import PopUpCodigo from "@/components/PopUpCodigo";
import { EditTourPopup } from "@/components/EditTourPopup";
import type { Tour } from "@/app/(tabs)/index";

type CardTourProps = {
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
    onUpdateTour?: (updatedTour: Tour) => void; // Callback para atualizar o tour
    onDelete?: () => void;
};

export default function CardTour({
    id,
    codigo,
    responsavel,
    responsavel_id,
    status,
    data,
    hora_inicio_prevista,
    hora_fim_prevista,
    titulo,
    robo_id,
    onUpdateTour,
    onDelete,
}: CardTourProps) {
    const getStyle = () => {
        switch (status) {
            case "scheduled":
                return styles.scheduled;
            case "in_progress":
                return styles.in_progress;
            case "paused":
                return styles.paused;
            case "finished":
                return styles.finished
            case "cancelled":
                return styles.canceled
        }
    };

    const [mostrarPopUpInfo, setMostrarPopUpInfo] = useState(false);
    const [mostrarPopUpCodigo, setMostrarPopUpCodigo] = useState(false);
    const [mostrarEditPopup, setMostrarEditPopup] = useState(false);

    function abrirPopUp() {
        setMostrarPopUpInfo(true);
    }

    function fecharPopUp() {
        setMostrarPopUpInfo(false);
    }

    const handleStartTour = () => {
        setMostrarPopUpInfo(false);
        setMostrarPopUpCodigo(true);
    };

    const fecharPopUpCodigo = () => {
        setMostrarPopUpCodigo(false);
    }

    const abrirEditPopup = () => {
        setMostrarEditPopup(true);
    };

    const fecharEditPopup = () => {
        setMostrarEditPopup(false);
    };

    const handleUpdateTour = (updatedTour: Tour) => {
        if (onUpdateTour) {
            onUpdateTour(updatedTour);
        }
        fecharEditPopup();
    };

    const tourData: Tour = {
        id,
        codigo,
        responsavel,
        responsavel_id,
        status,
        data,
        hora_inicio_prevista,
        hora_fim_prevista,
        titulo,
        robo_id,
    };

    return (
        <>
            {mostrarEditPopup && (
                <EditTourPopup
                    tour={tourData}
                    onClose={fecharEditPopup}
                    updateTour={handleUpdateTour}
                />
            )}

            <View style={styles.card_container}>
                <PopUpInfosGerais
                    visible={mostrarPopUpInfo}
                    onClose={fecharPopUp}
                    onConfirm={handleStartTour}
                />

                <PopUpCodigo
                    visible={mostrarPopUpCodigo}
                    onClose={fecharPopUpCodigo}
                    onConfirm={() => { }}
                    codigo={codigo}
                />

                <View style={styles.topo}>
                    <Text style={styles.text}>Tour #{codigo}</Text>
                    <View style={styles.botoes}>
                        <Pressable onPress={abrirEditPopup}>
                            <Feather name="edit-2" size={20} color="#9747FF" />
                        </Pressable>

                        <Pressable onPress={onDelete}>
                            <MaterialIcons name="close" size={20} color="black" />
                        </Pressable>
                    </View>
                </View>

                <View style={styles.infos}>
                    <View style={{ width: "50%" }}>
                        <Text style={styles.label}>Staff</Text>
                        <Text>{responsavel}</Text>
                    </View>

                    <View style={{ width: "50%" }}>
                        <Text style={styles.label}>Horário</Text>
                        <View style={{ display: "flex", flexDirection: "row", alignItems: "center", gap: 2 }}>
                            <Text>{hora_inicio_prevista}</Text>
                            <MaterialIcons name="arrow-right-alt" size={14} color="black" />
                            <Text>{hora_fim_prevista}</Text>
                        </View>
                    </View>
                </View>

                <View style={styles.infos}>
                    <View>
                        <Text style={styles.label}>Status</Text>
                        <Text style={[{ padding: 4, borderRadius: 2 }, getStyle()]}>
                            {status === "scheduled" ? "A começar" :
                                status === "in_progress" ? "Em progresso" :
                                    status === "paused" ? "Tour em pausa" :
                                        status === "cancelled" ? "Cancelado" :
                                            "Finalizado"}
                        </Text>
                    </View>

                    <View>
                        {status === "finished" || status === "cancelled" ? (
                            <></>) : (
                            <Pressable style={styles.button} onPress={abrirPopUp}>
                                <Text style={{ color: "#FFF", textAlign: "center" }}>
                                    {status === "scheduled" ? "Começar tour" :
                                        status === "in_progress" ? "Finalizar tour" :
                                            "Retomar tour"}
                                </Text>
                            </Pressable>
                        )}
                    </View>
                </View>
            </View>
        </>
    );
}

const styles = StyleSheet.create({
    card_container: {
        borderRadius: 20,
        padding: 16,
        backgroundColor: "rgba(255, 255, 255, 0.95)",
        width: "90%",
        gap: 16
    },

    text: {
        fontSize: 16,
        fontWeight: "700",
        color: "#404040"
    },

    topo: {
        display: "flex",
        flexDirection: "row",
        justifyContent: "space-between"
    },

    botoes: {
        display: "flex",
        flexDirection: "row",
        alignItems: "center",
        justifyContent: "space-around",
        width: "20%",
    },

    infos: {
        display: "flex",
        flexDirection: "row",
        marginTop: 16,
        alignItems: "center",
        justifyContent: "space-between"
    },

    label: {
        color: "rgba(19, 26, 41, 0.48)"
    },

    button: {
        borderRadius: 20,
        backgroundColor: "#855EDE",
        padding: 8,
        width: 120,
    },

    // Estilização STATUS
    scheduled: {
        backgroundColor: "rgba(151, 151, 151, 0.60)",
        color: "#404040",
    },
    in_progress: {
        backgroundColor: "rgba(255, 187, 0, 0.25)",
        color: "#FFBB00"
    },
    paused: {
        backgroundColor: "rgba(77, 166, 255, 0.25)",
        color: "#4DA6FF"
    },
    finished: {
        backgroundColor: "rgba(67, 215, 135, 0.60)",
        color: "#246E46"
    },
    canceled: {
        backgroundColor: "rgba(99, 90, 118, 0.30)",
        color: "#635A76"
    }
})
