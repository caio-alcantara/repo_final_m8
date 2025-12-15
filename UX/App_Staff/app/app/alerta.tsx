import { Animated, View, StyleSheet, Platform, UIManager, Text, TouchableOpacity, Image } from "react-native"
import { Navbar } from "@/components/navbar";
import { Header } from "@/components/header";
const AlertButton = require("@/assets/images/alert-button.png");
import { useState } from 'react'

if (Platform.OS === "android" && UIManager.setLayoutAnimationEnabledExperimental) {
    UIManager.setLayoutAnimationEnabledExperimental(true);
}

export default function MapScreen() {
    const [scaleAnim] = useState(new Animated.Value(1));

    const handleAlertPress = () => {
        // Animação de pressão
        Animated.sequence([
            Animated.timing(scaleAnim, {
                toValue: 0.9,
                duration: 100,
                useNativeDriver: true,
            }),
            Animated.timing(scaleAnim, {
                toValue: 1,
                duration: 100,
                useNativeDriver: true,
            }),
        ]).start();

        // Log da emergência acionada
        console.log("🚨 EMERGÊNCIA ACIONADA - Tour interrompido, equipe Inteli notificada");
    };

    return (
        <View style={styles.container}>
            <Header />

            <View style={styles.content}>
                {/* Círculo roxo claro com botão vermelho */}
                <TouchableOpacity
                    activeOpacity={0.8}
                    onPress={handleAlertPress}
                >
                    <Animated.View
                        style={[
                            styles.buttonCircle,
                            { transform: [{ scale: scaleAnim }] }
                        ]}
                    >
                        <Image source={AlertButton} style={styles.alertButton} resizeMode="contain" />
                    </Animated.View>
                </TouchableOpacity>

                {/* Título */}
                <Text style={styles.title}>DESEJA SOLICITAR A EMERGÊNCIA?</Text>

                {/* Texto explicativo */}
                <Text style={styles.description}>
                    Ao clicar neste ícone, o tour será interrompido e a equipe Inteli será acionada
                </Text>
            </View>
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
        paddingHorizontal: 40,
        paddingBottom: 60,
    },
    buttonCircle: {
        width: 280,
        height: 280,
        borderRadius: 140,
        backgroundColor: "#6440C4",
        alignItems: "center",
        justifyContent: "center",
        marginBottom: 40,
        shadowColor: "#000",
        shadowOffset: { width: 0, height: 8 },
        shadowOpacity: 0.4,
        shadowRadius: 12,
        elevation: 10,
    },
    alertButton: {
        width: 180,
        height: 180,
    },
    title: {
        fontSize: 24,
        fontWeight: "bold",
        color: "#FFFFFF",
        textAlign: "center",
        marginBottom: 20,
        letterSpacing: 1,
    },
    description: {
        fontSize: 24,
        color: "#FFFFFF",
        textAlign: "center",
        lineHeight: 28,
        opacity: 0.95,
        fontWeight: "500",
        paddingHorizontal: 20,
    },
});
