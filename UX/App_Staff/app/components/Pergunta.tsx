import { StyleSheet, View, Text, Pressable } from 'react-native'
import { useState } from 'react'
import MaterialCommunityIcons from '@expo/vector-icons/MaterialCommunityIcons'
import Feather from '@expo/vector-icons/Feather'

type Props = {
    pergunta: string,
    local: string,
    resposta: string
}

export function Pergunta({ pergunta, local, resposta }: Props) {
    const [showPergunta, setShowPergunta] = useState(false);
    return (
        <Pressable style={styles.container} onPress={() => setShowPergunta(!showPergunta)}>
            <View style={styles.pergunta}>
                <MaterialCommunityIcons name="message-text" size={24} color="#FFF" />
                <Text style={{ flex: 1, color: "#fff", flexWrap: "wrap" }}>{pergunta}</Text>
                <Text style={{ fontSize: 12, backgroundColor: "#5C3DA9", padding: 4, justifyContent: "center", color: "#FFF", borderRadius: 4 }}>
                    <Feather name="map-pin" size={12} color="#FFF" />
                    {local}
                </Text>
            </View>

            {showPergunta && (
                <>
                    <View style={styles.divider} />

                    <Text style={{ color: "#FFF" }}>
                        {resposta}
                    </Text>
                </>

            )}
        </Pressable>
    )
}

const styles = StyleSheet.create({
    container: {
        backgroundColor: "rgba(217, 217, 217, 0.20)",
        paddingVertical: 16,
        paddingHorizontal: 22,
        borderRadius: 20,
        width: "92%",
    },

    pergunta: {
        flexDirection: "row",
        gap: 12,
        justifyContent: "center",
        alignItems: "center"
    },

    divider: {
        height: StyleSheet.hairlineWidth,
        backgroundColor: '#CCC',
        width: '100%',
        marginVertical: 12,
    },
})
