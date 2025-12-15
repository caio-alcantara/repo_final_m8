import { View, Text, StyleSheet } from 'react-native'

type CheckpointProps = {
    id: number,
    status: string,
    label: string
}

export default function Checkpoint({ id, status, label }: CheckpointProps) {
    const getColor = () => {
        switch (status) {
            case "done":
                return styles.staus_done
            case "in_progress":
                return styles.status_in_progress
            case "not_started":
                return styles.status_not_started
        }
    }

    const align = (id: number) => {
        return id % 2 == 0 ? styles.align_direct : styles.align_reverse;
    }

    return (
        <View style={[{alignItems: 'center'}, align(id)]}>
            <View style={[{ width: 12, height: 12, borderRadius: 50, margin: 5 }, getColor()]} />
            <Text style={{color: 'white', paddingVertical: 2, paddingHorizontal: 4, backgroundColor: "#5C3DA9", borderRadius: 2, textTransform: "capitalize", textAlign: 'center'}}>{label}</Text>
        </View>
    )
}

const styles = StyleSheet.create({
    staus_done: {
        backgroundColor: "#06ED02"
    },
    status_in_progress: {
        backgroundColor: "#FFBB00"
    },
    status_not_started: {
        backgroundColor: "#979797"
    },

    align_reverse: {
        flexDirection: "column-reverse"
    },
    align_direct: {
        flexDirection: "column"
    }
})
