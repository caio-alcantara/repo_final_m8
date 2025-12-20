import { StyleSheet, Pressable, View } from "react-native"
import { AntDesign } from '@expo/vector-icons'


type Props = {
    onOpen: () => void;
};

export function AlertButton({ onOpen }: Props) {

    return (
        <View style={styles.container_add_icon}>
            <Pressable
                style={styles.add_tour_button}
                onPress={onOpen}
            >
                <AntDesign name="alert" size={30} color="#FFF" />
            </Pressable>
        </View>
    );
}

const styles = StyleSheet.create({
  container_add_icon: {
    width: "95%",
    position: "absolute",
    bottom: 130,
    right: 10,
    flexDirection: "row",
    justifyContent: "flex-end",
  },
  add_tour_button: {
    backgroundColor: "#855EDE",
    width: 72,
    height: 72,
    borderRadius: 36,
    alignItems: "center",
    justifyContent: "center",
  }
})
