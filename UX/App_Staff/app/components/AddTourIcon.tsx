import { StyleSheet, Pressable, View } from "react-native"
import Feather from '@expo/vector-icons/Feather'


type Props = {
  onOpen: () => void;
};

export function AddTourIcon({ onOpen }: Props) {

  return (
    <View style={styles.container_add_icon}>
      <Pressable
        style={styles.add_tour_button}
        onPress={onOpen}
      >
        <Feather name="plus" size={24} color="white" />
      </Pressable>
    </View>
  );
}

const styles = StyleSheet.create({
  container_add_icon: {
    width: "95%",
    position: "absolute",
    bottom: 100,
    right: 10,
    flexDirection: "row",
    justifyContent: "flex-end",
  },
  add_tour_button: {
    backgroundColor: "#855EDE",
    width: 64,
    height: 64,
    borderRadius: 50,
    alignItems: "center",
    justifyContent: "center",
  }
})
