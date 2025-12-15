import { View, Image, StyleSheet } from "react-native";


export function Header() {
  return (
    <View style={styles.header}>
        <Image source={require("../assets/images/logo_inteli.png")}/>
    </View>
  )
}

const styles = StyleSheet.create({
    header: {
        position: "absolute",
        top: 40,
        left: 133,
    }
})
