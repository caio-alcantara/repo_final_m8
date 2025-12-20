import { View, StyleSheet, Text } from "react-native";
import { Header } from "@/components/header";
import { Navbar } from "@/components/navbar";
import { MenuItem } from "@/components/MenuItem";
import { useRouter } from "expo-router";

export default function MenuScreen() {
  const router = useRouter();

  return (
    <View style={styles.container}>
      <Header />

      <View style={styles.content}>
        <Text style={styles.title}>Menu</Text>

        <View style={styles.menuList}>
          <MenuItem label="Configurações" icon="settings" showChevron />
          <MenuItem
            label="Sair"
            icon="log-out"
            onPress={() => router.replace("/login")}
          />
        </View>
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
    alignItems: "center",
  },
  content: {
    flex: 1,
    width: "86%",
    alignItems: "center",
    marginTop: 60,
  },
  title: {
    color: "#F3F4F6",
    fontSize: 24,
    fontWeight: "800",
    marginTop: 48,
    marginBottom: 32,
  },
  menuList: {
    width: "100%",
    gap: 8,
  },
});
