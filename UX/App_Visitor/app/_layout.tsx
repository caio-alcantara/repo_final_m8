// app/_layout.tsx
import { Stack } from "expo-router";
import { useEffect } from "react";
import { Keyboard, Platform } from "react-native";
import { StatusBar } from "expo-status-bar";
import * as NavigationBar from "expo-navigation-bar";
import * as SystemUI from "expo-system-ui";
import { TourProvider } from "@/context/TourContext";

const BG = "#1E1730";

export default function RootLayout() {
  useEffect(() => {
    const applyAndroidSystemBars = async () => {
      if (Platform.OS !== "android") return;
      await NavigationBar.setButtonStyleAsync("light");
    };

    applyAndroidSystemBars();

    // Alguns Androids “recompõem” a navbar ao abrir/fechar teclado
    const subShow = Keyboard.addListener("keyboardDidShow", applyAndroidSystemBars);
    const subHide = Keyboard.addListener("keyboardDidHide", applyAndroidSystemBars);

    return () => {
      subShow.remove();
      subHide.remove();
    };
  }, []);

  return (
    <>
      <StatusBar hidden />
      <TourProvider>
        <Stack
          screenOptions={{
            headerShown: false,
            animation: "none",
            contentStyle: { backgroundColor: BG },
          }}
        >
          <Stack.Screen name="(tabs)" options={{ headerShown: false }} />
        </Stack>
      </TourProvider>
    </>
  );
}
