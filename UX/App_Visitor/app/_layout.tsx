// app/_layout.tsx
import { Stack } from "expo-router";
import { useEffect } from "react";
import { Platform } from "react-native";
import { StatusBar } from "expo-status-bar";
import * as NavigationBar from "expo-navigation-bar";
import { TourProvider } from "@/context/TourContext";

export default function RootLayout() {
  useEffect(() => {
    if (Platform.OS === "android") {
      // esconde a barra do sistema
      NavigationBar.setVisibilityAsync("hidden");
      // usuário só vê se fizer gesto; ela sobrepõe o app
      NavigationBar.setBehaviorAsync("overlay-swipe");
    }
  }, []);

  return (
    <>
      {/* esconde a status bar lá de cima também */}
      <StatusBar hidden />
      <TourProvider>

      <Stack
        screenOptions={{
          headerShown: false,
        }}
      />
      </TourProvider>
    </>
  );
}
