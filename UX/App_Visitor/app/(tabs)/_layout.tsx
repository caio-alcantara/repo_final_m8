// app/(tabs)/_layout.tsx
import { Tabs } from "expo-router";
import React from "react";
import { View } from "react-native";
import Navbar from "../../components/navbar"; 

export default function TabsLayout() {
  return (
    // 🔥 CORREÇÃO: Use View com flex: 1 em vez de Fragment <>
    <View style={{ flex: 1, backgroundColor: "#161221" }}>
      
      <Tabs
        screenOptions={{
          headerShown: false,
          tabBarStyle: { display: "none" }, // Esconde a barra nativa
          animation: "none",
          
          // IMPORTANTE: Se o erro persistir, remova o lazy: false temporariamente.
          // lazy: false, 
        }}
      >
        <Tabs.Screen name="home" />
        <Tabs.Screen name="mapa" />
        <Tabs.Screen name="emergencia" />
        <Tabs.Screen name="faq" />
        <Tabs.Screen name="menu" />
        
        <Tabs.Screen name="nps" options={{ href: null }} />
        <Tabs.Screen name="chatIntro" options={{ href: null }} />
        <Tabs.Screen name="onboarding" options={{ href: null }} />
        <Tabs.Screen name="confirmacao" options={{ href: null }} />
        <Tabs.Screen name="index" options={{ href: null }} />
      </Tabs>

      {/* Navbar flutuante */}
      <Navbar />
    </View>
  );
}