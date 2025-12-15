import { ReactNode } from "react";
import { Image, StyleSheet, Text, View } from "react-native";

interface CardTutorialProps {
  text: string;
  image?: any;
  icon?: ReactNode;
}

export default function CardTutorial({ text, image, icon }: CardTutorialProps) {
  return (
    <View style={styles.content}>
      <Text style={styles.text}>{text}</Text>
      {image && <Image source={image} style={styles.image} resizeMode="contain" />}
      {icon && <View style={styles.iconContainer}>{icon}</View>}
    </View>
  );
}

const styles = StyleSheet.create({
  content: {
    alignItems: "center",
    justifyContent: "center",
    gap: 30,
    paddingVertical: 20,
  },
  text: {
    color: "#fff",
    textAlign: "center",
    fontSize: 20,
    lineHeight: 30,
    fontWeight: "500",
    paddingHorizontal: 10,
  },
  image: {
    width: 250,
    height: 250,
  },
  iconContainer: {
    marginTop: 10,
  },
});
