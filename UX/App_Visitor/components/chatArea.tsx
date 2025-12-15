// components/ChatArea.tsx
import { ScrollView, StyleSheet, ImageSourcePropType } from "react-native";
import ChatBubble from "./MessageBubble";
import userAvatar from "../assets/images/user.png";
import botAvatar from "../assets/images/bot.png";

export type ChatMessage = {
  id: string | number;
  text: string;
  time: string;
  side: "left" | "right";
  avatar?: ImageSourcePropType;
  status?: "normal" | "pending" | "error";
  renderMarkdown?: boolean; // 👈 NOVO
};

type ChatAreaProps = {
  messages: ChatMessage[];
};

export default function ChatArea({ messages }: ChatAreaProps) {
  return (
    <ScrollView
      style={styles.container}
      contentContainerStyle={styles.content}
      showsVerticalScrollIndicator={false}
    >
      {messages.map((msg) => {
        const avatar =
          msg.avatar ?? (msg.side === "right" ? userAvatar : botAvatar);

        return (
          <ChatBubble
            key={String(msg.id)}
            text={msg.text}
            time={msg.time}
            side={msg.side}
            avatar={avatar}
            status={msg.status}
            renderMarkdown={msg.renderMarkdown} // 👈 passa adiante
          />
        );
      })}
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  content: {
    paddingVertical: 10,
    paddingBottom: 402,
  },
});
