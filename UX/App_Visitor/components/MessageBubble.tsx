import React from "react";
import {
  View,
  Text,
  StyleSheet,
  Image,
  ImageSourcePropType,
} from "react-native";
import Markdown from "react-native-markdown-display";

type Props = {
  text: string;
  time: string;
  side: "left" | "right";
  avatar: ImageSourcePropType;
  status?: "normal" | "pending" | "error";
  renderMarkdown?: boolean; // 👈 NOVO
};

export default function MessageBubble({
  text,
  time,
  side,
  avatar,
  status = "normal",
  renderMarkdown,
}: Props) {
  const isLeft = side === "left";

  return (
    <View
      style={[
        styles.row,
        isLeft ? styles.rowLeft : styles.rowRight,
      ]}
    >
      {isLeft && (
        <Image source={avatar} style={styles.avatar} />
      )}

      <View
        style={[
          styles.bubble,
          isLeft ? styles.bubbleLeft : styles.bubbleRight,
          status === "error" && styles.bubbleError,
          status === "pending" && styles.bubblePending,
        ]}
      >
        {renderMarkdown ? (
          <Markdown style={markdownStyles}>
            {text}
          </Markdown>
        ) : (
          <Text style={styles.text}>{text}</Text>
        )}

        <View style={styles.footerRow}>
          <Text style={styles.time}>{time}</Text>
          {status === "pending" && (
            <Text style={styles.status}>aguardando…</Text>
          )}
          {status === "error" && (
            <Text style={styles.status}>erro</Text>
          )}
        </View>
      </View>

      {!isLeft && (
        <Image source={avatar} style={styles.avatar} />
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  row: {
    flexDirection: "row",
    marginVertical: 6,
    paddingHorizontal: 10,
  },
  rowLeft: {
    justifyContent: "flex-start",
  },
  rowRight: {
    justifyContent: "flex-end",
  },
  avatar: {
    width: 32,
    height: 32,
    borderRadius: 16,
    marginHorizontal: 6,
  },
  bubble: {
    maxWidth: "75%",
    borderRadius: 18,
    paddingVertical: 8,
    paddingHorizontal: 12,
  },
  bubbleLeft: {
    backgroundColor: "#2B2340",
    borderTopLeftRadius: 4,
  },
  bubbleRight: {
    backgroundColor: "#8141C2",
    borderTopRightRadius: 4,
  },
  bubbleError: {
    borderWidth: 1,
    borderColor: "#FF6B6B",
  },
  bubblePending: {
    borderWidth: 1,
    borderColor: "#FFD166",
  },
  text: {
    color: "#FFFFFF",
    fontSize: 16,
  },
  footerRow: {
    flexDirection: "row",
    justifyContent: "flex-end",
    marginTop: 4,
  },
  time: {
    color: "#B0A8C7",
    fontSize: 10,
    marginLeft: 8,
  },
  status: {
    color: "#FFD166",
    fontSize: 10,
    marginLeft: 8,
  },
});

const markdownStyles = {
  body: {
    color: "#FFFFFF",
    fontSize: 16,
  },
  strong: {
    fontWeight: "700",
  },
  em: {
    fontStyle: "italic",
  },
  paragraph: {
    marginTop: 0,
    marginBottom: 4,
  },
  bullet_list: {
    marginVertical: 4,
  },
  ordered_list: {
    marginVertical: 4,
  },
} as const;
