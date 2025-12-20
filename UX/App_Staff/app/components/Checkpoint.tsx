import { View, Text, StyleSheet, Dimensions } from "react-native";

type CheckpointProps = {
  id: number;
  status: string;
  label: string;
};

export default function Checkpoint({ id, status, label }: CheckpointProps) {
  const getColor = () => {
    switch (status) {
      case "done":
      case "finished":
        return styles.status_done;
      case "in_progress":
      case "running":
        return styles.status_in_progress;
      case "not_started":
      case "pending":
        return styles.status_not_started;
      case "skipped":
        return styles.status_skipped;
      default:
        return styles.status_not_started;
    }
  };

  const getStatusIcon = () => {
    switch (status) {
      case "done":
      case "finished":
        return "✓";
      case "in_progress":
      case "running":
        return "→";
      case "skipped":
        return "✗";
      default:
        return "○";
    }
  };

  const align = (id: number) => {
    return id % 2 === 0 ? styles.align_direct : styles.align_reverse;
  };

  const screenWidth = Dimensions.get("window").width;
  const isSmallScreen = screenWidth < 380;

  return (
    <View style={[{ alignItems: "center" }, align(id)]}>
      <View style={[styles.statusDot, getColor()]}>
        <Text style={styles.statusIcon}>{getStatusIcon()}</Text>
      </View>
      <Text style={[styles.label, isSmallScreen && styles.labelSmall]}>
        {label}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  statusDot: {
    width: 18,
    height: 18,
    borderRadius: 9,
    margin: 8,
    justifyContent: "center",
    alignItems: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.25,
    shadowRadius: 3,
    elevation: 5,
  },
  statusIcon: {
    color: "white",
    fontSize: 11,
    fontWeight: "bold",
  },
  status_done: {
    backgroundColor: "#06ED02",
  },
  status_in_progress: {
    backgroundColor: "#FFBB00",
  },
  status_not_started: {
    backgroundColor: "#979797",
  },
  status_skipped: {
    backgroundColor: "#E74C3C",
  },

  label: {
    color: "white",
    paddingVertical: 5,
    paddingHorizontal: 8,
    backgroundColor: "#5C3DA9",
    borderRadius: 6,
    textTransform: "capitalize",
    textAlign: "center",
    fontSize: 13,
    fontWeight: "600",
    maxWidth: 80,
    overflow: "hidden",
  },
  labelSmall: {
    fontSize: 11,
    paddingVertical: 4,
    paddingHorizontal: 6,
    maxWidth: 65,
  },

  align_reverse: {
    flexDirection: "column-reverse",
  },
  align_direct: {
    flexDirection: "column",
  },
});
