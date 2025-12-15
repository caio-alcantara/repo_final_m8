import { useState, useMemo } from "react";
import { View, Text, TouchableOpacity, StyleSheet } from "react-native";

type Props = {
  onDateChange: (date: Date) => void;
};

export default function DateSelector({ onDateChange }: Props) {
  const [selected, setSelected] = useState(new Date());

  function handleSelect(date: Date) {
    setSelected(date);
    onDateChange(date);
  }

  const dates = useMemo(() => {
    const arr = [];
    for (let i = -2; i <= 2; i++) {
      const d = new Date(selected);
      d.setDate(selected.getDate() + i);

      arr.push({
        offset: i,
        date: d,
        day: d.getDate(),
        month: d.toLocaleString("pt-BR", { month: "short" }).toUpperCase(),
      });
    }
    return arr;
  }, [selected]);

  return (
    <View style={styles.container}>
      <View style={styles.row}>
        {dates.map((item, index) => {
          const isCenter = item.offset === 0;
          const isMedium = Math.abs(item.offset) === 1;
          const isSmall = Math.abs(item.offset) === 2;

          return (
            <TouchableOpacity
              key={index}
              onPress={() => handleSelect(item.date)}
              style={[
                styles.box,
                isCenter && styles.big,
                isMedium && styles.medium,
                isSmall && styles.small,
              ]}
            >
              <Text style={styles.dayText}>{item.day}</Text>

              {isCenter && (
                <Text style={styles.monthText}>{item.month}</Text>
              )}
            </TouchableOpacity>
          );
        })}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    backgroundColor: "#201A2C",
    paddingVertical: 24,
    alignItems: "center",
    position: "absolute",
    top: 120
  },
  row: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
  },

  box: {
    borderWidth: 2,
    borderColor: "#5A43A6",
    justifyContent: "center",
    alignItems: "center",
    borderRadius: 16,
    backgroundColor: "#241d33",
  },

  // Tamanhos
  small: {
    width: 30,
    height: 30,
    borderRadius: 4,
    fontSize: 12
  },
  medium: {
    width: 45,
    height: 45,
    borderRadius: 8,
  },
  big: {
    width: 70,
    height: 70,
    borderRadius: 16,
    backgroundColor: "#372b6c",
  },

  dayText: {
    color: "white",
    fontSize: 20,
  },
  monthText: {
    color: "white",
    fontSize: 16,
    marginTop: 4,
    fontWeight: "bold",
  },
});
