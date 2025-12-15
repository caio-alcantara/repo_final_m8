import { useState } from 'react'
import { View, Text, TextInput, StyleSheet, TouchableOpacity } from 'react-native'
import DateTimePicker from '@react-native-community/datetimepicker'

type Props = {
  label: string;
  value: string;
  onChange: (time: string) => void;
  width?: string;
  testID?: string;
};

export function TimePickerField({ label, value, onChange, width = "48%", testID }: Props) {
  const [show, setShow] = useState(false);

  const formatarHora = (date: Date) => {
    return date.toLocaleTimeString("pt-BR", {
      hour: "2-digit",
      minute: "2-digit",
      hour12: false
    });
  };

  const handleChange = (event, selectedDate) => {
    if (selectedDate) {
      onChange(formatarHora(selectedDate));
    }
    setShow(false);
  };

  return (
    <View style={[styles.input_section, { width }]}>
      <Text style={styles.label}>{label}</Text>
      
      <TouchableOpacity onPress={() => setShow(true)} activeOpacity={0.8}>
        <TextInput
          style={styles.input}
          value={value}
          editable={false}
          pointerEvents="none"
        />
      </TouchableOpacity>

      {show && (
        <DateTimePicker
          testID={testID}
          value={new Date()}
          mode="time"
          is24Hour={true}
          display="default"
          onChange={handleChange}
        />
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  input_section: {
    display: "flex",
    flexDirection: "column",
    justifyContent: "space-between",
    borderWidth: 1,
    borderColor: "#E5E5E5",
    borderRadius: 12,
    paddingHorizontal: 10,
    paddingVertical: 8,
    marginBottom: 12,
  },
  label: {
    color: "rgba(19, 26, 41, 0.48)",
    fontSize: 12,
  },
  input: {
    fontSize: 14,
    paddingHorizontal: 0,
  },
});
