import { View, Text, StyleSheet } from 'react-native'
import { Picker } from '@react-native-picker/picker'

type Props = {
  label: string;
  value: string;
  onChange: (value: string) => void;
  width?: string;
};

const estadosBrasil = [
  "AC", "AL", "AP", "AM", "BA", "CE", "DF", "ES", "GO", "MA",
  "MT", "MS", "MG", "PA", "PB", "PR", "PE", "PI", "RJ", "RN",
  "RS", "RO", "RR", "SC", "SP", "SE", "TO"
];

export function StatePickerField({ label, value, onChange, width = "48%" }: Props) {
  return (
    <View style={[styles.input_section, { width }]}>
      <Text style={styles.label}>{label}</Text>
      
      <View style={[styles.input, { paddingLeft: 0, paddingRight: 0 }]}>
        <Picker
          selectedValue={value}
          onValueChange={onChange}
        >
          <Picker.Item label="Estado" value="" />
          {estadosBrasil.map(uf => (
            <Picker.Item key={uf} label={uf} value={uf} />
          ))}
        </Picker>
      </View>
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