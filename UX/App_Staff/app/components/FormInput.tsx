import { View, Text, TextInput, StyleSheet } from 'react-native'

type Props = {
  label: string;
  value: string;
  onChangeText: (text: string) => void;
  width?: string;
  editable?: boolean;
  pointerEvents?: 'none' | 'auto';
};

export function FormInput({ 
  label, 
  value, 
  onChangeText, 
  width = "95%",
  editable = true,
  pointerEvents = 'auto'
}: Props) {
  return (
    <View style={[styles.input_section, { width }]}>
      <Text style={styles.label}>{label}</Text>
      <TextInput
        style={styles.input}
        editable={editable}
        onChangeText={onChangeText}
        value={value}
        pointerEvents={pointerEvents}
      />
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