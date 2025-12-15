import { View } from 'react-native'
import { CheckBox } from 'react-native-elements'
import { FormInput } from './FormInput'

type Props = {
  hasCompanion: boolean;
  companionName: string;
  companionCpf: string;
  onToggleCompanion: () => void;
  onChangeCompanionName: (text: string) => void;
  onChangeCompanionCpf: (text: string) => void;
};

export function CompanionSection({
  hasCompanion,
  companionName,
  companionCpf,
  onToggleCompanion,
  onChangeCompanionName,
  onChangeCompanionCpf
}: Props) {
  return (
    <View>
      <CheckBox
        title='Vai trazer acompanhante?'
        checked={hasCompanion}
        onPress={onToggleCompanion}
      />

      {hasCompanion && (
        <>
          <FormInput
            label="Nome do Acompanhante"
            value={companionName}
            onChangeText={onChangeCompanionName}
            width="95%"
          />

          <FormInput
            label="CPF do Acompanhante"
            value={companionCpf}
            onChangeText={onChangeCompanionCpf}
            width="95%"
          />
        </>
      )}
    </View>
  );
}