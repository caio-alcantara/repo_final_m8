import { View, Text, TouchableOpacity, StyleSheet, Modal, Image} from "react-native";

interface Props{
    visible: boolean;
    onClose:()=> void;
    onConfirm:()=>void;
}

export default function PopUpInformacoes({ visible, onClose, onConfirm }: Props) {
    return (
      <Modal visible={visible} transparent animationType="fade">
        <View style={styles.overlay}>
          <View style={styles.container}>
                        
            <View style={styles.header}>
              <Text style={styles.title}>Informações gerais</Text>
              <View style={styles.icons}>              
                <View>
                    <TouchableOpacity>
                        <Image
                        source={require("@/assets/images/icons/editIcon.png")}/>
                        </TouchableOpacity>
                        
                        </View>
                        <View>
                        <TouchableOpacity onPress={onClose}>
                        <Image
                        source={require("@/assets/images/icons/X.png")}/> 
                        </TouchableOpacity>
                    </View>
                </View>
            </View>
  
            <Text style={styles.text}>
              O tour deve ser conduzido e acompanhado por um staff.
            </Text>
            <Text style={styles.text}>
            O cão-robô vai ficar responsável pela apresentação dos tópicos principais no primeiro andar, desde a recepção até a Dog House.
            </Text>
            <Text style={styles.text}>
            Ele está alimentado com dados sobre o Inteli até novembro de 2025.
            </Text>
            <Text style={styles.text}>
            O cão-robô parará para escutar dúvidas em cada um dos checkpoints dispostos na visualização presente na aba de mapa.
            </Text>
            <View style={styles.buttonBackground}>
            <TouchableOpacity style={styles.button} onPress={onConfirm}>
              <Text style={styles.buttonText}>Começar tour</Text>
            </TouchableOpacity>
            </View>
          </View>
        </View>
      </Modal>
    );
  }
  
  const styles = StyleSheet.create({
    overlay: {
      flex: 1,
      backgroundColor: "rgba(0,0,0,0.6)",
      justifyContent: "center",
      alignItems: "center",
    },
    container: {
      width: "85%",
      backgroundColor: "#402A78F2",
      padding: 20,
      borderRadius: 18,
      borderColor: "white",
      borderWidth: 1.5,   
    },

    header: {
      flexDirection: "row",
      justifyContent: "space-between",
      alignItems: "center",
      marginBottom: 12,
    },
    title: {
      color: "white",
      fontSize: 20,
      fontWeight: "bold",
    },
    text: {
      color: "white",
      marginBottom: 16,
      justifyContent: "center",

    },
    button: {
      backgroundColor: "#855EDE",
      paddingVertical: 12,
      borderRadius: 20,
      marginTop: 15,
      width: 160,
      height: 48,
      justifyContent: "center",

    },
    buttonText: {
      color: "white",
      textAlign: "center",
      fontWeight: "bold",
    },
    buttonBackground:{
    alignItems: "center",
    },
    icons:{
        flexDirection: "row",
        justifyContent: "center",
        alignItems: "center",
        marginBottom: 12,
    }
  });
