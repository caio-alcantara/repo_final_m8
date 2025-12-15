import React from 'react';
import {View,Text,StyleSheet,ImageBackground,StatusBar,TouchableOpacity,Image}from 'react-native';
import { useRouter } from 'expo-router';

export default function WelcomeScreen() {
    const router = useRouter();
    
    const handleEntry = () => {
        router.replace('/');
    };

    return (
        <View style={styles.container}>
            <StatusBar barStyle="light-content" backgroundColor="transparent" translucent />
            
            <ImageBackground
                source={require("@/assets/images/backgroung_login.png")} 
                style={styles.imageBackground}
                resizeMode="cover"
            >
                <View style={styles.overlayContent}>
                    
                    <Image 
                        source={require("@/assets/images/logo_inteli.png")}
                        style={styles.logo}
                        resizeMode="contain"
                    />
                    
                    <Text style={styles.greetingText}>Olá,Bem vindo!</Text>
                    
                    <TouchableOpacity 
                        style={styles.button} 
                        onPress={handleEntry}
                    >
                        <Text style={styles.buttonText}>Entrar</Text>
                    </TouchableOpacity>
                    
                </View>
            </ImageBackground>
        </View>
    );
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        maxHeight: "100%"
    },
    imageBackground: {
        flex: 1,
        justifyContent: 'flex-end',
    },
    overlayContent: {
        backgroundColor:"rgba(39, 32, 54, 0.84)",
        paddingHorizontal: 30, 
        paddingTop: 40,       
        paddingBottom: 40, 
        borderTopLeftRadius: 30,
        borderTopRightRadius: 30,
        alignItems: 'center',
    },
    logo: {
        width: 100, 
        height: 40,
        marginBottom: 20,
    },
    greetingText: {
        color: 'white',
        fontSize: 24,
        fontWeight: 'bold',
        marginBottom: 30,
    },
    button: {
        backgroundColor: '#855EDE', 
        paddingVertical: 12,
        borderRadius: 20,
        width: '100%', 
        alignItems: 'center',
    },
    buttonText: {
        color: 'white',
        fontSize: 18,
        fontWeight: 'bold',
    },
});