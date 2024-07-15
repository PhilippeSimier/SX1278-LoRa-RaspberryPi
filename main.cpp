/* 
 * File:   main.cpp
 * Author: philippe SIMIER Lycée Touchard Washington
 * 
 * Programme test unitaire classe SX1278
 * test les 2 méthodes continuous_receive() & send() 
 *
 * Created on 7 juillet 2024, 16:14
 */

#include <cstdlib>
#include <iostream>
#include <string> 
#include "SX1278.h"

using namespace std;

void callback_Rx(char* payload, int rssi, float snr); // user callback function for when a packet is received. 
void callback_Tx(void);  // callback function for when a packet is transmited.

int main(int argc, char** argv) {

    cout << "Programme LoRa" << endl;
       
    int8_t buffer[] = "LoRa";
    
    try {
      
        
        loRa.onRxDone(callback_Rx);  // Register a callback function for when a packet is received.
        loRa.onTxDone(callback_Tx);
        loRa.begin();
        loRa.continuous_receive(); // Puts the radio in continuous receive mode.
        
        sleep(1);
        loRa.send(buffer, 4);
        loRa.send("Bonjour le monde");
        loRa.send("Coucou");
        
        while(1){
            
            sleep(1);
        }     
       
    } catch (const std::runtime_error &e) {     
        std::cout << "Exception caught: " << e.what() << std::endl;
    }
    return 0;
}

/**
 * @brief Callback utilisateur appelé après la reception compléte 
 *        d'un packet
 * @param buffer une chaine de caratères char*
 * @param rssi  le niveau de reception dBm
 * @param snr   le rapport signal / bruit
 */
void callback_Rx(char* payload, int rssi, float snr) {
    std::cout << "Rx done : " << payload;
    std::cout << " RSSI : " << rssi << "dBm";
    std::cout << " SNR  : " << snr  << "dB" << std::endl; 
 
}

void callback_Tx(void) {
    std::cout << "Tx done : " << std::endl;
}


