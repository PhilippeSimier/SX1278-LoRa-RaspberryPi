/* 
 * File:   main.cpp
 * Author: philippe SIMIER
 * 
 * Programme test unitaire classe SX1278
 * test les 4 méthodes 
 *
 * Created on 7 juillet 2024, 16:14
 */

#include <cstdlib>
#include <iostream>
#include <string> 
#include "SX1278.h"

using namespace std;

void callback_Rx(void);  // fonction de rappel pour traiter les messages reçus
void callback_Tx(void);  // fonction de rappel packet envoyé

int main(int argc, char** argv) {

    cout << "Programme LoRa" << endl;
       
    int8_t buffer[] = "LoRa";
    
    try {
      
        loRa.begin();
        loRa.set_callback_RX(callback_Rx);
        loRa.set_callback_TX(callback_Tx);
        loRa.continuous_receive(); // passsage en mode reception continue
        sleep(60);
        loRa.send(buffer, 4);
        loRa.send("Bonjour le monde");
        
        while(1){
            sleep(1);
        }     
       
    } catch (const std::runtime_error &e) {     
        std::cout << "Exception caught: " << e.what() << std::endl;
    }
    return 0;
}

void callback_Rx(void) {
    std::cout << "Rx done : " << loRa.bufferRX;
    std::cout << " RSSI : " << loRa.rssi << "dBm";
    std::cout << " SNR  : " << loRa.snr  << "dB" << std::endl; 
 
}

void callback_Tx(void) {
    std::cout << "Tx done : " << std::endl;
}


