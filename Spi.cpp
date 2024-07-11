/* 
 * File:   Spi.cpp
 * Author: philippe SIMIER Lycée Touchard Le Mans
 * 
 * Created on 7 juillet 2024, 16:29
 * 
 * Une classe SPI  utilisée dans les applications où un Raspberry Pi 
 * doit communiquer avec des périphériques externes utilisant l'interface SPI.
 * 
 */

#include "Spi.h"


/**
 * @brief Le constructeur configure les paramètres nécessaires pour la communication SPI, 
 * comme  le canal à utiliser et la vitesse de l'horloge. 
 * Il ouvre le périphérique SPI correspondant
 * /dev/spidev0.0 pour le canal 0
 * 
 * @param channel
 * @param speed
 */
Spi::Spi(int channel, int speed) :
channel(channel),
speed(speed)
 {

    if (wiringPiSetupGpio() == -1) {
        throw std::runtime_error("Exception Spi wiringPiSetupGpio");
    }

    if ((channel = wiringPiSPISetup(channel, speed)) < 0) {
        throw std::runtime_error("Exception Spi wiringPiSPISetup");
    }
}


Spi::~Spi() {
}

/**
 * @brief méthode pour lire un registre
 * @param reg l'adresse du registre
 * @return la valeur lue dans le registre
 */
int8_t Spi::read_reg(int8_t reg) {

    int ret;
    unsigned char data[2];

    data[0] = reg;
    data[1] = 0x00;

    ret = wiringPiSPIDataRW(channel, data, 2);
    if (ret == -1)
        throw std::runtime_error("Exception Spi read_byte");

    return data[1];
}

/**
 * 
 * @param reg l'adresse du registre
 * @param byte
 * @return 
 */
int Spi::write_reg(int8_t reg, int8_t value) {

    int ret;
    unsigned char data[2];

    data[0] = (reg | 0x80); // Bit whr positionné à 1 pour accés en écriture
    data[1] = value;

    ret = wiringPiSPIDataRW(channel, data, 2);
    return ret;
}

/**
 * 
 * @param reg   adresse de base du fifo
 * @param buff  un pointeur vers un buffer 
 * @param size  la taille des données 
 * @return      le nombre d'octets lus
 */
int Spi::read_fifo(int8_t reg, int8_t *buff, int8_t size) {

    int ret;
    char unsigned data[257] = {0};

    memset(buff, '\0', size);
    data[0] = reg;
    ret = wiringPiSPIDataRW(channel, data, size + 1);
    memcpy(buff, &data[1], ret - 1);

    return ret;
}

/**
 * 
 * @param reg addresse de base du fifo
 * @param buff un pointeur sur un buffer
 * @param size la taille des datas à écrire
 * @return le nombre de d'octets écrits
 */
int Spi::write_fifo(int8_t reg, int8_t *buff, int8_t size) {

    int ret;
    char unsigned data[257] = {0};

    data[0] = (reg | 0x80);
    memcpy(&data[1], buff, size);
    ret = wiringPiSPIDataRW(channel, data, size + 1);


    return ret;

}


