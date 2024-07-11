/* 
 * File:   Spi.h
 * Author: philippe SIMIER Lycée Touchard Le Mans
 *
 * Created on 7 juillet 2024, 16:29
 * 
 * Une classe SPI  utilisée dans les applications où un Raspberry Pi 
 * doit communiquer avec des périphériques externes utilisant l'interface SPI.
 * 
 */

#ifndef SPI_H
#define SPI_H

#include <cstdlib>
#include <iostream>
#include <string.h>
#include <wiringPi.h>
#include "wiringPiSPI.h"

class Spi {
    
public:
    
    Spi(int channel=0, int speed=32000);
    Spi(const Spi& orig) = delete; // L'objet Spi n'est pas clonable
    virtual ~Spi();
    
    int8_t read_reg(int8_t reg);
    int write_reg(int8_t reg, int8_t byte);
    
    int read_fifo( int8_t reg, int8_t *buff, int8_t size);
    int write_fifo(int8_t reg, int8_t *buff, int8_t size);
    
    
    
protected:
    
    int channel;
    int speed;
 
};

#endif /* SPI_H */

