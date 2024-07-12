/* 
 * File:   SX1278.h
 * Author: philippe SIMIER Lycée Touchard Le Mans
 *
 * Created on 8 juillet 2024, 08:32
 */

#ifndef SX1278_H
#define SX1278_H

#include "Spi.h"
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>


#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FR_MSB 0x06
#define REG_FR_DIM 0x07
#define REG_FR_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_OCP 0x0B
#define REG_LNA 0x0C
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_RX_HEADER_CNT_VALUE_MSB 0x14
#define REG_RX_HEADER_CNT_VALUE_LSB 0x15
#define REG_RX_PACKET_CNT_VALUE_MSB 0x16
#define REG_RX_PACKET_CNT_VALUE_LSB 0x17
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1A
#define REG_RSSI_VALUE 0x1B
#define REG_MODEM_CONFIG_1 0x1D
#define REG_MODEM_CONFIG_2 0x1E
#define REG_MODEM_CONFIG_3 0x26
#define REG_PAYLOAD_LENGTH 0x22
#define REG_FIFO_RX_BYTE_ADDR 0x25
#define REG_PA_DAC 0x4d
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_TEMP 0x3c
#define REG_SYNC_WORD 0x39
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_DETECT_OPTIMIZE 0x31
#define REG_DETECTION_THRESHOLD 0x37

#define TX_BASE_ADDR 0x00
#define RX_BASE_ADDR 0x00

#define LORA_MODE 0x80

#define SLEEP_MODE 0x00
#define STDBY_MODE 0x01
#define TX_MODE 0x03
#define RXCONT_MODE 0x05

#define FLAG_RXDONE 0x40
#define FLAG_PAYLOAD_CRC_ERROR    0x20
#define FLAG_VALID_HEADER  0x10
#define FLAG_TXDONE 0x08
#define FLAG_CADDONE 0x04
#define FLAG_CADDETECTED 0x01

typedef enum BandWidth{
    BW7_8 =0,
    BW10_4 = 1<<4,
    BW15_6 = 2<<4,
    BW20_8 = 3<<4,
    BW31_25 = 4<<4,
    BW41_7 = 5<<4,
    BW62_5 = 6<<4,
    BW125 = 7<<4,
    BW250 = 8<<4,
    BW500 = 9<<4,
} BandWidth;

typedef enum SpreadingFactor{
    SF7 = 7<<4,
    SF8 = 8<<4,
    SF9 = 9<<4,
    SF10 = 10<<4,
    SF11 = 11<<4,
    SF12 = 12<<4,
} SpreadingFactor;

typedef enum ErrorCodingRate{
    CR5 = 1<<1,
    CR6 = 2<<1,
    CR7 = 3<<1,
    CR8 = 4<<1,
} ErrorCodingRate;

typedef enum OutputPower{
    OP0 = 0,
    OP1 = 1,
    OP2 = 2,
    OP3 = 3,
    OP4 = 4,
    OP5 = 5,
    OP6 = 6,
    OP7 = 7,
    OP8 = 8,
    OP9 = 9,
    OP10 = 10,
    OP11 = 11,
    OP12 = 12,
    OP13 = 13,
    OP14 = 14,
    OP15 = 15,
    OP16 = 16,
    OP17 = 17,
    OP20 = 20,
} OutputPower;

typedef enum PowerAmplifireOutputPin{
    RFO = 0x70,
    PA_BOOST = 0xf0,
} PowerAmplifireOutputPin;

typedef enum LnaGain{
    G1 = 1,
    G2 = 2,
    G3 = 3,
    G4 = 4,
    G5 = 5,
    G6 = 6,
} LnaGain;




class SX1278 {
    
public:
    
    SX1278();
    SX1278(const SX1278& orig) = delete;
    virtual ~SX1278();
    
    void setPins(int _channel, int _reset, int DIO_0);
    void begin(double frequency = 433775000);
    void send(int8_t *buf, int8_t size);
    void send(const std::string &message);
    
    void continuous_receive();
    void onRxDone(void (*ptrFuncRX)(char*, int, float));
    void onTxDone(void (*ptrFuncTX)(void));
    
    
    std::string payload();

    
    
    
    
private:
    
    Spi *spi;
    int gpio_reset;        //raspberry GPIO pin connected to RESET pin of LoRa chip
    int gpio_DIO_0;        //raspberry GPIO pin connected to DIO0 pin of LoRa chip to detect TX and Rx done events. 
    int channel;
    
    BandWidth bw;
    SpreadingFactor sf;    //only from SF7 to SF12. SF6 not support yet.
    ErrorCodingRate ecr;
    double freq;           // Frequency in Hz. Example 433775000
    double tsym;           // temps du symbole
    
    unsigned int preambleLen;
    unsigned char syncWord;
    
    OutputPower outPower;
    PowerAmplifireOutputPin powerOutPin; //This chips has to outputs for signal "High power" and regular.
    unsigned char ocp;     //Over Current Protection. 0 to turn OFF. Else reduces current from 45mA to 240mA    
        
    void (*callback_Rx)(char*, int,float);  // pointeur sur une fonction callback utilisateur de type void(char*,int,float)
    void (*callback_Tx)(void);
    
    int rssi;
    float snr;
    int8_t bufferRX[257];  // Buffer de réception
    
    void reset();
    
    int8_t get_op_mode();
    void set_explicit_header();
    void set_errorcrc(ErrorCodingRate cr);
    void set_crc_on();
    void set_crc_off();
    void set_bandwidth(BandWidth bw);
    void set_sf(SpreadingFactor sf);
    void set_tx_power(OutputPower power, PowerAmplifireOutputPin pa_pin);    // OP20 & PA_BOOST
    void set_syncw(unsigned char word); 
    void set_preamble(int preambleLen);
    void set_agc(_Bool AGC);
    void set_lna(LnaGain lnaGain, _Bool lnaBoost);
    void set_ocp(unsigned char OCP);
    
    void set_freq(double freq);
    
    void set_lowdatarateoptimize_off();
    void set_lowdatarateoptimize_on();
    
    void lora_write_fifo(int8_t *buf, int8_t size);
    
    void set_lora_mode();
    void set_sleep_mode();
    void set_standby_mode();
    void set_tx_mode();
    void set_rxcont_mode();
    
    void calculate_tsym();
    double calculate_packet_t(int8_t payloadLen);
    
    void set_dio0_rx_mapping();
    void set_dio0_tx_mapping();
    void reset_irq_flags();
    void Done_TX_RX();
    
    void get_rssi_pkt();
    void get_snr();
    
    static void interruptHandler();
};

extern SX1278 loRa;  

#endif /* SX1278_H */

