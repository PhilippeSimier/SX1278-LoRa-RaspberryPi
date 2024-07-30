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
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>


#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FR_MSB 0x06
#define REG_FR_MID 0x07
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
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_FIFO_RX_BYTE_ADDR 0x25
#define REG_MODEM_CONFIG_3 0x26
#define REG_DETECT_OPTIMIZE 0x31
#define REG_DETECTION_THRESHOLD 0x37
#define REG_TEMP 0x3c
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4d


#define TX_BASE_ADDR 0x00
#define RX_BASE_ADDR 0x00

#define MODE_LORA 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RXCONT 0x05
#define MODE_RXSINGLE 0x06

#define FLAG_RXDONE 0x40
#define FLAG_PAYLOAD_CRC_ERROR    0x20
#define FLAG_VALID_HEADER  0x10
#define FLAG_TXDONE 0x08
#define FLAG_CADDONE 0x04
#define FLAG_CADDETECTED 0x01


class SX1278 {
public:

    enum BandWidth {
        BW7_8,
        BW10_4,
        BW15_6,
        BW20_8,
        BW31_25,
        BW41_7,
        BW62_5,
        BW125,
        BW250,
        BW500,
    };

    enum SpreadingFactor {
        SF6 = 6,
        SF7,
        SF8,
        SF9,
        SF10,
        SF11,
        SF12,
    };

    enum ErrorCodingRate {
        CR5 = 1 << 1,
        CR6 = 2 << 1,
        CR7 = 3 << 1,
        CR8 = 4 << 1,
    };

    enum PowerAmpPin {
        RFO = 0x70,
        PA_BOOST = 0xf0,
    };

    SX1278();
    SX1278(const SX1278& orig) = delete;
    virtual ~SX1278();

    void setPins(int _channel, int _reset, int DIO_0);
    void begin(double frequency = 433775000);
    void send(void);
    void send(int8_t *buf, int8_t size);
    void send(const std::string &message);
    void clear();

    void continuous_receive();
    void onRxDone(void (*ptrFuncRX)(char*, int, float));
    void onTxDone(void (*ptrFuncTX)(void));

    void set_bandwidth(BandWidth bw);
    void set_sf(SpreadingFactor sf);
    void set_tx_power(int8_t power, PowerAmpPin pa_pin); // TX power in dBm, defaults to 20 & PA_BOOST
    void set_syncw(unsigned char word);
    void set_preamble(int preambleLen);
    void set_ecr(ErrorCodingRate cr);

    SX1278& operator<<(SX1278& (*)(SX1278&));
    SX1278& operator<<(const std::string&);
    SX1278& operator<<(const int);
    SX1278& operator<<(const double);
    SX1278& operator<<(const char);
    SX1278& operator<<(const char *);
    SX1278& operator<<(const bool);

    BandWidth bwFromString(const std::string& str);
    BandWidth bwFromDouble(const double val);
    SpreadingFactor sfFromString(const std::string& str);
    ErrorCodingRate ecrFromString(const std::string& str);
    




private:

    Spi *spi;
    int gpio_reset; //raspberry GPIO pin connected to RESET pin of LoRa chip
    int gpio_DIO_0; //raspberry GPIO pin connected to DIO0 pin of LoRa chip to detect TX and Rx done events. 
    int channel;

    BandWidth bw;
    SpreadingFactor sf; //only from SF7 to SF12. SF6 not support yet.
    ErrorCodingRate ecr;
    double freq; // Frequency in Hz. Example 433775000
    long tsym;   // durée du symbole en ms

    unsigned int preambleLen;
    unsigned char syncWord;

    int8_t outPower;
    PowerAmpPin powerOutPin; //This chips has to outputs for signal "High power" and regular.
    unsigned char ocp; //Over Current Protection. 0 to turn OFF. Else reduces current from 45mA to 240mA    

    void (*callback_Rx)(char*, int, float); // pointeur sur une fonction callback utilisateur de type void(char*,int,float)
    void (*callback_Tx)(void);

    int rssi;
    float snr;
    int8_t bufferRX[257]; // Buffer de réception
    std::string bufferTX; // Buffer d'émission


    void reset();

    int8_t get_op_mode();
    void set_explicit_header();

    void set_crc_on();
    void set_crc_off();

    void set_agc(_Bool AGC);
    void set_lna(int8_t lnaGain, _Bool lnaBoost);
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
    
    void set_dio0_rx_mapping();
    void set_dio0_tx_mapping();
    void reset_irq_flags();
    void Done_TX_RX();

    void get_rssi_pkt();
    void get_snr();

    static void interruptHandler();
};

SX1278& endPacket(SX1278& sx);
SX1278& beginPacket(SX1278& sx);

extern SX1278 loRa;

#endif /* SX1278_H */

