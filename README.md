# XS1278-LoRa-RaspberryPi

## It's just for P2P transmissions. Not for LoRaWAN.
  
May work with all sx127x chips, I didn't test it on these chips

This source uses wiringPi. Installation on raspbian:  
```
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build
```
1. Wire raspberry and lora chip by the table below

|Raspberry Pi | SX1278 |
|----|----------|
| GPIO 0 | RESET| 
| GPIO 22 |DIO 0     |
| SPI CE0 (GPIO8, pin 24)| NSS | 
| MOSI (GPIO10, pin 19)| MOSI | 
| MISO (GPIO9, pin 21)| MISO | 
| CLK (GPIO11, pin 23)| SCK | 

2. Clone the repo

3. Enter cloned repo dir

4.  make all

5. Try file ./test_unitaire_RA02

It uses continuous mode on module. Radio module will continiuously receive packets and each time execute user callback.





# Changelog

**11/07/2024 : ** Creation du README.md 

> **Notes :**


> - Licence : **licence publique générale** ![enter image description here](https://img.shields.io/badge/licence-GPL-green.svg)
> - Auteur  **Philippe SIMIER  - F4JRE**
>  ![enter image description here](https://img.shields.io/badge/built-passing-green.svg)
<!-- TOOLBOX 

Génération des badges : https://shields.io/
Génération de ce fichier : https://stackedit.io/editor#

