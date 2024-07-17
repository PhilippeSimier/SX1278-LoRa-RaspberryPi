# SX1278 API

## Include Library

```cpp
#include "SX1278.h"
```

## Setup

### Begin

Initialize the library with the specified frequency.

```cpp
loRa.begin(433775000);
```
 * `frequency` - frequency in Hz (`433E6`, `868E6`, `915E6`)


### Set pins

Override the default `NSS`, `NRESET`, and `DIO0` pins used by the library. **Must** be called before `LoRa.begin()`.

```arduino
loRa.setPins(channel, reset, DIO_0);
```
 * `channel` - new slave channel pin to use, defaults CE0 to `0`
 * `reset` - new reset pin to use, defaults to `0`
 * `dio0` - new DIO0 pin to use, defaults to `22`.  **Must** be interrupt capable via wiringPiISR.

This call is optional and only needs to be used if you need to change the default pins used.



#### Pin dio0 interrupt callbacks

The dio0 pin can be used for channel activity detection callback, transmission finish callback and/or receiving callback, check  `onTxDone`, and `onRxDone`.


## Sending data

### Begin packet

Start the sequence of sending a packet.

```cpp
loRa << beginPacket;
```

### Writing

Write data to the packet. Each packet can contain up to 255 bytes.

```cpp
loRa << "Welcome LoRa";
```
or
```cpp
 send(buffer, length)
```
* `buffer` - char* data to write to packet
* `length` - int size of data to write

```cpp
char buffer[] = "message";
loRa.send(buffer, sizeof(buffer));
```
*

### End packet

End the sequence of sending a packet.

```cpp
loRa << beginPacket << "welcom LoRa" << endPacket;
```


### Tx Done

**WARNING**: TxDone callback uses the interrupt pin on the `dio0` check `setPins` function!

### Register callback

Register a callback function for when a packet transmission finish.

```cpp
loRa.onRxDone(callback_Tx);

void callback_Tx() {
 // ...
}
```

 * `callback_Tx` - function to call when a packet transmission finish.

## Receiving data Rx Done

### Continuous receive mode

**WARNING**: Receive callback uses the interrupt pin on the `dio0`, check `setPins` function!

#### Register callback

Register a callback function for when a packet is received.

```cpp
loRa.onTxDone(callback_Rx);

void callback_Rx(char* payload, int rssi, float snr) { 
 // ...
}
```

 * `callback_Rx` - function to call when a packet is received.
 * `payload`  the char*
 * `rssi` the averaged RSSI of the last received packet (dBm).
 * `snr` the estimated SNR of the received packet in dB.

#### Receive mode

Puts the radio in continuous receive mode.

```cpp
loRa.continuous_receive();
```




## Radio parameters

### Spreading Factor

Change the spreading factor of the radio.

```arduino
loRa.set_sf(sf);
```
 * `sf` - spreading factor, defaults to `SF12`

Supported values are between `SF7` and `SF12`. 

### Signal Bandwidth

Change the signal bandwidth of the radio.

```arduino
loRa.set_bandwidth(bw);
```

 * `bw` - signal bandwidth in Hz, defaults to `BW125`.

Supported values are `BW7_8`, `BW10_4`, `BW15_6`, `BW20_8`, `BW31_25`, `BW41_7`, `BW62_5`, `BW125`, `BW250`, and `BW500`.

### Coding Rate

Change the coding rate of the radio.

```arduino
loRa.set_errorcrc(cr);;
```

 * `cr` - denominator of the coding rate, defaults to `CR5`

Supported values are between `CR5`,`CR6`, `CR7` and `CR8`,  these correspond to coding rates of `4/5` `4/6`  `4/7` and `4/8`. The coding rate numerator is fixed at `4`.

### Preamble Length

Change the preamble length of the radio.

```arduino
loRa.set_preamble(int preambleLen);;
```

 * `preambleLen` - preamble length in symbols, defaults to `6`

Supported values are between `6` and `65535`.

### Sync Word

Change the sync word of the radio.

```arduino
loRa.set_syncw(word);
```

 * `word` - byte value to use as the sync word, defaults to `0x12`


