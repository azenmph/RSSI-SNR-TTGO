#include <Arduino.h>
#include "lmic_project_config.h"
#include <lmic.h>
#include <hal/hal.h>
#include <U8g2lib.h>
#include "images.h"



U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

const int botonPin = 25;

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},  // DIO0, DIO1, DIO2
};


osjob_t sendjob;

// Definición de las claves OTAA para recibir a través de TTN
static const PROGMEM u1_t APPEUI[8] = {0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
static const PROGMEM u1_t DEVEUI[8] = {0xAE, 0x2A, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
static const PROGMEM u1_t APPKEY[16] = {0x8B, 0x5F, 0xA9, 0xA9, 0x91, 0x46, 0x92, 0x2F, 0x66, 0xF1, 0x50, 0x98, 0xC0, 0x61, 0x0F, 0x09};

unsigned long lastButtonPress = 0;

void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

void os_getDevKey (u1_t* buf) {
    memcpy(buf, APPKEY, 16);
}


void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_JOINED: // Una vez que se ha unido a la red
            Serial.println(F("EV_JOINED"));
            // Desactiva las uniones periódicas y se establece el tiempo de envío
            LMIC_setLinkCheckMode(0);
           // do_send(); // Envia un paquete despues de la union
            break; 

        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));
            // Una vez que se complete el envío
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F("ACK Recibido"));
                // Recibimos downlink
                //Serial.println(F("Downlink recibido"));
                // Muestra los valores RSSI y SNR del downlink
                int rssi = LMIC.rssi-72;
            
                float snr = (LMIC.snr)/4.0; 

                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB10_tr);
                u8g2.drawStr(0, 24, "RSSI:");
                u8g2.drawStr(64, 24, String(rssi).c_str());
                u8g2.drawStr(0, 48, "SNR:");
                u8g2.drawStr(64, 48, String(snr).c_str());
                u8g2.sendBuffer();
            }

            break;
            }
}

void setup() {
  
    Serial.begin(115200);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.drawXBM(1, 1, logo_medialab_width, logo_medialab_height, medialab_bits);
    u8g2.sendBuffer();
    delay(3000);

    pinMode(botonPin, INPUT_PULLUP);
    
    os_init();
    LMIC_reset();
    
    // Configurar la banda de frecuencia
    #ifdef CFG_eu868
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);
    #endif
    
     // Configuración de SF y opcion de transmisión
    LMIC_setDrTxpow(DR_SF7,14); // Establece SF7 como el factor de propagación y 14 dBm como la potencia de transmisión

    // Unión a la red
    //LMIC_startJoining();
    Serial.println("Configuración completada.");
}

void do_send() {
    if (!(LMIC.opmode & OP_TXRXPEND)) {
        uint8_t data[] = {0x01};
        LMIC_setTxData2(1, data, sizeof(data), 1);
        Serial.println(F("Enviando..."));
    } else {
        Serial.println(F("Operación pendiente"));
    }
}

void loop() {
    unsigned long currentMillis = millis();
    if (digitalRead(botonPin) == LOW && currentMillis - lastButtonPress > 1000) {
        Serial.println("Botón pulsado");
        // Imagen de cargando
        lastButtonPress = currentMillis;

 if (!(LMIC.opmode & OP_JOINING) && LMIC.devaddr == 0) {
            Serial.println("Iniciando proceso de unión...");
            LMIC_startJoining();
        } else if (LMIC.devaddr != 0) {
            Serial.println("Botón pulsado, enviando datos...");
            do_send();
        }
    }
    os_runloop_once();
}


