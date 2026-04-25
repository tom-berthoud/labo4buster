#include <BusTerUtils.h>
#include <SPI.h>
#include <Wire.h>

// Adresses CAN calculées depuis les DIP switches
static uint32_t rxAddress = 0;  // adresse de réception (DIP1-7)
static uint32_t txAddress = 0;  // adresse d'émission (rxAddress XOR 1)
static bool analogMode = false; // DIP8 : gestion de l'analogique
static const uint32_t CAN_ID_ANALOG  = 0x000;
static const uint32_t CAN_ID_DIGITAL = 0x080;

// Dernières valeurs envoyées (pour détecter les changements)
static int    prevAnalog[4]  = {-1, -1, -1, -1};
static int    prevDigital    = -1;  

void setup() {
    BusTer_InitInputOutput();
    // Lecture des DIP switches
    rxAddress  = (uint32_t)digitalRead(BUSTER_PIN_DIPSWITCH_1) << 0;
    rxAddress |= (uint32_t)digitalRead(BUSTER_PIN_DIPSWITCH_2) << 1;
    rxAddress |= (uint32_t)digitalRead(BUSTER_PIN_DIPSWITCH_3) << 2;
    rxAddress |= (uint32_t)digitalRead(BUSTER_PIN_DIPSWITCH_4) << 3;
    rxAddress |= (uint32_t)digitalRead(BUSTER_PIN_DIPSWITCH_5) << 4;
    rxAddress |= (uint32_t)digitalRead(BUSTER_PIN_DIPSWITCH_6) << 5;
    rxAddress |= (uint32_t)digitalRead(BUSTER_PIN_DIPSWITCH_7) << 6;
    rxAddress &= 0x7F;

    analogMode = (digitalRead(BUSTER_PIN_DIPSWITCH_8) == HIGH);

    // Adresse d'émission = inversion du bit 0 (DIP1) car nous avons la meme adresse à un bit près
    txAddress = rxAddress ^ 0x01;
    BusTer_CanInit(BUSTER_CAN_1MBPS);

    // RXB0 : messages digitaux (type 1, bits[10:7]=0001)
    BusTer_CanSetMask(BUSTER_Rxm0, BUSTER_Can2_0A, 0x7FF);
    BusTer_CanSetFilter(BUSTER_Rxf0, BUSTER_Can2_0A, CAN_ID_DIGITAL | rxAddress);
    BusTer_CanSetFilter(BUSTER_Rxf1, BUSTER_Can2_0A, CAN_ID_DIGITAL | rxAddress);

    // RXB1 : messages analogiques (type 0, bits[10:7]=0000)
    BusTer_CanSetMask(BUSTER_Rxm1, BUSTER_Can2_0A, 0x7FF);
    BusTer_CanSetFilter(BUSTER_Rxf2, BUSTER_Can2_0A, CAN_ID_ANALOG | rxAddress);
    BusTer_CanSetFilter(BUSTER_Rxf3, BUSTER_Can2_0A, CAN_ID_ANALOG | rxAddress);
    BusTer_CanSetFilter(BUSTER_Rxf4, BUSTER_Can2_0A, CAN_ID_ANALOG | rxAddress);
    BusTer_CanSetFilter(BUSTER_Rxf5, BUSTER_Can2_0A, CAN_ID_ANALOG | rxAddress);

    BusTer_CanActivateNormalMode();
    digitalWrite(BUSTER_PIN_STATUS_LED_GREEN, HIGH);
    digitalWrite(BUSTER_PIN_STATUS_LED_RED, LOW);
}

void loop() {
    // Émission digitale
    INT8U digitalData = 0;
    digitalData |= (INT8U)(digitalRead(BUSTER_PIN_IN_0) << 0);
    digitalData |= (INT8U)(digitalRead(BUSTER_PIN_IN_1) << 1);
    digitalData |= (INT8U)(digitalRead(BUSTER_PIN_IN_2) << 2);
    digitalData |= (INT8U)(digitalRead(BUSTER_PIN_IN_3) << 3);
    digitalData |= (INT8U)(digitalRead(BUSTER_PIN_IN_4) << 4);
    digitalData |= (INT8U)(digitalRead(BUSTER_PIN_IN_5) << 5);
    digitalData |= (INT8U)(digitalRead(BUSTER_PIN_IN_6) << 6);
    digitalData |= (INT8U)(digitalRead(BUSTER_PIN_IN_7) << 7);

    // Envoyer uniquement si la valeur a changé
    if ((int)digitalData != prevDigital && BusTer_CanSendReady(BUSTER_Txb0)) {
        BusTer_CanSend(BUSTER_Txb0, BUSTER_Can2_0A, CAN_ID_DIGITAL | txAddress, 1, &digitalData);
        prevDigital = (int)digitalData;
    }

    // Émission analogique (uniquement si DIP8=1)
    if (analogMode) {
        int ain[4];
        ain[0] = BusTer_AnalogRead(BUSTER_ANALOGINPUT_0);
        ain[1] = BusTer_AnalogRead(BUSTER_ANALOGINPUT_1);
        ain[2] = BusTer_AnalogRead(BUSTER_ANALOGINPUT_2);
        ain[3] = BusTer_AnalogRead(BUSTER_ANALOGINPUT_3);

        bool changed = (ain[0] != prevAnalog[0]) || (ain[1] != prevAnalog[1]) ||
                       (ain[2] != prevAnalog[2]) || (ain[3] != prevAnalog[3]);

        if (changed && BusTer_CanSendReady(BUSTER_Txb1)) {
            INT8U analogBuf[8];
            for (int i = 0; i < 4; i++) {
                uint16_t val = (uint16_t)ain[i];
                analogBuf[i * 2]     = (INT8U)(val & 0xFF);
                analogBuf[i * 2 + 1] = (INT8U)((val >> 8) & 0xFF);
            }
            BusTer_CanSend(BUSTER_Txb1, BUSTER_Can2_0A, CAN_ID_ANALOG | txAddress, 8, analogBuf);

            prevAnalog[0] = ain[0];
            prevAnalog[1] = ain[1];
            prevAnalog[2] = ain[2];
            prevAnalog[3] = ain[3];
        }
    }
    
    // Réception digitale (RXB0)
    if (BusTer_CanMessageReceived(BUSTER_Rxb0)) {
        INT8U  rxBuf[8];
        INT8U  len;
        if (BusTer_CanRead(BUSTER_Rxb0, nullptr , &len, rxBuf) && len >= 1) {
            INT8U d = rxBuf[0];
            digitalWrite(BUSTER_PIN_OUT_0, (d >> 0) & 1);
            digitalWrite(BUSTER_PIN_OUT_1, (d >> 1) & 1);
            digitalWrite(BUSTER_PIN_OUT_2, (d >> 2) & 1);
            digitalWrite(BUSTER_PIN_OUT_3, (d >> 3) & 1);
            digitalWrite(BUSTER_PIN_OUT_4, (d >> 4) & 1);
            digitalWrite(BUSTER_PIN_OUT_5, (d >> 5) & 1);
            digitalWrite(BUSTER_PIN_OUT_6, (d >> 6) & 1);
            digitalWrite(BUSTER_PIN_OUT_7, (d >> 7) & 1);
        }
    }

    // Réception analogique (RXB1, uniquement si DIP8=1)
    if (analogMode && BusTer_CanMessageReceived(BUSTER_Rxb1)) {
        INT8U  len;
        INT8U  rxBuf[8];
        if (BusTer_CanRead(BUSTER_Rxb1, nullptr , &len, rxBuf) && len >= 8) {
            int v0 = (int)((uint16_t)rxBuf[0] | ((uint16_t)rxBuf[1] << 8));
            int v1 = (int)((uint16_t)rxBuf[2] | ((uint16_t)rxBuf[3] << 8));
            int v2 = (int)((uint16_t)rxBuf[4] | ((uint16_t)rxBuf[5] << 8));
            int v3 = (int)((uint16_t)rxBuf[6] | ((uint16_t)rxBuf[7] << 8));
            BusTer_AnalogWrite(v0, v1, v2, v3);
        }
    }

    // Gestion des erreurs CAN
    if (BusTer_CanHasError()) {
        digitalWrite(BUSTER_PIN_STATUS_LED_RED, HIGH);
        digitalWrite(BUSTER_PIN_STATUS_LED_GREEN, LOW);
        BusTer_CanClearError();
    } else {
        digitalWrite(BUSTER_PIN_STATUS_LED_RED, LOW);
        digitalWrite(BUSTER_PIN_STATUS_LED_GREEN, HIGH);
    }
}