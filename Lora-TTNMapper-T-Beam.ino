#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"
#include "gps.h"
#include "CayenneLPP.h"

//Cayenne Code. Si la salida de datos a TTN va a ser en formato CayenneLPP ,
//CAYENNELPP_OUT_DATA 1 , si se va a decodificar personalmente poner a 0
#define CAYENNELPP_OUT_DATA 1

//#if CAYENNELPP_OUT_DATA == 1
//#include "CayenneLPP.h"
//#endif

// T-Beam specific hardware
#define BUILTIN_LED 21

//Other configs
#define TX_INTERVAL_SEC 90 //Intervalo de transmision Tx (tener en cuenta el duty cicle)

char s[32]; // used to sprintf for Serial output
uint8_t txBuffer[9];
//#if CAYENNELPP_OUT_DATA == 1
uint8_t lppBuffer[11]; //buffer para cayennelpp gps payload
//#endif

gps gps; //instanciado de la clase gps
CayenneLPP lpp(51); //instanciado de la clase CayenneLPP, el size=51 aun no se que es

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = TX_INTERVAL_SEC;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN, // was "14,"
  .dio = {26, 33, 32},
};

void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    if (gps.checkGpsFix())
    {
      // Prepare upstream data transmission at the next possible time.

      //Preparacion de paquete CayenneLPP
     
      #if CAYENNELPP_OUT_DATA == 1
      Serial.println(F("Tipo dato de salida: CayenneLPP"));
      Serial.println(F("Reset buffer serial GPS"));
      lpp.reset();
      Serial.println(F("Preparacion del paquete"));
      //lpp.addGPS(1,39.56715,-0.28117,2);  //prueba debug
      float LatitudeBinaryC = gps.getLatitude();
      float LongitudeBinaryC = gps.getLongitude();
      float altitudeGpsC = gps.getAltitude();
      float hdopGpsC;
      gps.buildClppPacket(); //creacion del paquete CayenneLPP
      Serial.println("**********Paquete LPP construido************");
      Serial.println(LatitudeBinaryC);
      Serial.println(LongitudeBinaryC);
      Serial.println(altitudeGpsC);
      Serial.println(hdopGpsC);
      Serial.println("*********************************************");
      lpp.addGPS(1, LatitudeBinaryC, LongitudeBinaryC, altitudeGpsC); //[canal de CayenneAPI][latitud][longitud][altura]
      Serial.println(F("Conversion a CayenneLPP"));
      lpp.copy(lppBuffer); //problema aqui, creo q por el puntero * puntero quitado
      Serial.println(F("Integracion CayenneLPP en LoRaWan"));
      LMIC_setTxData2(1, lppBuffer, sizeof(lppBuffer), 0);
      Serial.println(F("CayenneLPP Packet queued"));
      digitalWrite(BUILTIN_LED, HIGH);
      #endif

     //Preparacion del paquete TTN
      #if CAYENNELPP_OUT_DATA == 0
      Serial.println(F("Tipo dato de salida: TTN"));
      gps.buildPacket(txBuffer);
      LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
      Serial.println(F("TTN Packet queued"));
      digitalWrite(BUILTIN_LED, HIGH);
      #endif

    }
    else
    {
      //try again in 3 seconds
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("TTN Mapper"));

  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);  //no funciona , error wifi_off not declared in scope
  btStop();
  gps.init();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  //Hackeada la lista de canales y los valores de SF para single channel
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF9, DR_SF9),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF9, DR_SF9),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 868100000, DR_RANGE_MAP(DR_SF9, DR_SF9),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 868100000, DR_RANGE_MAP(DR_SF9, DR_SF9),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 868100000, DR_RANGE_MAP(DR_SF9, DR_SF9),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 868100000, DR_RANGE_MAP(DR_SF9, DR_SF9),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 868100000, DR_RANGE_MAP(DR_SF9, DR_SF9),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868100000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9,14);

  do_send(&sendjob);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

}

void loop() {
    os_runloop_once();


}
