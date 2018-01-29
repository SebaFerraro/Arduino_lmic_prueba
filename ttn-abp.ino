/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t temprature_sens_read();
int hallRead();

//uint8_t g_phyFuns;

#ifdef __cplusplus
}
#endif

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60      /* Time ESP32 will go to sleep (in seconds) */
#define KEEPALIVE  10

// The LORAWAN session state is saved in EEPROM
// Here just the frame sequence number is saved as its ABP with a Single Channel Gateway,
// See http://forum.thethingsnetwork.org/t/new-backend-how-to-connect/1983/91 
#define INITED_FLAG 0x12344321 // <<<< Change this to have the frame sequence restart from zero  
typedef struct {
  uint32_t inited;
  u4_t seqnoUp;
} LORA_STATE;
//LORA_STATE loraState;

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x61, 0xF2, 0x1E, 0x6E, 0xE3, 0x00, 0x1C, 0x4C, 0x21, 0x69, 0x52, 0x54, 0x23, 0x9B, 0x05, 0xB7 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0xC4, 0x4F, 0xB3, 0x42, 0x4F, 0x17, 0xDF, 0x51, 0x96, 0x80, 0x34, 0x22, 0x64, 0xBA, 0x9F, 0xC2 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x2601163F; // <-- Change this address for every node!
static const uint8_t CHANNELSEL = 0;
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata [4] = {0};
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 21,
    .dio = {16, 17, 22},
};


RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int temp = 0;
RTC_DATA_ATTR int hall = 0;
RTC_DATA_ATTR LORA_STATE loraState;
/*
Method to print the reason by which ESP32
has been awaken from sleep
*/

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}

float temperatureRead(void)
{
     return (temprature_sens_read() - 32) / 1.8;
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        Serial.println("Envia datos mydata:" + String(mydata[0]) + String(mydata[1]) + String(mydata[2]) + String(mydata[3]));
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
        Serial.println(LMIC.freq);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
    Serial.println(F("LMIC"));
    if (loraState.inited != INITED_FLAG) {
     loraState.inited = INITED_FLAG;
     loraState.seqnoUp = 0;
    }
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    Serial.println(F("os_init"));
    LMIC_reset();
    Serial.println(F("LMIC_Reset"));
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
    Serial.println(F("LMIC_setSession"));
    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(0);
    //Disable FSB1, channels 0-7
    for (int i = 0; i < 64; i++) {
     if (i != CHANNELSEL){
       LMIC_disableChannel(i);
       Serial.print("LMIC disableChannel");
       Serial.println(i);
     } 
    //}
    //Disable FSB2-8, channels 16-72
    //for (int i = 16; i < 73; i++) {
    //    if (i != 10)
    //    LMIC_disableChannel(i);
//  
     }
    #endif
    LMIC_disableTracking();
    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,20);
    LMIC.seqnoUp = loraState.seqnoUp;

    // Let LMIC compensate for +/- 1% clock error
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    int t=(int)temperatureRead();
    int h= hallRead();
    //String vcc = String(ESP.getVcc());
    //Serial.println("VCC:" + vcc);
    Serial.println("Temp:" + String(t) + " Hall:"+ String(h));
    if (t != temp || bootCount == KEEPALIVE ){
      temp=t;
      hall=h;
      Serial.println("Envia datos temp:" + String(t) + " Hall:"+ String(h));
      mydata[0]=(uint8_t)((temp>>8) & 0b11111111);
      mydata[1]=(uint8_t)(temp & 0b11111111);
      mydata[2]=(uint8_t)((hall>>8) & 0b11111111);
      mydata[3]=(uint8_t)(hall & 0b11111111);
      do_send(&sendjob);
      if (bootCount == KEEPALIVE){
       Serial.println("BootCount:" + String(bootCount) + " KEEPALIVE");
       }
      bootCount=0;
     
      Serial.println(String(LMIC.txCnt));  
    }else {
       bootCount++;
       Serial.println("BootCount:" + String(bootCount));  
    }
      
    // Start job
    
    //Print the wakeup reason for ESP32
    print_wakeup_reason();

     /*
     First we configure the wake up source
     We set our ESP32 to wake up every 5 seconds
     */
     esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
     Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
     " Seconds");

     /*
     Next we decide what all peripherals to shut down/keep on
     By default, ESP32 will automatically power down the peripherals
     not needed by the wakeup source, but if you want to be a poweruser
     this is for you. Read in detail at the API docs
     http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
     Left the line commented as an example of how to configure peripherals.
     The line below turns off all RTC peripherals in deep sleep.
     */
     //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
     //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

     /*
     Now that we have setup a wake cause and if needed setup the
     peripherals state in deep sleep, we can now start going to
     deep sleep.
     In the case that no wake up sources were provided but deep
     sleep was started, it will sleep forever unless hardware
     reset occurs.
     */
     //Serial.println("Going to sleep now");
     //esp_deep_sleep_start();
     //Serial.println("This will never be printed");
}

void loop() {
    os_runloop_once();
    //do_send();
    delay(2000);
    loraState.seqnoUp = LMIC.seqnoUp;
    Serial.print("loraState.seqnoUp = "); Serial.println(loraState.seqnoUp);    
    Serial.print("Up time "); Serial.print(millis());
    
    os_radio(RADIO_RST);
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
}
