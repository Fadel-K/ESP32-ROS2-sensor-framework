#include <ACAN_ESP32.h>

#define LED_BUILTIN 38

void setup () {
  Serial.begin (9600) ;
  Serial.println ("Hello") ;
  ACAN_ESP32_Settings settings (125 * 1000) ; // 125 kbit/s
  settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode ;
  const uint32_t errorCode = ACAN_ESP32::can.begin (settings) ;
  if (0 == errorCode) {
    Serial.println ("Can ok") ;
  }else{
    Serial.print ("Error Can: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;


void loop () {
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 500 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    const bool ok = ACAN_ESP32::can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
  }
  while (ACAN_ESP32::can.receive (frame)) {
    gReceivedFrameCount += 1 ;
  }
}