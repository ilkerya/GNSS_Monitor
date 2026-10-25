/*
  GNSS_Detect.h  -  finds out which receiver is connected to mySerial and runs its driver

    1. u-blox  : UBX handshake at 38400 baud          -> GNSS_UBlox.h
    2. Teseo   : $PSTMGETSWVER / NMEA at 115200 baud  -> GNSS_ST.h

  Both modules use the same UART (mySerial) and the same RX/TX pins.
  If nothing answers, detection is retried every GNSS_RETRY_MS.
*/
#pragma once

#define GNSS_RETRY_MS 5000

static uint32_t gnssLastProbe = 0;

GnssModule GNSS_Detect() {
  Serial.println(F("Detecting GNSS module..."));

  if (UBlox_Detect()) {
    gnssModule = GNSS_UBLOX;
    UBlox_Setup();
  } else if (Teseo_Detect()) {
    gnssModule = GNSS_TESEO;
    Teseo_Setup();
  } else {
    gnssModule = GNSS_NONE;
    Serial.println(F("No GNSS module detected (u-blox @38400 / Teseo @115200)"));
  }
  gnssLastProbe = millis();
  return gnssModule;
}

// Call from loop()
void GNSS_Task() {
  switch (gnssModule) {
    case GNSS_UBLOX: UBlox_Task(); break;
    case GNSS_TESEO: Teseo_Task(); break;
    default:
      if (millis() - gnssLastProbe >= GNSS_RETRY_MS) {
        if (GNSS_Detect() != GNSS_NONE) NavSig_Init();   // redraw with the new badge
      }
      break;
  }
}
