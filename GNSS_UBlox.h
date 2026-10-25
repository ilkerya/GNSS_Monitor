// Returns the 14-character (space padded) signal name used on Serial and on the TFT
const char* GNSS_SignalName(uint8_t gnssId, uint8_t sigId)
{
    switch (gnssId)
    {
      case 0:
        {
          switch (sigId)
          {
            case 0:
              return "GPS L1 C/A    ";
            case 3:
              return "GPS L2 CL     ";
            case 4:
              return "GPS L2 CM     ";
            case 6:
              return "GPS L5 I      ";
            case 7:
              return "GPS L5 Q      ";
            default:
              return "GPS Unknown   ";
          }
        }
      case 1:
        {
          switch (sigId)
          {
            case 0:
              return "SBAS L1 C/A   ";
            default:
              return "SBAS Unknown  ";
          }
        }
      case 2:
        {
          switch (sigId)
          {
            case 0:
              return "Galileo E1 C  ";
            case 1:
              return "Galileo E1 B  ";
            case 3:
              return "Galileo E5 aI ";
            case 4:
              return "Galileo E5 aQ ";
            case 5:
              return "Galileo E5 bI ";
            case 6:
              return "Galileo E5 bQ ";
            default:
              return "GAL Unknown   ";
          }
        }
      case 3:
        {
          switch (sigId)
          {
            case 0:
              return "BeiDou B1I D1 ";
            case 1:
              return "BeiDou B1I D2 ";
            case 2:
              return "BeiDou B2I D1 ";
            case 3:
              return "BeiDou B2I D2 ";
            case 5:
              return "BeiDou B1 Cp  ";
            case 6:
              return "BeiDou B2 Cd  ";
            case 7:
              return "BeiDou B2 ap  ";
            case 8:
              return "BeiDou B2 ad  ";
            default:
              return "BDS Unknown   ";
          }
        }
      case 5:
        {
          switch (sigId)
          {
            case 0:
              return "QZSS L1 C/A   ";
            case 1:
              return "QZSS L1S      ";
            case 4:
              return "QZSS L2 CM    ";
            case 5:
              return "QZSS L2 CL    ";
            case 8:
              return "QZSS L5 I     ";
            case 9:
              return "QZSS L5 Q     ";
            default:
              return "QZSS Unknown  ";
          }
        }
      case 6:
        {
          switch (sigId)
          {
            case 0:
              return "GLONASS L1 OF ";
            case 2:
              return "GLONASS L2 OF ";
            default:
              return "GLO Unknown   ";
          }
        }
      case 7:
        {
          switch (sigId)
          {
            case 0:
              return "NavIC L5 A    ";
            default:
              return "NavIC Unknown ";
          }
        }
      default:
        return "Unknown       ";
    }

}

void newSIG(UBX_NAV_SIG_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("New NAV SIG data received. It contains "));
  Serial.print(ubxDataStruct->header.numSigs); // Print numSigs (Number of signals / blocks)
  Serial.println(F(" data blocks:"));

  NavSig_Store(ubxDataStruct); // hand a copy to the TFT (drawn later from loop())

  uint8_t numBlocks = ubxDataStruct->header.numSigs;
  if (numBlocks > UBX_NAV_SIG_MAX_BLOCKS) numBlocks = UBX_NAV_SIG_MAX_BLOCKS; // the library stores max 92 blocks

  for (uint8_t block = 0; block < numBlocks; block++) // For each block
  {
    char SV[14 + 4 + 4 + 256 + 1]; // Allocate space for sigId plus svId plus cno plus bars plus NULL
    strcpy(SV, GNSS_SignalName(ubxDataStruct->blocks[block].gnssId, ubxDataStruct->blocks[block].sigId));

  if (ubxDataStruct->blocks[block].svId < 10) sprintf(&SV[14], "%d   ", ubxDataStruct->blocks[block].svId); // Align the svId
  else if (ubxDataStruct->blocks[block].svId < 100) sprintf(&SV[14], "%d  ", ubxDataStruct->blocks[block].svId); // Align the svId
  else sprintf(&SV[14], "%d ", ubxDataStruct->blocks[block].svId); // Align the svId

  if (ubxDataStruct->blocks[block].cno < 10) sprintf(&SV[18], "  %d ", ubxDataStruct->blocks[block].cno); // Align the cno
  else if (ubxDataStruct->blocks[block].cno < 100) sprintf(&SV[18], " %d ", ubxDataStruct->blocks[block].cno); // Align the cno
  else sprintf(&SV[18], "%d ", ubxDataStruct->blocks[block].cno); // Align the cno

  // Print cno as a bar chart
  uint8_t cno;
  for (cno = 0; cno <= ubxDataStruct->blocks[block].cno; cno++)
    SV[22 + cno] = '=';
  SV[22 + cno] = 0; // NULL

  Serial.println(SV);
  }
}
// ---------------------------------------------------------------------------
// u-blox driver: detection + configuration (called from GNSS_Detect.h)
// ---------------------------------------------------------------------------

// Probe for a u-blox receiver on mySerial at 38400 baud (NEO-F10N default).
bool UBlox_Detect() {
  mySerial.begin(38400); // The NEO-F10N defaults to 38400 baud
  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  return myGNSS.begin(mySerial);
}

// Configure the detected u-blox receiver and enable NAV-SIG with callback.
void UBlox_Setup() {
  Serial.println(F("u-blox GNSS ok...."));
  if (myGNSS.getModuleInfo())
  {
    Serial.print(F("FWVER: "));
    Serial.print(myGNSS.getFirmwareVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getFirmwareVersionLow()); // Returns uint8_t
    
    Serial.print(F("Firmware: "));
    Serial.println(myGNSS.getFirmwareType()); // Returns HPG, SPG, SPGL1L5 etc. as (const char *)

    Serial.print(F("PROTVER: "));
    Serial.print(myGNSS.getProtocolVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getProtocolVersionLow()); // Returns uint8_t
    
    Serial.print(F("MOD: "));
    Serial.println(myGNSS.getModuleName()); // Returns ZED-F9P, MAX-M10S etc. as (const char *)
  }
  else
      Serial.println(F("Error: could not read module info!"));

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setMeasurementRate(5000); //Produce one solution every five seconds (NAV SIG produces a _lot_ of data!)

  myGNSS.setVal8(UBLOX_CFG_SIGNAL_GPS_L5_ENA, 1); // Make sure the GPS L5 band is enabled (needed on the NEO-F9P)

  myGNSS.setGPSL5HealthOverride(true); // Mark L5 signals as healthy - store in RAM and BBR

  myGNSS.setLNAMode(SFE_UBLOX_LNA_MODE_NORMAL); // Set the LNA gain to normal (full). Other options: LOWGAIN, BYPASS

  myGNSS.softwareResetGNSSOnly(); // Restart the GNSS to apply the L5 health override

  myGNSS.setAutoNAVSIGcallbackPtr(&newSIG); // Enable automatic NAV SIG messages with callback to newSIG
}

// Call from loop() while the u-blox driver is active
void UBlox_Task() {
  myGNSS.checkUblox();     // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed (-> newSIG -> NavSig_Store)
}
