void newSIG(UBX_NAV_SIG_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("New NAV SIG data received. It contains "));
  Serial.print(ubxDataStruct->header.numSigs); // Print numSigs (Number of signals / blocks)
  Serial.println(F(" data blocks:"));

  for (uint8_t block = 0; block < ubxDataStruct->header.numSigs; block++) // For each block
  {
    char SV[14 + 4 + 4 + 256 + 1]; // Allocate space for sigId plus svId plus cno plus bars plus NULL
    switch (ubxDataStruct->blocks[block].gnssId)
    {
      case 0:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "GPS L1 C/A    ");
              break;
            case 3:
              sprintf(SV, "GPS L2 CL     ");
              break;
            case 4:
              sprintf(SV, "GPS L2 CM     ");
              break;
            case 6:
              sprintf(SV, "GPS L5 I      ");
              break;
            case 7:
              sprintf(SV, "GPS L5 Q      ");
              break;
            default:
              sprintf(SV, "GPS Unknown   ");
              break;
          }
        }
        break;
      case 1:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "SBAS L1 C/A   ");
              break;
            default:
              sprintf(SV, "SBAS Unknown  ");
              break;
          }
        }
        break;
      case 2:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "Galileo E1 C  ");
              break;
            case 1:
              sprintf(SV, "Galileo E1 B  ");
              break;
            case 3:
              sprintf(SV, "Galileo E5 aI ");
              break;
            case 4:
              sprintf(SV, "Galileo E5 aQ ");
              break;
            case 5:
              sprintf(SV, "Galileo E5 bI ");
              break;
            case 6:
              sprintf(SV, "Galileo E5 bQ ");
              break;
            default:
              sprintf(SV, "GAL Unknown   ");
              break;
          }
        }
        break;
      case 3:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "BeiDou B1I D1 ");
              break;
            case 1:
              sprintf(SV, "BeiDou B1I D2 ");
              break;
            case 2:
              sprintf(SV, "BeiDou B2I D1 ");
              break;
            case 3:
              sprintf(SV, "BeiDou B2I D2 ");
              break;
            case 5:
              sprintf(SV, "BeiDou B1 Cp  ");
              break;
            case 6:
              sprintf(SV, "BeiDou B2 Cd  ");
              break;
            case 7:
              sprintf(SV, "BeiDou B2 ap  ");
              break;
            case 8:
              sprintf(SV, "BeiDou B2 ad  ");
              break;
            default:
              sprintf(SV, "BDS Unknown   ");
              break;
          }
        }
        break;
      case 5:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "QZSS L1 C/A   ");
              break;
            case 1:
              sprintf(SV, "QZSS L1S      ");
              break;
            case 4:
              sprintf(SV, "QZSS L2 CM    ");
              break;
            case 5:
              sprintf(SV, "QZSS L2 CL    ");
              break;
            case 8:
              sprintf(SV, "QZSS L5 I     ");
              break;
            case 9:
              sprintf(SV, "QZSS L5 Q     ");
              break;
            default:
              sprintf(SV, "QZSS Unknown  ");
              break;
          }
        }
        break;
      case 6:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "GLONASS L1 OF ");
              break;
            case 2:
              sprintf(SV, "GLONASS L2 OF ");
              break;
            default:
              sprintf(SV, "GLO Unknown   ");
              break;
          }
        }
        break;
      case 7:
        {
          switch (ubxDataStruct->blocks[block].sigId)
          {
            case 0:
              sprintf(SV, "NavIC L5 A    ");
              break;
            default:
              sprintf(SV, "NavIC Unknown ");
              break;
          }
        }
        break;
      default:
        sprintf(SV, "Unknown       ");
        break;
    }

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
void GNSS_Setup(){
  Serial.println("SparkFun u-blox Example");
  
  mySerial.begin(38400); // The NEO-F10N defaults to 38400 baud

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
/*
  while (myGNSS.begin(mySerial) == false) //Connect to the u-blox module using mySerial (UART)
  {
    Serial.println(F("u-blox GNSS not detected. Please check wiring. Retrying..."));
    delay(1000);
  }
  */
  if(myGNSS.begin(mySerial) == false) 
      Serial.println(F("u-blox GNSS not detected...."));
    else   Serial.println(F("u-blox GNSS ok...."));
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
