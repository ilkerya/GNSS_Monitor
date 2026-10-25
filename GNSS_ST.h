/*
  GNSS_ST.h  -  ST Teseo-LIV4F driver (NMEA)

  The Teseo-LIV4F talks NMEA on its UART (default 115200 8N1). It is connected to
  the same port / RX-TX pins as the u-blox module (mySerial).

  Detection : send $PSTMGETSWVER,255 and wait for $PSTMVER, or for valid NMEA traffic
  Data      : $--GSV  -> one entry per satellite/signal (C/N0, signal ID)
              $--GSA  -> satellites used in the fix (marks the bar as "used")
              $--RMC / $--GGA -> UTC time of day
  An epoch is committed to the display when the UART has been quiet for
  TESEO_IDLE_MS after a burst that contained GSV sentences (the TFT is
  therefore only redrawn while no NMEA is arriving -> no RX overflow).

  Constellation / signal numbering is converted to the u-blox gnssId/sigId scheme,
  so the display and Serial output look the same as with the u-blox module.
*/
#pragma once

#define TESEO_BAUD          115200
#define TESEO_DETECT_MS     1500     // how long to listen during detection
#define TESEO_IDLE_MS       40       // quiet time that ends an NMEA burst
#define TESEO_LINE_MAX      120      // NMEA max is 82, ST proprietary can be longer
#define TESEO_MAX_USED      64

static char     tsLine[TESEO_LINE_MAX + 1];
static uint8_t  tsLen          = 0;
static uint32_t tsLastRx       = 0;
static bool     tsGsvSeen      = false;
static uint32_t tsTimeSec      = 0;
static char     tsVersion[48]  = "";

// satellites used in the fix (from GSA): gnssId << 8 | svId
static uint16_t tsUsed[TESEO_MAX_USED];
static uint8_t  tsUsedCount    = 0;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
static uint8_t Teseo_Hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;
}

// true if the line "$....*hh" has a valid checksum
static bool Teseo_ChecksumOk(const char *s) {
  if (s[0] != '$') return false;
  uint8_t cs = 0;
  const char *p = s + 1;
  while (*p && *p != '*') cs ^= (uint8_t)*p++;
  if (*p != '*') return false;
  uint8_t h = Teseo_Hex(p[1]), l = Teseo_Hex(p[2]);
  if (h == 0xFF || l == 0xFF) return false;
  return cs == ((h << 4) | l);
}

// Sends "$<body>*hh\r\n"
static void Teseo_SendCmd(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X\r\n", cs);
  mySerial.print('$');
  mySerial.print(body);
  mySerial.print(tail);
}

// Splits the sentence (without checksum) at ',' in place. Empty fields stay as "".
static uint8_t Teseo_Split(char *s, char **f, uint8_t maxF) {
  uint8_t n = 0;
  char *star = strchr(s, '*');
  if (star) *star = 0;
  f[n++] = s;
  for (char *p = s; *p && n < maxF; p++) {
    if (*p == ',') { *p = 0; f[n++] = p + 1; }
  }
  return n;
}

// "hhmmss.sss" -> seconds of day
static bool Teseo_ParseTime(const char *t, uint32_t *sec) {
  if (strlen(t) < 6) return false;
  for (uint8_t i = 0; i < 6; i++) if (t[i] < '0' || t[i] > '9') return false;
  *sec = ((t[0]-'0')*10 + (t[1]-'0')) * 3600UL +
         ((t[2]-'0')*10 + (t[3]-'0')) * 60UL +
         ((t[4]-'0')*10 + (t[5]-'0'));
  return true;
}

// Talker ID -> u-blox gnssId (255 = unknown, decide from PRN)
static uint8_t Teseo_TalkerToGnss(const char *talker) {
  if (talker[0] == 'G' && talker[1] == 'P') return 0;  // GPS (+SBAS)
  if (talker[0] == 'G' && talker[1] == 'L') return 6;  // GLONASS
  if (talker[0] == 'G' && talker[1] == 'A') return 2;  // Galileo
  if (talker[0] == 'B' && talker[1] == 'D') return 3;  // BeiDou
  if (talker[0] == 'G' && talker[1] == 'B') return 3;  // BeiDou (NMEA 4.1x)
  if (talker[0] == 'Q' && talker[1] == 'Z') return 5;  // QZSS
  if (talker[0] == 'G' && talker[1] == 'Q') return 5;  // QZSS (NMEA 4.11)
  if (talker[0] == 'G' && talker[1] == 'I') return 7;  // NavIC
  return 255;                                          // GN = multi-constellation
}

// Normalise PRN to (gnssId, svId) as used by u-blox NAV-SIG
static void Teseo_Classify(uint8_t hint, uint16_t prn, uint8_t *gnssId, uint8_t *svId) {
  uint8_t g = hint;
  if (g == 255 || g == 0) {                      // unknown, or GPS talker that may carry SBAS
    if      (prn >=   1 && prn <=  32) g = 0;    // GPS
    else if (prn >=  33 && prn <=  64) g = 1;    // SBAS (NMEA 33..64 = PRN 120..151)
    else if (prn >=  65 && prn <=  96) g = 6;    // GLONASS
    else if (prn >= 141 && prn <= 190) g = 3;    // BeiDou (ST numbering)
    else if (prn >= 193 && prn <= 202) g = 5;    // QZSS
    else if (prn >= 301 && prn <= 336) g = 2;    // Galileo (ST numbering)
    else if (prn >= 401 && prn <= 463) g = 3;    // BeiDou (alt. numbering)
    else g = (hint == 255) ? 0 : hint;
  }
  uint16_t sv = prn;
  switch (g) {
    case 1: if (prn >= 33 && prn <= 64) sv = prn + 87; break;     // -> 120..151
    case 2: if (prn > 300) sv = prn - 300; break;
    case 3: if (prn > 400) sv = prn - 400; else if (prn > 140) sv = prn - 140; break;
    case 5: if (prn >= 193) sv = prn - 192; break;
    case 6: if (prn >= 65) sv = prn - 64; break;
  }
  *gnssId = g;
  *svId   = (sv > 255) ? 255 : (uint8_t)sv;
}

// NMEA signal ID (hex digit, NMEA 4.10/4.11 + ST UM3009) -> u-blox sigId
static uint8_t Teseo_SigId(uint8_t gnssId, uint8_t nmeaSig) {
  switch (gnssId) {
    case 0:  // GPS : 1 L1C/A, 5 L2CM, 6 L2CL, 7 L5I, 8 L5Q
      switch (nmeaSig) { case 5: return 4; case 6: return 3; case 7: return 6; case 8: return 7; default: return 0; }
    case 2:  // Galileo : 1 E5a, 2 E5b, 6/7 E1 (ST reports E1 as 6)
      switch (nmeaSig) { case 1: return 3; case 2: return 5; default: return 0; }
    case 3:  // BeiDou : 1/2 B1I (ST reports 2), 3 B1C, 5 B2a, 0xB B2I
      switch (nmeaSig) { case 3: return 5; case 5: return 7; case 0xB: return 2; default: return 0; }
    case 5:  // QZSS : 1-3 L1, 4 L1S, 5 L2CM, 6 L2CL, 7 L5I, 8 L5Q
      switch (nmeaSig) { case 4: return 1; case 5: return 4; case 6: return 5; case 7: return 8; case 8: return 9; default: return 0; }
    case 6:  // GLONASS : 1 L1OF, 3 L2OF
      return (nmeaSig == 3) ? 2 : 0;
    default:
      return 0;
  }
}

static bool Teseo_IsUsed(uint8_t gnssId, uint8_t svId) {
  uint16_t key = ((uint16_t)gnssId << 8) | svId;
  for (uint8_t i = 0; i < tsUsedCount; i++) if (tsUsed[i] == key) return true;
  return false;
}

// ---------------------------------------------------------------------------
// sentence handlers
// ---------------------------------------------------------------------------
// $--GSV,numMsg,msgNum,numSats,{prn,elev,az,cno}x1..4[,signalId]
static void Teseo_HandleGSV(const char *talker, char **f, uint8_t n) {
  if (n < 4) return;
  uint8_t hint   = Teseo_TalkerToGnss(talker);
  uint8_t groups = (n - 4) / 4;
  uint8_t sig    = 1;                                   // no signal ID field -> L1
  if (((n - 4) % 4) == 1 && f[n - 1][0]) sig = Teseo_Hex(f[n - 1][0]);

  for (uint8_t g = 0; g < groups; g++) {
    const char *prnS = f[4 + g * 4];
    const char *cnoS = f[4 + g * 4 + 3];
    if (!prnS[0]) continue;
    uint8_t gnssId, svId;
    Teseo_Classify(hint, (uint16_t)atoi(prnS), &gnssId, &svId);
    uint8_t cno = cnoS[0] ? (uint8_t)atoi(cnoS) : 0;   // empty = not tracked
    NavSig_Add(gnssId, Teseo_SigId(gnssId, sig), svId, cno, false);
  }
  tsGsvSeen = true;
}

// $--GSA,mode,fix,prn1..prn12,pdop,hdop,vdop[,systemId]
static void Teseo_HandleGSA(const char *talker, char **f, uint8_t n) {
  if (n < 15) return;
  uint8_t hint = Teseo_TalkerToGnss(talker);
  if (n >= 19 && f[18][0]) {                            // ST: 1 GPS, 2 GLO, 3 GAL, 4 BDS, 5 QZSS
    switch (atoi(f[18])) {
      case 1: hint = 0; break;  case 2: hint = 6; break;  case 3: hint = 2; break;
      case 4: hint = 3; break;  case 5: hint = 5; break;  case 6: hint = 7; break;
    }
  }
  for (uint8_t i = 3; i < 15; i++) {
    if (!f[i][0] || tsUsedCount >= TESEO_MAX_USED) continue;
    uint8_t gnssId, svId;
    Teseo_Classify(hint, (uint16_t)atoi(f[i]), &gnssId, &svId);
    tsUsed[tsUsedCount++] = ((uint16_t)gnssId << 8) | svId;
  }
}

static void Teseo_HandleLine(char *line) {
  if (!Teseo_ChecksumOk(line)) return;

  if (strncmp(line, "$PSTMVER,", 9) == 0) {             // version reply
    char *end = strchr(line, '*');
    if (end) *end = 0;
    strncpy(tsVersion, line + 9, sizeof(tsVersion) - 1);
    return;
  }
  if (line[0] != '$' || strlen(line) < 7) return;

  char talker[3] = { line[1], line[2], 0 };
  const char *type = line + 3;
  char *f[40];
  uint8_t n = Teseo_Split(line, f, 40);

  if      (strncmp(type, "GSV", 3) == 0) Teseo_HandleGSV(talker, f, n);
  else if (strncmp(type, "GSA", 3) == 0) Teseo_HandleGSA(talker, f, n);
  else if (strncmp(type, "RMC", 3) == 0 || strncmp(type, "GGA", 3) == 0) {
    if (n > 1) Teseo_ParseTime(f[1], &tsTimeSec);
  }
}

// End of an NMEA burst: apply "used" flags, publish to display + Serial
static void Teseo_CommitEpoch() {
  for (uint8_t i = 0; i < NavSig_WorkCount(); i++) {
    NavSigEntry *e = NavSig_WorkEntry(i);
    e->used = Teseo_IsUsed(e->gnssId, e->svId);
  }
  NavSig_Commit(tsTimeSec, true);
  NavSig_PrintSerial("NMEA GSV");
  NavSig_Clear();
  tsUsedCount = 0;
  tsGsvSeen   = false;
}

// Reads all available bytes; returns true if at least one byte was received
static bool Teseo_ReadSerial() {
  bool got = false;
  while (mySerial.available()) {
    char c = mySerial.read();
    got = true;
    if (c == '$') tsLen = 0;                            // resync on every sentence start
    if (c == '\r' || c == '\n') {
      if (tsLen) { tsLine[tsLen] = 0; Teseo_HandleLine(tsLine); tsLen = 0; }
    } else if (tsLen < TESEO_LINE_MAX) {
      tsLine[tsLen++] = c;
    } else {
      tsLen = 0;                                        // overlong line -> drop
    }
  }
  if (got) tsLastRx = millis();
  return got;
}

// ---------------------------------------------------------------------------
// public API (called from GNSS_Detect.h / loop)
// ---------------------------------------------------------------------------

// Probe for a Teseo on mySerial. Returns true if a Teseo (or any NMEA receiver) answers.
bool Teseo_Detect() {
  mySerial.end();
  mySerial.begin(TESEO_BAUD);
  delay(20);
  while (mySerial.available()) mySerial.read();

  tsVersion[0] = 0;
  Teseo_SendCmd("PSTMGETSWVER,255");                    // -> $PSTMVER,...

  uint16_t validNmea = 0;
  uint32_t t0 = millis();
  tsLen = 0;
  while (millis() - t0 < TESEO_DETECT_MS) {
    while (mySerial.available()) {
      char c = mySerial.read();
      if (c == '$') tsLen = 0;
      if (c == '\r' || c == '\n') {
        if (tsLen) {
          tsLine[tsLen] = 0;
          if (Teseo_ChecksumOk(tsLine)) {
            validNmea++;
            Teseo_HandleLine(tsLine);                   // picks up $PSTMVER
          }
          tsLen = 0;
        }
      } else if (tsLen < TESEO_LINE_MAX) tsLine[tsLen++] = c;
      else tsLen = 0;
    }
    if (tsVersion[0] && validNmea > 1) break;           // got the version reply, done
  }
  NavSig_Clear();                                       // drop anything parsed while probing
  tsUsedCount = 0;
  tsGsvSeen   = false;
  return tsVersion[0] || validNmea >= 2;
}

void Teseo_Setup() {
  Serial.println(F("ST Teseo GNSS ok...."));
  if (tsVersion[0]) {
    Serial.print(F("PSTMVER: "));
    Serial.println(tsVersion);
  } else {
    Serial.println(F("No $PSTMVER reply - generic NMEA receiver assumed"));
  }
  // The LIV4F default NMEA set includes GGA/RMC/GSA/GSV. Nothing is written to
  // its configuration here, so the module settings stay untouched.
  tsLastRx = millis();
}

// Call from loop() while the Teseo driver is active
void Teseo_Task() {
  Teseo_ReadSerial();
  if (tsGsvSeen && (millis() - tsLastRx) >= TESEO_IDLE_MS) Teseo_CommitEpoch();
}
