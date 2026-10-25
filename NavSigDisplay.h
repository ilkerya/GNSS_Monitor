/*
  NavSigDisplay.h
  Shows the GNSS signal list (same content as the Serial Monitor output)
  on the Arduino GIGA Display Shield (800x480, landscape).
  Works with both receivers - whichever one was detected at start-up:
    u-blox (UBX-NAV-SIG)    -> GNSS_UBlox.h
    ST Teseo-LIV4F (NMEA)   -> GNSS_ST.h
  The active module is highlighted with a filled badge in the title bar.

  Layout
    y   0..19   title line   : module badges [u-blox] [ST Teseo], signal count, time, data age
    y  20..39   summary line : signals per constellation + number used in the fix
    y  42..57   column header: Signal / SV / C/N0
    y  60..479  signal rows  : 2 columns x 26 rows = 52 signals per page
                               (if there are more, the pages rotate every 2.5 s)

  Each row: "<signal name> <svId> <cno>"  + a C/N0 bar (2 px per dB-Hz).
    Name colour  = constellation
    Bar colour   = signal strength (green >= 40, yellow >= 30, orange >= 20, red > 0)
    Filled bar   = pseudorange used in the navigation solution, outline = tracked only

  Data flow
    u-blox: newSIG() (UBX callback)  -> NavSig_Store()
    Teseo : NMEA GSV/GSA parser      -> NavSig_Clear() / NavSig_Add() / NavSig_Commit()
    loop()                           -> NavSig_Task()   redraws when new data arrived, or on the 0.5 s tick

  Drawing never happens inside the callback, and the full redraw is done with
  startBuffering()/endBuffering() so the screen does not flicker.
*/
#pragma once

// Implemented in GNSS_UBlox.h - 14-character signal name, u-blox gnssId/sigId numbering
const char* GNSS_SignalName(uint8_t gnssId, uint8_t sigId);

// Which receiver was detected at start-up
enum GnssModule : uint8_t { GNSS_NONE = 0, GNSS_UBLOX, GNSS_TESEO };
GnssModule gnssModule = GNSS_NONE;   // set by GNSS_Detect()

// ---------- colours (RGB565) ----------
#define NS_BLACK    0x0000
#define NS_WHITE    0xFFFF
#define NS_GREY     0x8410
#define NS_DGREY    0x2104
#define NS_RED      0xF800
#define NS_ORANGE   0xFD20
#define NS_YELLOW   0xFFE0
#define NS_GREEN    0x07E0
#define NS_CYAN     0x07FF
#define NS_BLUE     0x441F   // lighter blue, readable on black
#define NS_MAGENTA  0xF81F
#define NS_HEADERBG 0x0010   // dark navy

// ---------- layout ----------
#define NS_CHAR_W        12   // text size 2 -> 12 x 16 px per character
#define NS_ROW_H         16
#define NS_TOP_ROWS_Y    60
#define NS_COLS          2
#define NS_COL_W         400
#define NS_ROWS_PER_COL  ((480 - NS_TOP_ROWS_Y) / NS_ROW_H)   // 26
#define NS_PER_PAGE      (NS_COLS * NS_ROWS_PER_COL)          // 52
#define NS_BAR_X         (22 * NS_CHAR_W + 4)                 // after "name(14) sv(4) cno(4)"
#define NS_BAR_MAX_W     (NS_COL_W - NS_BAR_X - 8)
#define NS_MAX_SIGS      UBX_NAV_SIG_MAX_BLOCKS

// ---------- snapshot of the last NAV-SIG message ----------
struct NavSigEntry {
  uint8_t gnssId;
  uint8_t sigId;
  uint8_t svId;
  uint8_t cno;
  bool    used;   // pseudorange used in the fix
};

static NavSigEntry   nsSig[NS_MAX_SIGS];
static uint8_t       nsCount      = 0;
static uint8_t       nsReported   = 0;      // numSigs as reported by the receiver
static uint32_t      nsTimeSec    = 0;      // seconds of day
static bool          nsTimeUtc    = false;  // false: GPS time (u-blox iTOW), true: UTC (NMEA)
static uint32_t      nsLastMillis = 0;
static bool          nsHaveData   = false;
static volatile bool nsNewData    = false;
static uint8_t       nsPage       = 0;
static uint8_t       nsTick       = 0;

// ---------- helpers ----------
static uint16_t NavSig_ConstColor(uint8_t gnssId) {
  switch (gnssId) {
    case 0:  return NS_GREEN;    // GPS
    case 1:  return NS_GREY;     // SBAS
    case 2:  return NS_CYAN;     // Galileo
    case 3:  return NS_RED;      // BeiDou
    case 5:  return NS_MAGENTA;  // QZSS
    case 6:  return NS_YELLOW;   // GLONASS
    case 7:  return NS_ORANGE;   // NavIC
    default: return NS_WHITE;
  }
}

static uint16_t NavSig_CnoColor(uint8_t cno) {
  if (cno >= 40) return NS_GREEN;
  if (cno >= 30) return NS_YELLOW;
  if (cno >= 20) return NS_ORANGE;
  if (cno > 0)   return NS_RED;
  return NS_DGREY;
}

// ---------- generic snapshot API (used by the Teseo NMEA driver) ----------
static NavSigEntry nsWork[NS_MAX_SIGS];   // filled while an epoch is being received
static uint8_t     nsWorkCount = 0;

void NavSig_Clear() { nsWorkCount = 0; }

// returns false if the table is full
bool NavSig_Add(uint8_t gnssId, uint8_t sigId, uint8_t svId, uint8_t cno, bool used) {
  if (nsWorkCount >= NS_MAX_SIGS) return false;
  nsWork[nsWorkCount++] = { gnssId, sigId, svId, cno, used };
  return true;
}

NavSigEntry* NavSig_WorkEntry(uint8_t i) { return (i < nsWorkCount) ? &nsWork[i] : nullptr; }
uint8_t      NavSig_WorkCount()          { return nsWorkCount; }

void NavSig_Commit(uint32_t timeSec, bool utc) {
  memcpy(nsSig, nsWork, sizeof(NavSigEntry) * nsWorkCount);
  nsCount      = nsWorkCount;
  nsReported   = nsWorkCount;
  nsTimeSec    = timeSec;
  nsTimeUtc    = utc;
  nsLastMillis = millis();
  nsHaveData   = true;
  nsNewData    = true;
}

// Prints the current snapshot in the same format as the u-blox newSIG() output
void NavSig_PrintSerial(const char *source) {
  Serial.println();
  Serial.print(F("New "));
  Serial.print(source);
  Serial.print(F(" data received. It contains "));
  Serial.print(nsCount);
  Serial.println(F(" data blocks:"));
  char line[14 + 4 + 4 + 100 + 1];
  for (uint8_t i = 0; i < nsCount; i++) {
    snprintf(line, 23, "%s%-4u%3u ", GNSS_SignalName(nsSig[i].gnssId, nsSig[i].sigId),
             nsSig[i].svId, nsSig[i].cno);
    uint8_t c = nsSig[i].cno > 99 ? 99 : nsSig[i].cno;
    uint8_t k;
    for (k = 0; k <= c; k++) line[22 + k] = '=';
    line[22 + k] = 0;
    Serial.println(line);
  }
}

// Called from the UBX callback: copy only what we need, no drawing here.
void NavSig_Store(UBX_NAV_SIG_data_t *d) {
  uint8_t n = d->header.numSigs;
  if (n > NS_MAX_SIGS) n = NS_MAX_SIGS;
  for (uint8_t i = 0; i < n; i++) {
    nsSig[i].gnssId = d->blocks[i].gnssId;
    nsSig[i].sigId  = d->blocks[i].sigId;
    nsSig[i].svId   = d->blocks[i].svId;
    nsSig[i].cno    = d->blocks[i].cno;
    // u-blox spec: sigFlags bits 1..0 = health, bit 2 = prSmoothed, bit 3 = prUsed
    nsSig[i].used   = (d->blocks[i].sigFlags.all >> 3) & 0x01;
  }
  nsCount      = n;
  nsReported   = d->header.numSigs;
  nsTimeSec    = (d->header.iTOW / 1000UL) % 86400UL;   // GPS time of day
  nsTimeUtc    = false;
  nsLastMillis = millis();
  nsHaveData   = true;
  nsNewData    = true;
}

// ---------- drawing ----------
#define NS_STATUS_X 392

// Module badge: filled + bold colour when active, grey outline when not
static int16_t NavSig_DrawBadge(int16_t x, const char *text, uint16_t color, bool active) {
  int16_t w = strlen(text) * NS_CHAR_W + 8;
  if (active) {
    tft.fillRoundRect(x, 1, w, 18, 4, color);
    tft.setTextColor(NS_BLACK);
  } else {
    tft.drawRoundRect(x, 1, w, 18, 4, NS_GREY);
    tft.setTextColor(NS_GREY);
  }
  tft.setCursor(x + 4, 2);
  tft.print(text);
  return x + w + 4;
}

static void NavSig_DrawStatus() {
  // Right part of the title line: time of day + age of the data
  tft.fillRect(NS_STATUS_X - 6, 0, 800 - NS_STATUS_X + 6, 20, NS_HEADERBG);
  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setCursor(NS_STATUS_X, 2);

  if (gnssModule == GNSS_NONE && !nsHaveData) {
    tft.setTextColor(NS_RED);
    tft.print("No GNSS module detected");
    return;
  }
  if (!nsHaveData) {
    tft.setTextColor(NS_YELLOW);
    tft.print(gnssModule == GNSS_TESEO ? "Waiting for GSV..." : "Waiting for NAV-SIG...");
    return;
  }

  uint32_t tod = nsTimeSec;
  char buf[40];
  snprintf(buf, sizeof(buf), "%s %02lu:%02lu:%02lu", nsTimeUtc ? "UTC" : "GPS",
           (unsigned long)(tod / 3600), (unsigned long)((tod / 60) % 60), (unsigned long)(tod % 60));
  tft.setTextColor(NS_WHITE);
  tft.print(buf);

  uint32_t age = (millis() - nsLastMillis) / 1000UL;
  snprintf(buf, sizeof(buf), "  age %lus", (unsigned long)age);
  tft.setTextColor(age > 12 ? NS_RED : NS_GREY);   // u-blox: every 5 s, Teseo: every 1 s
  tft.print(buf);
}

static void NavSig_DrawRow(uint8_t idx, int16_t x, int16_t y) {
  const NavSigEntry &s = nsSig[idx];
  char buf[12];

  tft.setTextSize(2);
  tft.setCursor(x, y);
  tft.setTextColor(NavSig_ConstColor(s.gnssId));
  tft.print(GNSS_SignalName(s.gnssId, s.sigId));   // 14 chars, same text as Serial

  snprintf(buf, sizeof(buf), "%-4u", s.svId);
  tft.setTextColor(NS_WHITE);
  tft.print(buf);

  snprintf(buf, sizeof(buf), "%3u ", s.cno);
  tft.setTextColor(NavSig_CnoColor(s.cno));
  tft.print(buf);

  // C/N0 bar
  int16_t w = s.cno * 2;
  if (w > NS_BAR_MAX_W) w = NS_BAR_MAX_W;
  if (w > 0) {
    if (s.used) tft.fillRect(x + NS_BAR_X, y + 2, w, NS_ROW_H - 4, NavSig_CnoColor(s.cno));
    else        tft.drawRect(x + NS_BAR_X, y + 2, w, NS_ROW_H - 4, NavSig_CnoColor(s.cno));
  }
}

static void NavSig_DrawAll() {
  tft.startBuffering();
  tft.fillScreen(NS_BLACK);
  tft.setTextWrap(false);

  // count signals used in the fix
  uint8_t totalUsed = 0;
  for (uint8_t i = 0; i < nsCount; i++) if (nsSig[i].used) totalUsed++;

  // ---- title line ----
  tft.fillRect(0, 0, 800, 20, NS_HEADERBG);
  tft.setTextSize(2);
  int16_t x = 2;
  x = NavSig_DrawBadge(x, "u-blox",   NS_CYAN,   gnssModule == GNSS_UBLOX);
  x = NavSig_DrawBadge(x, "ST Teseo", NS_YELLOW, gnssModule == GNSS_TESEO);
  tft.setCursor(x + 6, 2);
  tft.setTextColor(NS_WHITE);
  char buf[48];
  snprintf(buf, sizeof(buf), "%usig %uused", nsReported, totalUsed);
  tft.print(buf);
  NavSig_DrawStatus();

  // ---- summary line: per-constellation count (used / tracked) ----
  static const uint8_t ids[]   = {0, 2, 3, 6, 5, 1, 7};
  static const char*   names[] = {"GPS", "GAL", "BDS", "GLO", "QZS", "SBS", "NIC"};
  tft.setCursor(4, 22);
  for (uint8_t k = 0; k < sizeof(ids); k++) {
    uint8_t tracked = 0, used = 0;
    for (uint8_t i = 0; i < nsCount; i++) {
      if (nsSig[i].gnssId != ids[k]) continue;
      tracked++;
      if (nsSig[i].used) used++;
    }
    if (tracked == 0) continue;
    tft.setTextColor(NavSig_ConstColor(ids[k]));
    snprintf(buf, sizeof(buf), "%s %u/%u ", names[k], used, tracked);   // used / tracked
    tft.print(buf);
  }

  // ---- column headers ----
  uint8_t pages = (nsCount + NS_PER_PAGE - 1) / NS_PER_PAGE;
  if (pages == 0) pages = 1;
  if (nsPage >= pages) nsPage = 0;

  tft.drawFastHLine(0, 40, 800, NS_DGREY);
  tft.setTextColor(NS_GREY);
  for (uint8_t c = 0; c < NS_COLS; c++) {
    tft.setCursor(c * NS_COL_W + 4, 42);
    tft.print("Signal        SV  C/N0");
  }
  if (pages > 1) {
    snprintf(buf, sizeof(buf), "%u/%u", nsPage + 1, pages);
    tft.setTextColor(NS_WHITE);
    tft.setCursor(800 - (int16_t)strlen(buf) * NS_CHAR_W - 4, 42);
    tft.print(buf);
  }
  tft.drawFastHLine(0, 58, 800, NS_DGREY);
  tft.drawFastVLine(NS_COL_W - 2, 42, 480 - 42, NS_DGREY);

  // ---- signal rows ----
  uint16_t first = (uint16_t)nsPage * NS_PER_PAGE;
  for (uint16_t i = first; i < nsCount && i < first + NS_PER_PAGE; i++) {
    uint16_t slot = i - first;
    int16_t x = (slot / NS_ROWS_PER_COL) * NS_COL_W + 4;
    int16_t y = NS_TOP_ROWS_Y + (slot % NS_ROWS_PER_COL) * NS_ROW_H;
    NavSig_DrawRow(i, x, y);
  }

  tft.endBuffering();
}

// Call once after TFT_Start()
void NavSig_Init() {
  NavSig_DrawAll();
}

// Call from loop(). 'tick' = true on the 0.5 s timer event.
void NavSig_Task(bool tick) {
  if (nsNewData) {
    nsNewData = false;
    nsPage = 0;
    nsTick = 0;
    NavSig_DrawAll();
    return;
  }
  if (!tick) return;

  // rotate pages every 2.5 s if the list does not fit on one screen
  if (nsCount > NS_PER_PAGE && ++nsTick >= 5) {
    nsTick = 0;
    nsPage++;
    NavSig_DrawAll();
    return;
  }

  // otherwise only refresh the small status field (data age)
  tft.startBuffering();
  NavSig_DrawStatus();
  tft.endBuffering();
}
