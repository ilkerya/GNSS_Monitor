/*
  NavSigDisplay.h
  Shows the UBX-NAV-SIG signal list (same content as the Serial Monitor output)
  on the Arduino GIGA Display Shield (800x480, landscape).

  Layout
    y   0..19   title line   : "GNSS NAV-SIG", module status, GPS time of day, data age
    y  20..39   summary line : signals per constellation + number used in the fix
    y  42..57   column header: Signal / SV / C/N0
    y  60..479  signal rows  : 2 columns x 26 rows = 52 signals per page
                               (if there are more, the pages rotate every 2.5 s)

  Each row: "<signal name> <svId> <cno>"  + a C/N0 bar (2 px per dB-Hz).
    Name colour  = constellation
    Bar colour   = signal strength (green >= 40, yellow >= 30, orange >= 20, red > 0)
    Filled bar   = pseudorange used in the navigation solution, outline = tracked only

  Data flow
    newSIG() (UBX callback)  -> NavSig_Store()  copies the data into a small snapshot
    loop()                   -> NavSig_Task()   redraws when new data arrived, or on the 0.5 s tick

  Drawing never happens inside the callback, and the full redraw is done with
  startBuffering()/endBuffering() so the screen does not flicker.
*/
#pragma once

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
static uint32_t      nsITOW       = 0;
static uint32_t      nsLastMillis = 0;
static bool          nsHaveData   = false;
static volatile bool nsNewData    = false;
static uint8_t       nsPage       = 0;
static uint8_t       nsTick       = 0;
bool                 nsGnssOk     = false;  // set by GNSS_Setup()

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
  nsITOW       = d->header.iTOW;
  nsLastMillis = millis();
  nsHaveData   = true;
  nsNewData    = true;
}

// ---------- drawing ----------
static void NavSig_DrawStatus() {
  // Right part of the title line: GPS time of day + age of the data
  tft.fillRect(350, 0, 450, 20, NS_HEADERBG);
  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setCursor(360, 2);

  if (!nsGnssOk && !nsHaveData) {
    tft.setTextColor(NS_RED);
    tft.print("u-blox GNSS not detected");
    return;
  }
  if (!nsHaveData) {
    tft.setTextColor(NS_YELLOW);
    tft.print("Waiting for NAV-SIG...");
    return;
  }

  uint32_t tod = (nsITOW / 1000UL) % 86400UL;   // GPS time of day
  char buf[40];
  snprintf(buf, sizeof(buf), "GPS %02lu:%02lu:%02lu",
           (unsigned long)(tod / 3600), (unsigned long)((tod / 60) % 60), (unsigned long)(tod % 60));
  tft.setTextColor(NS_WHITE);
  tft.print(buf);

  uint32_t age = (millis() - nsLastMillis) / 1000UL;
  snprintf(buf, sizeof(buf), "  age %lus", (unsigned long)age);
  tft.setTextColor(age > 12 ? NS_RED : NS_GREY);   // NAV-SIG comes every 5 s
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
  tft.setCursor(4, 2);
  tft.setTextColor(NS_YELLOW);
  tft.print("GNSS NAV-SIG");
  tft.setTextColor(NS_WHITE);
  char buf[48];
  snprintf(buf, sizeof(buf), "  %u sig %u used", nsReported, totalUsed);
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
