/*
  GigaDisplay GFX - Demo
  This sketch performs a comprehensive speed test of the GFX library, 
  measuring performance for text, lines, shapes, and rotations.
  Optimized for Landscape mode (800x480) on the Arduino Giga Display.
*/
#include <mbed.h>
#include "Arduino_GigaDisplay_GFX.h"

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SERIAL myGNSS;

#define mySerial Serial2 // Change this to (e.g.) Serial1 if needed


GigaDisplay_GFX tft;

#define GC9A01A_CYAN    0x07FF
#define GC9A01A_RED     0xf800
#define GC9A01A_BLUE    0x001F
#define GC9A01A_GREEN   0x07E0
#define GC9A01A_MAGENTA 0xF81F
#define GC9A01A_WHITE   0xffff
#define GC9A01A_BLACK   0x0000
#define GC9A01A_YELLOW  0xFFE0

mbed::Ticker timer100us;

volatile uint32_t timerCounter = 0;
volatile bool eventTriggered = false;
uint16_t Counter;

#include "GNSS_UBlox.h"
#include "Display.h"
/*
#include "DAQ.h"
#include "Sensors.h"
#include "MemSave.h"
#include "Functions.h"
#include "Menu.h"
#include "Connect.h"
*/
// Die Interrupt Service Routine (ISR) - wird alle 100 Mikrosekunden aufgerufen
void handle100usInterrupt() {
    timerCounter++;
    if(timerCounter > 5000){
        timerCounter = 0;
        eventTriggered = true;
    }
}
void setup() {
     pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
      delay(100);
    timer100us.attach_us(&handle100usInterrupt, 100);
  TFT_Start();
  GNSS_Setup();
  delay(1000);
}

void loop(void) {
  if (eventTriggered) {
    eventTriggered = false; // Flag zurücksetzen
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        // Fügen Sie hier Code ein, der alle 100µs ausgeführt werden soll,
        // aber zu lang für die ISR selbst ist (z. B. Berechnungen).
  //  Serial.print("Counter: ");
   // Serial.println(Counter); //  
    Counter++;
   // for (uint8_t rotation = 0; rotation < 4; rotation++) {
    // tft.setRotation(rotation);
      testText();

   // }  
  }
          myGNSS.checkUblox(); // Check for the arrival of new data and process it.
       myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
}



unsigned long testText() {
 // tft.fillScreen(GC9A01A_BLACK);
  //unsigned long start = micros();

// tft.fillScreen(0x0000);
  tft.setTextColor(GC9A01A_YELLOW);
    tft.setTextSize(5);
      tft.setCursor(5, 0);
  tft.print("Counter:");
 // tft.setCursor(5, 8);
 // tft.println("     "); 
 // tft.setTextColor(GC9A01A_BLACK);
  //  tft.setCursor(275, 0);
 //  tft.println("     "); 
// tft.setCursor(275, 0);
  tft.setTextColor(GC9A01A_BLUE, GC9A01A_BLACK); 
//  tft.setTextColor(GC9A01A_RED);
  
   tft.println(Counter); 
 
 
 // tft.setTextColor(GC9A01A_WHITE);  tft.setTextSize(1);
 // tft.println("Hello World!");

//  tft.setTextColor(GC9A01A_YELLOW); tft.setTextSize(2);
 // tft.println(1234.56);


/*
  tft.setTextColor(GC9A01A_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(GC9A01A_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
  */
}
