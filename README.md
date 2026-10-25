
GNSS Modules ublox/ST Teseo Livf
D19 Rx as input
D18 Tx output 

I added a NAV-SIG screen for the GIGA Display (800×480). It shows the same signal list you see in the Serial Monitor, and the Serial output itself stays the same.
What's on the screen
Top bar: the total number of signals, how many are used in the position fix, GPS time of day, and how old the data is. The age turns red if no update has arrived for more than 12 s.
Second line: used/tracked counts for each satellite system (GPS, Galileo, BeiDou, GLONASS and so on), colour-coded.
Signal list: two columns showing signal name, satellite number, signal strength, and a bar in place of the ==== text bar. The bar is green, yellow, orange or red by strength. A filled bar means the signal is used in the fix; an outline means it's tracked only.
More than 52 signals: the list splits into pages that switch every 2.5 s.
Code changes
New file NavSigDisplay.h: when new satellite data arrives it saves a small copy, and loop() redraws the screen from that copy. The screen is redrawn in one go so it doesn't flicker. Between updates only the "age" field is redrawn, every 0.5 s.
GNSS_UBlox.h: the signal-name lookup is now one function, GNSS_SignalName(), shared by Serial and the display. I also fixed a possible overrun: the receiver can report more than the 92 signals the SparkFun library stores, and the old loop would then read past the end of its array.
GNSS_Monitor.ino: adds NavSig_Init() in setup() and NavSig_Task() in loop(). The old "Counter:" text demo is commented out.
How I tested it: I couldn't compile against the GIGA board package because Arduino's download server is blocked from my sandbox. Instead, I compiled the display code on my side with the real Adafruit_GFX library on a simulated 800×480 screen. The preview image comes from that run, using made-up satellite data. Please do a first build in your Arduino IDE.
I haven't pushed anything to GitHub. I can commit it to a branch and open a pull request if you'd like.
Files are in GNSS_Monitor.zip:
GNSS_Monitor.ino
GNSS_UBlox.h
NavSigDisplay.h
Display.h
GNSS_ST.h
tft_preview.png

I committed the changes on a new branch, tft-navsig-display, but the push to GitHub failed. This session isn't authorised to write to ilkerya/GNSS_Monitor, so the push came back with "access denied" (403).
You can get the code into your repo in either of these ways:
Let me push it: add ilkerya/GNSS_Monitor to this session's sources (repository access), then ask me to push again. I can also open a pull request.
Push it yourself: copy NavSigDisplay.h, GNSS_UBlox.h and GNSS_Monitor.ino from the zip I sent into your local copy of the repo, then commit and push them.
