# IRSendReceive
IR send/receive system with fhem support based on ESP8266

A program (sketch) for an ESP8266 board with following functions
   - http web interface for sending & receiving IR codes, getting status information
     and for system configuration
   - network configuration via WiFi manager
   - receiving IR codes from IR remote controls via an IR receiver (e.g. TSOP3123) at GPIO14
   - sending IR codes using IR transmitter diodes connected to GPIO 4
   - forwarding of received IR codes to fhem home automation system via http
   - over the air update (OTA)

Huge code parts of this program are taken from the project ESP8266-HTTP-IR-Blaster
(https://github.com/mdhiggins/ESP8266-HTTP-IR-Blaster) from Michael Higgins.
Thanks to Michael for the cool work!
