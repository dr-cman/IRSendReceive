
 # IRSendReceive

 A program for an ESP8266 board with following functions
 Version 1.0.1: initial version
   - http web interface for sending & receiving IR codes, getting status information
     and for system configuration
   - network configuration via WiFi manager
   - receiving IR codes from IR remote controls via an IR receiver (e.g. TSOP3123) at GPIO 14
   - sending IR codes using IR transmitter diodes connected to GPIO 4
   - forwarding of received IR codes to Fhem sevrer via http to set a configurable dummy variable
   - over the air update (OTA)

 ### Version 1.1.0: status led added (using ticker)
   - additional status LED connected to GPIO 0. The LED
        blinks with 0.5 Hz during connection to WiFi network
        is on for 2 seconds when an IR command is sent to Fhem
        is on for 1 second whem a Temperature/Humidity value is sent to Fhem (Version >= 1.2.0)

### Version 1.2.0: DHT22 temperature/humidity sensor added
   - DHT22 temperature/humidity sensor connected to GPIO 2 with configurable cyclic readout
   - readout values are sent via http to the Fhem server to set a configurable Fhem dummy variable

### Version 1.2.1: bugfix & Status to Fhem added
   - update of forceSend was missing -> T/H always sent to Fhem
   - ststus information about received/sent and transferred IR commands is sent to Fhem dummy variable for IR commands

### http commands

     http://xxx.xxx.xxx.xxx<cmd>

     <cmd>
     /         Homepage with status information
     /setup    configure IRSendReceive
               FhemIP=xxx.xxx.xxx.xxx  IP adress of Fhem server  (default 192.168.2.12)
               FhemPort=xxxx           Port number of Fhem server (default 8083)
               FhemMsg=[0|1]           Send IR command messages to Fhem (default 1)
                                       0=no messages; 1=send messages
               FhemFmt=[0|"fhem"]      transfer format of received IR commands (default 1)
                                       0 -> json output; "fhem" -> fhem output (default "fhem")
               FhemVarIR               Fhem variable to be set with IR command data (default "d_IR")
               FhemVarTH               Fhem variable to be set with T/H values (default "d_Temp1")
               DHTcycle                cycle time for T/H measurements in ms (default 60000)
     /msg      send IR command 
     /json     send IR command in json format
     /received retrieve received IR command in json format
               Arguments:  
               &id=n n=[1..5]   retrieve details of lan n-th received code
     /update   start OTY firmware update
 
 Huge parts of this program code are taken from the project ESP8266-HTTP-IR-Blaster
 (https:github.com/mdhiggins/ESP8266-HTTP-IR-Blaster) from Michael Higgins.
 Thanks to Michael for the cool work!


