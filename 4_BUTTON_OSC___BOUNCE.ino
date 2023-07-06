
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>                // for sending OSC messages
#include <OSCBundle.h>
#include <OSCData.h>

char ssid[] = "****";                 // your network SSID (name)
char pass[] = "****";              // your network password  B3hindCl0s3dD00rs

WiFiUDP Udp;  // A UDP instance to let us send and receive packets over UDP

const IPAddress destIp(10,4,1,21);   // remote IP of the target device
const unsigned int destPort = 9898;    // remote port of the target device where the NodeMCU sends OSC to
const unsigned int localPort = 8888;   // local port to listen for UDP packets at the NodeMCU (another device must send OSC messages to this port)

// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce2
#include <Bounce2.h>

// WE WILL attach() THE Bounce INSTANCE TO THE FOLLOWING PIN IN setup()
int BOUNCE_PINS[4] = {2,0,4,5};

int INDICATOR_LEDS[4] = {15,13,12,14};
int INDICATOR_STATE[4];
int currentIndicator = 0;

// INSTANTIATE A Bounce OBJECT
Bounce bounce1 = Bounce();
Bounce bounce2 = Bounce();
Bounce bounce3 = Bounce();
Bounce bounce4 = Bounce();

unsigned long waitTime = millis();

OSCErrorCode error;
unsigned int ledState = LOW;     

#ifndef BUILTIN_LED
#ifdef LED_BUILTIN
#define BUILTIN_LED LED_BUILTIN
#else
#define BUILTIN_LED 13
#endif
#endif


void setup() {
  Serial.begin(115200);
  pinMode(BUILTIN_LED, OUTPUT);
 // pinMode(16, OUTPUT);

  pinMode(INDICATOR_LEDS[0], OUTPUT);
  pinMode(INDICATOR_LEDS[1], OUTPUT);
  pinMode(INDICATOR_LEDS[2], OUTPUT);
  pinMode(INDICATOR_LEDS[3], OUTPUT);
  
  digitalWrite(BUILTIN_LED, ledState);    // turn *on* led
  //digitalWrite(16, HIGH);

  // BOUNCE SETUP

  // SELECT ONE OF THE FOLLOWING :
  // 1) IF YOUR INPUT HAS AN INTERNAL PULL-UP
  // bounce.attach( BOUNCE_PIN ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
  // 2) IF YOUR INPUT USES AN EXTERNAL PULL-UP
  bounce1.attach( BOUNCE_PINS[0], INPUT_PULLUP ); // USE EXTERNAL PULL-UP
  bounce2.attach( BOUNCE_PINS[1], INPUT_PULLUP );
  bounce3.attach( BOUNCE_PINS[2], INPUT_PULLUP );
  bounce4.attach( BOUNCE_PINS[3], INPUT_PULLUP );

  // DEBOUNCE INTERVAL IN MILLISECONDS
  bounce1.interval(5); // interval in ms
  bounce2.interval(5); // interval in ms
  bounce3.interval(5); // interval in ms
  bounce4.interval(5); // interval in ms
  
  // while (!Serial);

////WIFI 
// Specify a static IP address
// If you erase this line, your ESP8266 will get a dynamic IP address
  //WiFi.config(IPAddress(192,168,89,123),IPAddress(192,168,0,1), IPAddress(255,255,255,0)); 

// Connect to WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Starting UDP");
    Udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(Udp.localPort());
    Serial.println("Type 'RESET' to reboot");
}

int butt1 = 0;
int butt2 = 0;
int butt3 = 0;
int butt4 = 0;

void loop() {
    
    // Update the Bounce instance (YOU MUST DO THIS EVERY LOOP)
    bounce1.update();
    bounce2.update();
    bounce3.update();
    bounce4.update();
    
    if ( bounce1.changed() ) {
      butt1 = bounce1.read();
      ///send message
        OSCMessage msgOut("/1"); ///// Address each here!!!!
        msgOut.add(butt1);
        Udp.beginPacket(destIp, destPort);
        msgOut.send(Udp); // send the bytes to the SLIP stream
        Udp.endPacket(); // mark the end of the OSC Packet
        msgOut.empty(); // free space occupied by message
        
        
        currentIndicator = 0;
        updateIndicator();
        Serial.println(currentIndicator+1);
    }

    if ( bounce2.changed() ) {
      butt2 = bounce2.read();
      ///send message
        OSCMessage msgOut("/2"); ///// Address each here!!!!
        msgOut.add(butt2);
        Udp.beginPacket(destIp, destPort);
        msgOut.send(Udp); // send the bytes to the SLIP stream
        Udp.endPacket(); // mark the end of the OSC Packet
        msgOut.empty(); // free space occupied by message
        //Serial.println("2");
        
        currentIndicator = 1;
        updateIndicator();
        Serial.println(currentIndicator+1);
    }

    if ( bounce3.changed() ) {
      butt3 = bounce3.read();
      ///send message
        OSCMessage msgOut("/3"); ///// Address each here!!!!
        msgOut.add(butt3);
        Udp.beginPacket(destIp, destPort);
        msgOut.send(Udp); // send the bytes to the SLIP stream
        Udp.endPacket(); // mark the end of the OSC Packet
        msgOut.empty(); // free space occupied by message
        //Serial.println("3");

        currentIndicator = 2;
        updateIndicator();
        Serial.println(currentIndicator+1);
    }

    if ( bounce4.changed() ) {
      butt4 = bounce4.read();
      ///send message
        OSCMessage msgOut("/4"); ///// Address each here!!!!
        msgOut.add(butt4);
        Udp.beginPacket(destIp, destPort);
        msgOut.send(Udp); // send the bytes to the SLIP stream
        Udp.endPacket(); // mark the end of the OSC Packet
        msgOut.empty(); // free space occupied by message
        //Serial.println("4");

        currentIndicator = 3;
        updateIndicator();
        Serial.println(currentIndicator+1);
        
    }
  
  ///// Type 'Reset' in monitor to reboot device
  if(Serial.available()){
   // message = Serial.read();
    String command = Serial.readStringUntil('\n');
    //Serial.print("You typed: " );
    Serial.println(command);
    if(command.equals("reset")){
      Serial.print("Im Reseting");
      ESP.restart();
      }
  }

 //// Auto reboot if loses WIFI
  unsigned long current_time = millis(); // number of milliseconds since the upload
  unsigned long previous_time = 0;
  unsigned long wait = 10000;  // 10 seconds delay

  if ((WiFi.status() != WL_CONNECTED) && (current_time - previous_time >=wait)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WIFI network");
    ESP.restart();
    previous_time = current_time;
  }

}

void updateIndicator(){
         for(int i=0; i<4; i++){
          if(INDICATOR_STATE[i] == HIGH){
            digitalWrite(INDICATOR_STATE[i], LOW);
            }
          }
            digitalWrite(INDICATOR_STATE[currentIndicator], HIGH);
  }

  
/*
///OSC LISTEN
  OSCMessage msg;
  int size = Udp.parsePacket();

  if (size > 0) {
    while (size--) {
      msg.fill(Udp.read());
    }
    if (!msg.hasError()) {
      msg.dispatch("/led", led);
      //char data = msg.read();
    } else {
      error = msg.getError();
      Serial.print("error: ");
      Serial.println(error);
    }
  }
  
}
void led(OSCMessage &msg) {
  ledState = msg.getInt(0);
  digitalWrite(BUILTIN_LED, ledState);
  Serial.print("/led: ");
  Serial.println(ledState);
}
*/
