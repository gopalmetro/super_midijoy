// A8MidiJoy
// Atari 8-Bit Music Interface Teensy / Teensyduino Code
// based on A8F for 2600 by Sebastian Tomczak (little-scale)
// adapted for Atari 8-bit by Frederik Holst (2014)
// http://www.phobotron.de/midijoy_en.html
// Adapted to Teensy LC by Gopal Metro (2016)
// Developing integration of SIO2FTDI, SIO2ARDUINO by Gopal Metro (2016)

/*
SIO2FTDI - ATARI SIO PORT
SIO 3 (Data In)(Orange)---------> TXD
SIO 4 (Grnd)(black)---------> GND
SIO 5 (Data Out)(grey)----------> RXD
SIO 7 (CMD)(purple)----------> DSR/CTS or RI (could be more than one connection if a switch is employed)
*/

/*
CATALEX MicroSD Card Adapter to Teensy LC
Teensy Pin 11(DOUT) - MOSI
Teensy Pin 12(DIN)  - MISO
Teensy Pin 13(SCK)  - SCK
Teensy Pin 10(CS)   - CS
*/

/*
SIO2ARDUINO - Modified Teensy LC/3.x Pin connections

NOTE: Use Serial3.begin to start the 3rd serial bus.  Need to modify the SPI2Arduino code to handle this.
Atari Data In  (SIO pin 3) -> Teensy pin 1 (8?) (TX) (note: this is usually the orange SIO cable)
Atari Ground   (SIO pin 4) -> Teensy ground (GND)
Atari Data Out (SIO pin 5) -> Teensy pin 0(7) (RX) (note: this is usually the green SIO cable)
Atari Command  (SIO pin 7) -> Teensy pin 3 (!!!need to swap this for a different pin)

//This is the default SPI2Arduino SD Card pin connections.  Verify agains the CATALEX pin connections above.
SD board CS pin            -> Arduino digital pin 10
SD board DI pin            -> Arduino digital pin 11 (verify this shouldn't be 12)
SD board DO pin            -> Arduino digital pin 12 (verify this shouldn't be 11)
SD board CLK pin           -> Arduino digital pin 13
SD board 5v pin            -> Arduino 5V pin
SD board GND pin           -> Arduino GND pin
*/

/*
TEENSY LC, 3.x (USE THIS MAPPING)
Atari 8-Bit and Teensy Hardware Setup: 

Atari Player 1 Pin 1 ---> Teensy Port D0 (LC/3.x - 2)
Atari Player 1 Pin 2 ---> Teensy Port D1 (LC/3.x - 14)
Atari Player 1 Pin 3 ---> Teensy Port D2 (LC/3.x - 7)
Atari Player 1 Pin 4 ---> Teensy Port D3 (LC/3.x - 8)
Atari Player 1 Pin 6 ---> Teensy Port D7 (LC/3.x - 5)
Atari Player 1 Pin 8 ---> Teensy Ground  (LC/3.x - G)

NEED TO SORT OF THE BIT SHIFTING FOR THIS PORT FROM B4-B7 TO B16-B17
Atari Player 2 Pin 1 ---> Teensy Port [B4]B0 (LC/3.x - 16)
Atari Player 2 Pin 2 ---> Teensy Port [B5]B1 (LC/3.x - 17)
Atari Player 2 Pin 3 ---> Teensy Port [B6]B2 (LC/3.x - 19)
Atari Player 2 Pin 4 ---> Teensy Port [B7]B3 (LC/3.x - 18)
Atari Player 2 Pin 6 ---> Teensy Port D6     (LC/3.x - 21)
Atari Player 2 Pin 9 ---> Teensy Port C0 (LC/3.x - 15) For future use: 


OPTIONAL (GOPAL NOTE: Incorrect pins, need to update to LC/3.x pins and need to adjust bit shifting in the 'writeatari' function):
Connect a "classic" serial MIDI I/O-board to Teensy Port D2 (RXD, digital pin 7)
and D3 (TXD, digital pin 8) respectively in order to connect older MIDI devices.
Schematics can be found here:
https://www.pjrc.com/teensy/td_libs_MIDI.html

*/

#include <MIDI.h>

#define TEENSYLC 1 //SET TRUE IF USING TEENSY LC, ELSE SET FALSE
#define TEENSY2 0 //SET TRUE IF USING TEENSE 2.X, ELSE SET FALSE

int pinTable[] = {2,14,7,8,6,20,21,5,16,17,18,19};
int pinTableSize = sizeof(pinTable)/sizeof(pinTable[0]);


byte data; // general working byte for serially-received data

byte channel;
byte pitch; 
byte velocity;

int dT = 3000; // delay time for write cycles in microseconds

int voice[] = {255,255,255,255};
int vcount = 0;
int maxvoice = 4; // maximum number of simultaneous voices on the 8-Bit (Atari: 4, C64: 3)
int startchannel = 1; // first MIDI channel to be used - increase this by [maxvoice] if you use more than one interface

// Setup
void setup() {
  
  Serial.begin(38400);
  if (!Serial) {
    delay(1000);
    Serial.println("Serial begin");
  }
  
  for (int i=0; i<pinTableSize; i++) { 
    pinMode(pinTable[i],OUTPUT); 
  } 
  
  GPIOD_PDOR = B00000000;
  GPIOB_PDOR = B00000000;
    
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleNoteOn(doNote);
  MIDI.setHandleNoteOff(doNoteOff);

  usbMIDI.setHandleNoteOn(doNote); 
  usbMIDI.setHandleNoteOff(doNoteOff); 

}


// Main Program
void loop() {
  MIDI.read();
  usbMIDI.read(); 
}


// Functions
void doNote(byte channel, byte pitch, byte velocity) {
  Serial.println("note on");

  if (channel > startchannel) {
    vcount = channel-startchannel;
    voice[vcount] = pitch;
  } else {
    for (int x=0;x<=(maxvoice-1);x++) {
      if (voice[x] == 255) {
        vcount = x;
        voice[vcount] = pitch;
        break;
      }
    } 
  }
  
  Serial.print("pitch: ");Serial.println(pitch);
  Serial.print("vcount: ");Serial.println(vcount);
  Serial.print("velocity: ");Serial.println(velocity);
  writeatari(B00000000 | pitch, vcount);
  delay(1000);
  writeatari(B10000000 | velocity / 8, vcount);

}

void doNoteOff(byte channel, byte pitch, byte velocity) {
  Serial.println("note off ");

  if (channel > startchannel) {
    vcount = channel-startchannel;
    voice[vcount] = 255;
  } else {
    for (int x=(maxvoice-1);x>=0;x--) {
      if (voice[x] == pitch) {
        vcount = x;
        voice[vcount] = 255;
        break;
      }
    } 
  }

  writeatari(B10000000, vcount);

}

// WRITING DATA TO THE ATARI

void writeatari(int data, int voice) {
  Serial.println("writeatari: active");

  GPIOD_PDOR = ~((data & 15) | (voice << 6));
  GPIOB_PDOR = ~(data >> 4); // >> 4; shifting down to bottom half of Teensy LC/3.x port B

  delayMicroseconds(dT);

}
