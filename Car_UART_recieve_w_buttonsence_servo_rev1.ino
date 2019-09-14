/* 
*  Libraries
 *  ---------
 *  - SoftwareSerial
 *  - Adafruit_BLE
 *  - Adafruit_BluefruitLE_SPI
 *  - Adafruit_BluefruitLE_UART
 *  -SPI
 *  -servo
 *
 * Connections
 * -----------
 * Arduino Uno and Bluefruit BLE:
 *  
 *  Arduino Uno   |   Bluefruit BLE
 *  ------------------------------
 *        -       |       DFU
 *        GND     |       GND
 *        -       |       RTS
 *        5V      |       VIN
 *        7       |       RXI
 *        8      |       TXO
 *        11      |       CTS
 *        12      |       MOD


*/

  #include <SPI.h>
  #include <string.h>
  #include <Arduino.h>
  #include "Adafruit_BLE.h"
  #include <Servo.h>

  #include "Adafruit_BluefruitLE_UART.h"
  
  #include "BluefruitConfig.h"
  #if SOFTWARE_SERIAL_AVAILABLE
    #include <SoftwareSerial.h>
  #endif
/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


//Set servo names
Servo servorightfront;
Servo servoleftfront;
Servo servorightrear;
Servo servoleftrear;
//set servo output pins
int pinservorightfront = 10;   // a maximum of eight servo objects can be created but PWM pins must be used
int pinservoleftfront = 9;
int pinservorightrear= 5;
int pinservoleftrear = 3;

uint8_t buttnum = 1;

//sets variables for distance device
long duration, distance;
int trigPin = 4;
int echoPin = 2; 


//Back up variables and flag
bool back_flag = false;
int back_start = 7; // variable to set at what distance to back up the car
int back_stop = 30; // variable to set at what distance to steop backing up the car

int for_speed = 20;  //speed to travel forward and back forward speed = 90 + for_speed --- backspeed = 90 - for_speed 
int side_speed = 10;  //speed to turn right and left 
//right turn ----  right wheels = 90 +  side_speed left wheels = 90 - for_speed
//left turn ----   right wheels = 90 -  side_speed left wheels = 90 + for_speed

int newspeedr = 90; // right speed calculation to pass to servo write function
int newspeedl = 90; // left speed calculation to pass to servo write function



/**************************************************/



void setup()
{
  // attaches the servo to the PWM pins defined above
  servorightfront.attach(pinservorightfront); 
  servoleftfront.attach(pinservoleftfront);
  servorightrear.attach(pinservorightrear);  
  servoleftrear.attach(pinservoleftrear);
    
  pinMode(trigPin, OUTPUT); //Range finder Trigger Pin assignment and OUTPUT
  pinMode(echoPin, INPUT);  //Range finder Echo Pin assignment and IINPUT
  
  Serial.begin(9600);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}
//servo set function
void servoset()
{
  servorightfront.write(newspeedr);   //  Sets right side wheel speed
  servorightrear.write(newspeedr);    //  Sets right side wheel speed
  servoleftfront.write(newspeedl);    //  Sets left side wheel speed
  servoleftrear.write(newspeedl);     //  Sets left side wheel speed
  Serial.print("  newspeedr=  ");
  Serial.print(newspeedr);
  Serial.print("  newspeedl=  ");
  Serial.println(newspeedl);
}
//Bluefruit recieve function
void recvData()
{ 
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

 

  // Buttons
  if (packetbuffer[1] == 'B') {
    buttnum = packetbuffer[2] - '0';
  //   
    Serial.print ("Button "); Serial.println(buttnum);
   
}
}
// distance calc function
void dist()
{
  digitalWrite(trigPin, LOW); 
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2) / 29.1;
    Serial.print("distance  ");
    Serial.print(distance);
    Serial.println (" cm ");
}
/**************************************************/

void loop()
{
//calls Bluefruit recieve function to get button number from Adafruit App controller
    recvData();

//calls distance function to dermine distance from object in front of car    
    dist();
    
    
    
 //if distance is less than back up start distance or backflag is true continue to backup   
  if (distance < back_start || back_flag == true ) 
{ 
    buttnum = 6;
    back_flag = true;
    Serial.print(" Backing up ");
    Serial.print(buttnum);
     
}
 //if distance is greater than back up start distance and backflag is true stop reset backup flag and turn right about 90 degrees 
 //and set button number to 1 for stop
 
   if (distance > back_stop && back_flag == true)
{
  back_flag = false;
  buttnum = 1;
  Serial.print("Turn Right ");
  Serial.print(buttnum);
  newspeedr = 90 - side_speed;    
  newspeedl = 90 - side_speed;
  servoset();
  delay(1000); 
      
}
  if (buttnum == 1)
{
  Serial.print("Stop   ");
  Serial.print(buttnum);
  newspeedr = 90;
  newspeedl = 90;
  
}

  if (buttnum == 5)
{
  Serial.print("Forward   ");
  Serial.print(buttnum);
   newspeedr = 90 - for_speed;
   newspeedl = 90 + for_speed;    
}

if (buttnum == 6)
{
  Serial.print("Back   ");
  Serial.print(buttnum);
  newspeedr = 90 + for_speed;
  newspeedl = 90 - for_speed;   
}
if (buttnum == 8)
{
  Serial.print("Right   ");
  Serial.print(buttnum);
  newspeedr = 90 - side_speed;    
  newspeedl = 90 - side_speed; 

}
  if (buttnum == 7)
{
  Serial.print("Left   ");
  Serial.print(buttnum);
  newspeedr = 90 + side_speed;    
  newspeedl = 90 + side_speed; 

}
  //  90 is the resting value or no speed mid value 0-180

  servoset();
}

    
