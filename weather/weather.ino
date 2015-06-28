#include <SPI.h>

#include <printf.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_BMP085.h>

#include <Wire.h>

#include <BH1750FVI.h>

Adafruit_BMP085 bmp;

BH1750FVI LightSensor;

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9, 10);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };   // Radio pipe addresses for the 2 nodes to communicate.
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;
bool isWeatherStation = true;

void setup() {
    radio.begin();
}

void sendWeatherData(){
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); //light sensor
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH); //bmp
  delay(500);
  //Serial.begin(9600);
  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_L);
  LightSensor.SetMode(Continuous_H_resolution_Mode);
  //Serial.println("lightRunning...");
  if (!bmp.begin()) {
    //Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  radio.powerUp();          
  //sender
  //radio.openWritingPipe(pipes[0]);
  //radio.openReadingPipe(1,pipes[1]);

  //receiver
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);

}

void loop() {
  // Pong back role.  Receive each packet, dump it out, and send it back
  if (isWeatherStation) {
    if ( radio.available() ) {                                  // if there is data ready, pls mov
      Serial.print("Temperature = ");
      Serial.print(bmp.readTemperature());
      Serial.println(" *C");

      Serial.print("Pressure = ");
      Serial.print(bmp.readPressure());
      Serial.println(" Pa");

      // Calculate altitude assuming 'standard' barometric
      // pressure of 1013.25 millibar = 101325 Pascal
      Serial.print("Altitude = ");
      Serial.print(bmp.readAltitude());
      Serial.println(" meters");

      Serial.print("Pressure at sealevel (calculated) = ");
      Serial.print(bmp.readSealevelPressure());
      Serial.println(" Pa");

      // you can get a more precise measurement of altitude
      // if you know the current sea level pressure which will
      // vary with weather and such. If it is 1015 millibars
      // that is equal to 101500 Pascals.
      Serial.print("Real altitude = ");
      Serial.print(bmp.readAltitude(101400));
      Serial.println(" meters");

      Serial.println();

      uint16_t lux = LightSensor.GetLightIntensity();
      Serial.print("Light: ");
      Serial.print(lux);
      Serial.println(" lx");
wa
      unsigned long got_time;
      while (radio.available()) {                             // Dump the payloads until we've gotten everything
        radio.read( &got_time, sizeof(unsigned long) );       // Get the payload, and see if this was the last one.
        // Spew it.  Include our time, because the ping_out millis counter is unreliable
        printf("Got payload %lu @ %lu...", got_time, millis()); // due to it sleeping
      }

      radio.stopListening();                                  // First, stop listening so we can talk
      radio.write( &got_time, sizeof(unsigned long) );        // Send the final one back.
      printf("Sent response.\n\r");
      radio.startListening();                                 // Now, resume listening so we catch the next packets.
    } else {
      delay(50);                                             // Delay so the serial data can print out
      do_sleep();

    }
  }
  if (not isWeatherStation)  {                     // stuff should be moved around lol, fix pls
    radio.powerUp();                                // Power up the radio after sleeping
    radio.stopListening();                          // First, stop listening so we can talk.

    unsigned long time = millis();                  // Take the time, and send it.
    printf("Now sending... %lu \n\r", time);

    radio.write( &time, sizeof(unsigned long) );

    radio.startListening();                         // Now, continue listening

    unsigned long started_waiting_at = millis();    // Wait here until we get a response, or timeout (250ms)
    bool timeout = false;
    while ( ! radio.available()  ) {
      if (millis() - started_waiting_at > 250 ) { // Break out of the while loop if nothing available
        timeout = true;
        break;
      }
    }

    if ( timeout ) {                                // Describe the results
      printf("Failed, response timed out.\n\r");
    } else {
      unsigned long got_time;                     // Grab the response, compare, and send to debugging spew
      radio.read( &got_time, sizeof(unsigned long) );

      printf("Got response %lu, round-trip delay: %lu\n\r", got_time, millis() - got_time);
    }

    // Shut down the system
    delay(500);                     // Experiment with some delay here to see if it has an effect
    // Power down the radio.
    radio.powerDown();              // NOTE: The radio MUST be powered back up again manually

    // Sleep the MCU.
    do_sleep();


  }
}

void wakeUp() {
  sleep_disable();
}

// Sleep helpers

//Prescaler values
// 0=16ms, 1=32ms,2=64ms,3=125ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec

void setup_watchdog(uint8_t prescalar) {

  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);                      // Clear the WD System Reset Flag
  WDTCSR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
}

ISR(WDT_vect)
{
  //--sleep_cycles_remaining;
  Serial.println("WDT");
}

void do_sleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  attachInterrupt(0, wakeUp, LOW);
  WDTCSR |= _BV(WDIE);
  sleep_mode();                        // System sleeps here
  // The WDT_vect interrupt wakes the MCU from here
  sleep_disable();                     // System continues execution here when watchdog timed out
  detachInterrupt(0);
  WDTCSR &= ~_BV(WDIE);
}
