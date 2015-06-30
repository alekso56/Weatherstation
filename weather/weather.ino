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
float weatherData[6];//temp,pressure,altitude in meters,Lux intensity,current,,solarvolt,

void setup() {
  radio.begin();
  if (isWeatherStation) {
    radio.powerDown();
    pinMode(5, OUTPUT);//bmp
    pinMode(4, OUTPUT);//current sensor
    pinMode(3, OUTPUT);//solar voltage sensor
    pinMode(2, OUTPUT);//light sensor
    pinMode(A1, INPUT);//current sensor
    pinMode(A0, INPUT);//solar voltage sensor
  } else {
    radio.startListening();
  }
}

void getLightSensorData() {
  digitalWrite(2, HIGH); //light sensor
  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_L);
  LightSensor.SetMode(Continuous_H_resolution_Mode);
  delay(50);
  weatherData[3] = LightSensor.GetLightIntensity();
  digitalWrite(2, LOW); //light sensor
}
void getBMPData() {
  digitalWrite(5, HIGH); //bmp
  delay(50);
  if (!bmp.begin()) {
    // Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    digitalWrite(5, LOW); //bmp
    return;
  }
  weatherData[0] = bmp.readTemperature();
  weatherData[1] = bmp.readPressure(); //Pa
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  weatherData[2] = bmp.readAltitude(101400); //meters
  digitalWrite(5, LOW); //bmp
}
void getCurrentData() {
  digitalWrite(4, HIGH); //current
  weatherData[4] = analogRead(A1); //current
  digitalWrite(4, LOW); //current
}
void getVoltageData() {
  digitalWrite(3, HIGH); //voltage
  weatherData[5]  = analogRead(A0) * (15.8 / 1023.0);
  digitalWrite(3, LOW); //voltage
}
void sendWeatherData() {
  radio.powerUp();
  //sender
  radio.openWritingPipe(pipes[0]);
  //radio.openReadingPipe(1, pipes[1]);

  radio.write( &weatherData, sizeof(weatherData) );
  // Shut down the system
  delay(500);                     // Experiment with some delay here to see if it has an effect
  // Power down the radio.
  radio.powerDown();              // NOTE: The radio MUST be powered back up again manually
}

void debugData() {
  Serial.begin(9600);
  Serial.print(weatherData[0]);
  Serial.println(" C");
  Serial.print(weatherData[1]);
  Serial.println(" Pa");
  Serial.print(weatherData[2]);
  Serial.println(" meters");
  Serial.print(weatherData[3]);
  Serial.println(" lux");
  Serial.print(weatherData[4]);
  Serial.println(" current");
  Serial.print(weatherData[5]);
  Serial.println(" volts");
  delay(5);
  Serial.end();
}

void loop() {
  if (isWeatherStation) {
    getLightSensorData();
    getBMPData();
    getCurrentData();
    getVoltageData();
    // sendWeatherData();
    debugData();
    delay(500);
  }
  if (not isWeatherStation)  {
    //receiver
    //radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1, pipes[0]);
    if ( radio.available() ) {                                  // if there is data ready
      while (radio.available()) {                             // Dump the payloads until we've gotten everything
        radio.read( &weatherData, sizeof(weatherData) );       // Get the payload, and see if this was the last one.
      }
      debugData();
    } else {
      delay(50);                                             // Delay so the serial data can print out
      do_sleep();

    }
  }
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
void WakeUP() {
  sleep_disable();
}

void do_sleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  attachInterrupt(0, WakeUP, LOW);
  WDTCSR |= _BV(WDIE);
  sleep_mode();                        // System sleeps here
  // The WDT_vect interrupt wakes the MCU from here
  sleep_disable();                     // System continues execution here when watchdog timed out
  detachInterrupt(0);
  WDTCSR &= ~_BV(WDIE);
}
