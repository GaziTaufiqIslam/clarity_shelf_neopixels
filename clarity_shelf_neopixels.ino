/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-neopixel-led-strip
 */

#include <Adafruit_NeoPixel.h>

#define PIN_NEO_PIXEL 16  // The ESP32 pin GPIO16 connected to NeoPixel
#define NUM_PIXELS 80     // The number of LEDs (pixels) on NeoPixel LED strip (increase number according to length)


#define TRIG_PIN 21 // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 20 // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin

// This holds the RGB value for the neopixel light
int Brightness = 0;

//This is the brightness added or subtracted for pulsing
int fadeAmount = 5;

// The duration and distance variables for the ultrasonic proximity sensor
int duration_us, distance_cm;

// This stores whether the object has been detected or not, the value is either true or false
bool isLoaded;


Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

void setup() {
  NeoPixel.begin();  // initialize NeoPixel strip object (REQUIRED)
    Serial.begin (9600);

  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);
}

void loop() {

    // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);


    // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;


    Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");



if(distance_cm <=22 ) { // Checks if proximity reading is within 25 or not

  if (isLoaded == false)  { //Checks if isLoaded state was already false or not
    loaderLight();  // Calls function to turn on loading lights
    isLoaded = true;  //After loading lights has been displayed and completed value becomes true
  }

  solid(255); //Calls the function to keep the lights solid when something is kept within 25 cm



}


else {
 pulsing(); //calls the pulsing function when nothing is kept within 25cm
}
}
//ENd of Function loop

void pulsing () {
 
   Brightness = Brightness + fadeAmount;  //Brightness increases by fadeamount at each function call
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
    NeoPixel.setPixelColor(pixel, NeoPixel.Color(Brightness, Brightness, Brightness));  // it only takes effect if pixels.show() is called
  }
  NeoPixel.show();
  delay(30); //Delay between each function call


   if (Brightness <= 0 || Brightness >= 180) { //checks if brightness is full or not
    NeoPixel.clear();  // Turns off all neopixels
    fadeAmount = -fadeAmount; // Turns fadeamount negative to pulsing is reveresd
  }
  isLoaded = false; // Loading turned false since no product is kept

 
}


void solid (int x) {

for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
    NeoPixel.setPixelColor(pixel, NeoPixel.Color(x, x, x));

      // it only takes effect if pixels.show() is called
  }
        NeoPixel.show();
}


void loaderLight() {
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
    NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 255, 255));  // it only takes effect if pixels.show() is called
    NeoPixel.show();                                           // update to the NeoPixel Led Strip

    delay(12.5);  // 10ms pause between each pixel
  }
  NeoPixel.clear();

    Brightness = 175;  // This keeps brightness at 250 before pulsing starts to maintain seamlessness of pulsing  

}