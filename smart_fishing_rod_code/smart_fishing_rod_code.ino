/**
  ** Authored by: Sashank Vempati
  ** Date: 06/10/2023
  ** Description: 
  * Code for displaying weight measurements based on force exerted on fishing rod
  * This Arduino script performs several functions:
  * Reads data from Load Cell sensor and converts it into weight based on known 
      weight calibration values and equation converting sensor values to grams.
  * Updates weight value generated onto the OLED display
  * Reads accelerometer and gyroscope data from IMU sensor and converts values to 
      the current angle of orientation (in degrees).
  * LED turns green or blue when IMU sensor is oriented at the "optimal" angle, 
      and turns red otherwise

  ** We have used several resources to make all this functionality possible
  * Get sensor data from Load Cell using HX711 Amplifier:
      https://randomnerdtutorials.com/arduino-load-cell-hx711/
  * Generate content on OLED Display using I2C communication:
      https://randomnerdtutorials.com/guide-for-oled-display-with-arduino/
  * Extracting IMU data and deriving angles:
      https://electrosome.com/interfacing-mpu-6050-gy-521-arduino-uno/
  * Programming a pushbutton switch using Arduino:
      https://docs.arduino.cc/built-in-examples/digital/Button
  * Turn on RGB LED with different values:
      https://howtomechatronics.com/tutorials/arduino/how-to-use-a-rgb-led-with-arduino/
**/

#include <HX711.h>
#include <Wire.h> //library allows communication with I2C / TWI devices
#include <math.h> //library includes mathematical functions
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

// Digital pin values for Arduino
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
const int resetPin = 4;
const int savePin = 5;
int redPin= 6;
int greenPin = 7;
int bluePin = 8;

// Initializing variables for IMU sensor angles 
int sum = 0;
int count = 0;
int reference = 35200;
int saveAngle;

// These values are used for sensor calibration.
// We will be using the initial calibration point when the sensor is at rest (no weight exerted on sensor).
// We are defining a constant k to use for converting from sensor values to weight (g). 
float initial_point = 41500;
float k = (390000+38400)/200;

//imu sensor
const int MPU=0x68; //I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //16-bit integers
int AcXcal,AcYcal,AcZcal,GyXcal,GyYcal,GyZcal,tcal; //calibration variables
double t,tx,tf,pitch,roll;

// Initializing values used for the Pushbutton switches
int resetState;
int saveState;
int prevState = LOW;

// Initializing HX711 instance, used for extracting data from load cell amplifier
HX711 scale;

//OLED display stuff
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };


void setup() {
  Serial.begin(9600);

  // Initialize the scale for the load cell amplifier
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Initialize digital switch pins as input for the correponding pin values
  pinMode(resetPin, INPUT);
  pinMode(savePin, INPUT);

  //Initialize LED pins as output for the corresponding pin values
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initializing IMU sensor
  Wire.begin(); //initiate wire library and I2C
  Wire.beginTransmission(MPU); //begin transmission to I2C slave device
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)  
  Wire.endTransmission(true); //ends transmission to I2C slave device

  //Initializing OLED display

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(1000); // Pause for 1 second

}

void loop() {
  scale.wait_ready_timeout(1000); // Wait for the HX711 to become ready
  long reading = scale.read_average(1); // Read the raw value from the load cell amplifier
  Serial.print(reading);
  Serial.print(" ");

  // We noticed that the weight offsets by 50g, so it adds 50g weight correction
  // to the measure if the calculated weight above 20g.
  if((reading+initial_point)/k < 20) Serial.print((reading+initial_point)/k);
  else Serial.print((reading+initial_point)/k + 50);
  Serial.print(" grams ");

  // Request sensor data from IMU sensor using I2C communication protocol
  Wire.beginTransmission(MPU); //begin transmission to I2C slave device
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); //restarts transmission to I2C slave device
  Wire.requestFrom(MPU,14,true); //request 14 registers in total  

  // Acceleration data correction
  AcXcal = -950;
  AcYcal = -300;
  AcZcal = 0;

  // Gyro correction
  GyXcal = 480;
  GyYcal = 170;
  GyZcal = 210;


  // read accelerometer data
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

  //read temperature data 
  // Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L) 

  // read gyroscope data
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L) 

  // temperature calculation
  // tx = Tmp + tcal;
  // t = tx/340 + 36.53; //equation for temperature in degrees C from datasheet
  // tf = (t * 9/5) + 32; //fahrenheit

  // get pitch/roll
  getAngle(AcX,AcY,AcZ);

  // printing angles to serial port
  // We use either the Pitch and Roll based on how we mounted the IMU sensor.
  // We are using Pitch for the fishing rod angle because on our IMU sensor's orientation.
  Serial.print("Pitch = "); Serial.println(pitch);
  // Serial.print(" Roll = "); Serial.println(roll);

  // LED
  // The LED turns green when IMU sensor is around the same angle 
  // as when the reset button was pushed. 
  if(pitch <= saveAngle + 5 && pitch >= saveAngle - 5){
    setColor(0, 255, 0); // Green color
  }
  else if(pitch >= 45 && pitch <= 75){
    setColor(0, 0, 255); // Blue Color in the optimal angle and reset button hasn't been pressed.
  } else {
    setColor(255, 0, 0); // Red Color
  }

  // switches
  resetState = digitalRead(resetPin); // reset button
  saveState = digitalRead(savePin);   // save button

  // When the reset button is pressed, LED flashes red and zeros the weight reading
  if(resetState == HIGH){
    // reference = resistance;
    initial_point = -1 * reading;
    saveAngle = pitch;

    setColor(0, 0, 0); 
    delay(100);
    setColor(255, 0, 0); // Red Color
    delay(100);
    setColor(0, 0, 0);
    delay(100);
    setColor(255, 0, 0); 
    delay(100);
    setColor(0, 0, 0); 
    delay(100);
    setColor(255, 0, 0); 
    delay(100);
    setColor(0, 0, 0);
    delay(100);
    setColor(255, 0, 0); 
    delay(100);
    setColor(0, 0, 0);
  }

  // When the save button is pressed and fishing rod is at the optimal angle, it flashes green.
  if(saveState == HIGH && pitch >= 45 && pitch <= 75){
    prevState = HIGH;
  
    setColor(0, 0, 0); 
    delay(100);
    setColor(0, 255, 0); // Green Color
    delay(100);
    setColor(0, 0, 0);
    delay(100);
    setColor(0, 255, 0); 
    delay(100);
    setColor(0, 0, 0); 
    delay(100);
    setColor(0, 255, 0); 
    delay(100);
    setColor(0, 0, 0);
    delay(100);
    setColor(0, 255, 0);
    delay(100);
    setColor(0, 0, 0);

    prevState = saveState; // Update previous state info that the save button has been pressed

  }

  // Stops updating the weight reading when the save button is pressed
  // If reset button is pressed, the weight zeros and starts updating values again
  if(prevState == HIGH){
    if(resetState == LOW){
      return;
    } else {
      prevState = LOW;
    }
  }

  // OLED display, set attributes and the size of the content
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);

  // Updates weight reading to the display using the same weight calculation as before.
  if((reading+initial_point)/k < 20) display.print((reading+initial_point)/k);
  else display.print((reading+initial_point)/k + 50);

  // Display the weight measurement units (grams)
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(70, 45);
  display.println("grams");

  // Show new contents on the display
  display.display();

  delay(500); // 500ms Delay between readings
}

// This function calculates the angle from the accelerometer data. 
void getAngle(int Ax,int Ay,int Az) 
{
    double x = Ax;
    double y = Ay;
    double z = Az;

    pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
    roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation

    //converting radians into degrees
    pitch = pitch * (180.0/3.14);
    roll = roll * (180.0/3.14) ;
}

// This function set the RGB color values for the LED
void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

