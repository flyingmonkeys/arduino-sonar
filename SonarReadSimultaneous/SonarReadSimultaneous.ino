/*
  SonarRead
  
  Read analog voltages on pins A0-A5, convert them to distances based on connections to the MB1360 
  sonar transducer sensors, and output them on the serial bus.
  
  The MB1360 is wired to the Arduino Uno in "analog voltage output" mode, such that the AN pin
  (pin 3) outputs a voltage proportional to the range of the first significant target it detects.
  The voltage is Vcc/(1024*cm), which effectively constrains the range to 10m for 5V Vcc.
  
  The minimum time between range readings is 99ms. This sketch will ping all sensors simultaneously to
  test crosstalk issues. 
*/

/* WARNING: Use of SPI or I2C ports may conflict with the range enable GPIO pins
//#define USE_SPI
//#define USE_I2C

#ifdef USE_SPI
  #include <SPI.h>
#endif
#ifdef USE_I2C
  #include <Wire.h>
#endif

/* Define analog ports for each sensor */
#define SENSOR_X_PLUS  A0
#define SENSOR_X_MINUS A1
#define SENSOR_Y_PLUS  A2
#define SENSOR_Y_MINUS A3
#define SENSOR_Z_PLUS  A4
#define SENSOR_Z_MINUS A5


const int rangeEnablePinX0 = 2; // use pin 2 for range enable pin on X-axis
const int rangeEnablePinX1 = 3; // use pin 3 for range enable pin on X-axis
const int rangeEnablePinY0 = 4; // use pin 4 for range enable pin on Y-axis
const int rangeEnablePinY1 = 5; // use pin 5 for range enable pin on Y-axis
const int rangeEnablePinZ0 = 6; // use pin 6 for range enable pin on Z-axis
const int rangeEnablePinZ1 = 7; // use pin 7 for range enable pin on Z-axis

const int minEnablePulseWidthUs = 200; // minimum pulse width = 20us
const int minRangeReadTimeMs = 99; // minimum range reading cycle time is 99ms (can only read range data every 99ms)
const int calibrationTimeMs = 20-1; // MB1360 calibration time = 20.5ms (err on the shorter time)

// Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)  
// MB1360 analog output (5V Vcc) = 2.44mV/cm = 6.2mV/in = Vcc/(1024*cm)
const float analogReadToInches = (5.0/1023.0) * (1.0/6.2012) * (1000.0); // (5V at 1023 A/D value), (6.2012 mV/in), (1000mV/V)

// the setup routine runs once when you press reset:
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // initialize SPI interface (Uno pins 10-13)
#ifdef USE_SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV128); // 16Mhz / 128 clock speed
  SPI.setBitOrder(MSBFIRST);
  pinMode(10, OUTPUT); // set Slave Select (pin 10) to output (master) mode
#endif
#ifdef USE_I2C
  Wire.begin(); // configure as a master (don't pass any parameters)
#endif

  // Set up and initialize range enable output pins
  pinMode(rangeEnablePinX0, OUTPUT);
  pinMode(rangeEnablePinX1, OUTPUT);
  pinMode(rangeEnablePinY0, OUTPUT);
  pinMode(rangeEnablePinY1, OUTPUT);
  pinMode(rangeEnablePinZ0, OUTPUT);
  pinMode(rangeEnablePinZ1, OUTPUT);
  digitalWrite(rangeEnablePinX0, LOW); // set low initially
  digitalWrite(rangeEnablePinX1, LOW); // set low initially
  digitalWrite(rangeEnablePinY0, LOW); // set low initially
  digitalWrite(rangeEnablePinY1, LOW); // set low initially
  digitalWrite(rangeEnablePinZ0, LOW); // set low initially
  digitalWrite(rangeEnablePinZ1, LOW); // set low initially
  
  delay(50); // delay 50ms from power-up for the MB1360 sensor
}

// the loop routine runs over and over again forever:
void loop() 
{
  float distanceInches_X_plus;
  float distanceInches_X_minus;
  float distanceInches_Y_plus;
  float distanceInches_Y_minus;
  float distanceInches_Z_plus;
  float distanceInches_Z_minus;
  
  // trigger all sensors
  digitalWrite(rangeEnablePinX0, HIGH);
  digitalWrite(rangeEnablePinX1, HIGH);
  digitalWrite(rangeEnablePinY0, HIGH);
  digitalWrite(rangeEnablePinY1, HIGH);
  digitalWrite(rangeEnablePinZ0, HIGH);
  digitalWrite(rangeEnablePinZ1, HIGH);
  delayMicroseconds(minEnablePulseWidthUs); 
  digitalWrite(rangeEnablePinX0, LOW);
  digitalWrite(rangeEnablePinX1, LOW);
  digitalWrite(rangeEnablePinY0, LOW);
  digitalWrite(rangeEnablePinY1, LOW);
  digitalWrite(rangeEnablePinZ0, LOW);
  digitalWrite(rangeEnablePinZ1, LOW);
  
  delay(minRangeReadTimeMs); // wait max range time
  
  // now sensor readings are valid, so read now
  distanceInches_X_plus = analogRead(SENSOR_X_PLUS) * analogReadToInches;
  distanceInches_X_minus = analogRead(SENSOR_X_MINUS) * analogReadToInches;
  distanceInches_Y_plus = analogRead(SENSOR_Y_PLUS) * analogReadToInches;
  distanceInches_Y_minus = analogRead(SENSOR_Y_MINUS) * analogReadToInches;
  distanceInches_Z_plus = analogRead(SENSOR_Z_PLUS) * analogReadToInches;
  distanceInches_Z_minus = analogRead(SENSOR_Z_MINUS) * analogReadToInches;
  
  // print out the value you read:
//  Serial.print("Distance");
//  Serial.print(" ");
  Serial.print(distanceInches_X_plus,2);  // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_X_minus,2); // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_Y_plus,2);  // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_Y_minus,2); // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_Z_plus,2);  // 2 decimal places
  Serial.print(" ");
  Serial.println(distanceInches_Z_minus,2); // 2 decimal places

#ifdef USE_SPI
SPI.transfer(100); // transfer() exchanges one byte
SPI.transfer((byte)(distanceInches_Z_plus));
#endif
#ifdef USE_I2C
Wire.beginTransmission(76); // transmit to device #76 (made up address)
Wire.write(100);
Wire.endTransmission();
#endif

}
