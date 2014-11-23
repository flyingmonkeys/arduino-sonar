/*
  SonarRead
  
  Read analog voltages on pins A0-A5, convert them to distances based on connections to the MB1360 
  sonar transducer sensors, and output them on the serial bus.
  
  The MB1360 is wired to the Arduino Uno in "analog voltage output" mode, such that the AN pin
  (pin 3) outputs a voltage proportional to the range of the first significant target it detects.
  The voltage is Vcc/(1024*2cm), which effectively constrains the range to 10m for 5V Vcc.
  
  The minimum time between range readings is 99ms. However, 20.5ms of that time is spent calibrating the
  sensor, so there is no active ping or echo ranging taking place. We can use this to our advantage
  and trigger the next reading 20.5ms early, while the active sensor is still ranging and listening for
  an echo. This will decrease the total cycle time of reading all three axes.
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
#define SENSOR_X_PLUS  A1
#define SENSOR_X_MINUS A2
#define SENSOR_Y_PLUS  A0
#define SENSOR_Y_MINUS A3
#define SENSOR_Z_PLUS  A5
#define SENSOR_Z_MINUS A4

#define GPIO_PIN_OFFSET 2
const int rangeEnablePinX_p = GPIO_PIN_OFFSET + 1;
const int rangeEnablePinX_m = GPIO_PIN_OFFSET + 2;
const int rangeEnablePinY_p = GPIO_PIN_OFFSET + 0;
const int rangeEnablePinY_m = GPIO_PIN_OFFSET + 3;
const int rangeEnablePinZ_p = GPIO_PIN_OFFSET + 5; 
const int rangeEnablePinZ_m = GPIO_PIN_OFFSET + 4;

const int minEnablePulseWidthUs = 200; // minimum pulse width = 20us
const int minRangeReadTimeMs = 99; // minimum range reading cycle time is 99ms (can only read range data every 99ms)
const int calibrationTimeMs = 20-1; // MB1360 calibration time = 20.5ms (err on the shorter time)

// Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)  
// MB1360 analog output (5V Vcc) = 2.44414mV/cm = 6.2mV/in = Vcc/(1024*2cm)
const float analogReadToInches = (5000.0/1023.0) / 6.2012; // (5V at 1023 A/D value), (6.2012 mV/in), (1000mV/V)

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
  pinMode(rangeEnablePinX_p, OUTPUT);
  pinMode(rangeEnablePinX_m, OUTPUT);
  pinMode(rangeEnablePinY_p, OUTPUT);
  pinMode(rangeEnablePinY_m, OUTPUT);
  pinMode(rangeEnablePinZ_p, OUTPUT);
  pinMode(rangeEnablePinZ_m, OUTPUT);
  digitalWrite(rangeEnablePinX_p, LOW); // set low initially
  digitalWrite(rangeEnablePinX_m, LOW); // set low initially
  digitalWrite(rangeEnablePinY_p, LOW); // set low initially
  digitalWrite(rangeEnablePinY_m, LOW); // set low initially
  digitalWrite(rangeEnablePinZ_p, LOW); // set low initially
  digitalWrite(rangeEnablePinZ_m, LOW); // set low initially
  
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
  
  // trigger X axis sensor(s)
  digitalWrite(rangeEnablePinX_p, HIGH);
  digitalWrite(rangeEnablePinX_m, HIGH);
  delayMicroseconds(minEnablePulseWidthUs); 
  digitalWrite(rangeEnablePinX_p, LOW);
  digitalWrite(rangeEnablePinX_m, LOW);
  
  delay(calibrationTimeMs); // wait calibration time
  
  // now Z-axis sensor readings are valid, so read now
  distanceInches_Z_plus = analogRead(SENSOR_Z_PLUS) * analogReadToInches;
  distanceInches_Z_minus = analogRead(SENSOR_Z_MINUS) * analogReadToInches;
  
  delay(minRangeReadTimeMs-calibrationTimeMs-calibrationTimeMs); // wait minimum cycle time, less twice the calibration time
  
  // trigger Y axis sensor(s)
  digitalWrite(rangeEnablePinY_p, HIGH);
  digitalWrite(rangeEnablePinY_m, HIGH);
  delayMicroseconds(minEnablePulseWidthUs); 
  digitalWrite(rangeEnablePinY_p, LOW);
  digitalWrite(rangeEnablePinY_m, LOW);
  
  delay(calibrationTimeMs); // wait calibration time

  // now X-axis sensor readings are valid, so read now
  distanceInches_X_plus = analogRead(SENSOR_X_PLUS) * analogReadToInches;
  distanceInches_X_minus = analogRead(SENSOR_X_MINUS) * analogReadToInches;

  delay(minRangeReadTimeMs-calibrationTimeMs-calibrationTimeMs); // wait minimum cycle time, less twice the calibration time
  
  // trigger Z axis sensor(s)
  digitalWrite(rangeEnablePinZ_p, HIGH);
  digitalWrite(rangeEnablePinZ_m, HIGH);
  delayMicroseconds(minEnablePulseWidthUs); 
  digitalWrite(rangeEnablePinZ_p, LOW);
  digitalWrite(rangeEnablePinZ_m, LOW);
  
  delay(calibrationTimeMs); // wait calibration time

  // now Y-axis sensor readings are valid, so read now
  distanceInches_Y_plus = analogRead(SENSOR_Y_PLUS) * analogReadToInches;
  distanceInches_Y_minus = analogRead(SENSOR_Y_MINUS) * analogReadToInches;

  delay(minRangeReadTimeMs-calibrationTimeMs-calibrationTimeMs);
 
  // print out the value you read:
//  Serial.print("Distance");
//  Serial.print(" ");
  Serial.print(distanceInches_X_plus/12.0,2);  // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_X_minus/12.0,2); // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_Y_plus/12.0,2);  // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_Y_minus/12.0,2); // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_Z_plus/12.0,2);  // 2 decimal places
  Serial.print(" ");
  Serial.println(distanceInches_Z_minus/12.0,2); // 2 decimal places

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
