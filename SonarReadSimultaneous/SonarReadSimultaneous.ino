/*
  SonarRead
  
  Read analog voltages on pins A0-A5, convert them to distances based on connections to the MB1360 
  sonar transducer sensors, and output them on the serial bus.
  
  The MB1360 is wired to the Arduino Uno in "analog voltage output" mode, such that the AN pin
  (pin 3) outputs a voltage proportional to the range of the first significant target it detects.
  The voltage is Vcc/(1024*2cm), which effectively constrains the range to 10m for 5V Vcc.
  
  The minimum time between range readings is 99ms. This sketch will ping all sensors simultaneously to
  test crosstalk issues. 
  
  Sensors are labeled Q0-Q3 corresponding to Cartesian quadrants 0-3 with respect to the camera
  housing being oriented on the +Y axis (between Q0 and Q1). 
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
#define SENSOR_Q0      A2
#define SENSOR_Q1      A5
#define SENSOR_Q2      A1
#define SENSOR_Q3      A0
#define SENSOR_Z_PLUS  A4
#define SENSOR_Z_MINUS A3


#define GPIO_PIN_OFFSET 2
/* The offsets should follow the sequence for the analog ports */
const int rangeEnablePinQ0 =  GPIO_PIN_OFFSET + 2;
const int rangeEnablePinQ1 =  GPIO_PIN_OFFSET + 5;
const int rangeEnablePinQ2 =  GPIO_PIN_OFFSET + 1;
const int rangeEnablePinQ3 =  GPIO_PIN_OFFSET + 0;
const int rangeEnablePinZ_p = GPIO_PIN_OFFSET + 4; 
const int rangeEnablePinZ_m = GPIO_PIN_OFFSET + 3;

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
  pinMode(rangeEnablePinQ0, OUTPUT);
  pinMode(rangeEnablePinQ1, OUTPUT);
  pinMode(rangeEnablePinQ2, OUTPUT);
  pinMode(rangeEnablePinQ3, OUTPUT);
  pinMode(rangeEnablePinZ_p, OUTPUT);
  pinMode(rangeEnablePinZ_m, OUTPUT);
  digitalWrite(rangeEnablePinQ0, LOW); // set low initially
  digitalWrite(rangeEnablePinQ1, LOW); // set low initially
  digitalWrite(rangeEnablePinQ2, LOW); // set low initially
  digitalWrite(rangeEnablePinQ3, LOW); // set low initially
  digitalWrite(rangeEnablePinZ_p, LOW); // set low initially
  digitalWrite(rangeEnablePinZ_m, LOW); // set low initially
  
  delay(50); // delay 50ms from power-up for the MB1360 sensor
}

// the loop routine runs over and over again forever:
void loop() 
{
  float distanceInches_Q0;
  float distanceInches_Q1;
  float distanceInches_Q2;
  float distanceInches_Q3;
  float distanceInches_Z_plus;
  float distanceInches_Z_minus;
  
  // trigger all sensor(s)
  digitalWrite(rangeEnablePinQ0, HIGH);
  digitalWrite(rangeEnablePinQ1, HIGH);
  digitalWrite(rangeEnablePinQ2, HIGH);
  digitalWrite(rangeEnablePinQ3, HIGH);
  digitalWrite(rangeEnablePinZ_p, HIGH);
  digitalWrite(rangeEnablePinZ_m, HIGH);
  delayMicroseconds(minEnablePulseWidthUs); 
  digitalWrite(rangeEnablePinQ0, LOW);
  digitalWrite(rangeEnablePinQ1, LOW);
  digitalWrite(rangeEnablePinQ2, LOW);
  digitalWrite(rangeEnablePinQ3, LOW);
  digitalWrite(rangeEnablePinZ_p, LOW);
  digitalWrite(rangeEnablePinZ_m, LOW);
  
  delay(minRangeReadTimeMs); // wait max range time
  
  // now sensor readings are valid, so read now
  distanceInches_Q0 = analogRead(SENSOR_Q0) * analogReadToInches;
  distanceInches_Q1 = analogRead(SENSOR_Q1) * analogReadToInches;
  distanceInches_Q2 = analogRead(SENSOR_Q2) * analogReadToInches;
  distanceInches_Q3 = analogRead(SENSOR_Q3) * analogReadToInches;
  distanceInches_Z_plus = analogRead(SENSOR_Z_PLUS) * analogReadToInches;
  distanceInches_Z_minus = analogRead(SENSOR_Z_MINUS) * analogReadToInches;
  
  // print out the value you read:
//  Serial.print("Distance");
//  Serial.print(" ");
  Serial.print(distanceInches_Q0/12.0,2);  // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_Q1/12.0,2); // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_Q2/12.0,2);  // 2 decimal places
  Serial.print(" ");
  Serial.print(distanceInches_Q3/12.0,2); // 2 decimal places
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
