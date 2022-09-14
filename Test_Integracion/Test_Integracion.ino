#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#define GPSSerial Serial1
#include <TinyGPS.h>

#include <Arduino.h>
#include <math.h>

MAX30105 particleSensor;

TinyGPS gps;//Declaramos el objeto gps

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength=100; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

int32_t finalspo2=0;
int32_t finalheartRate=0;

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read


  
//*** VARIABLES OBTENCIÓN DE DATOS GPS****//
float latitud= 0.0;
float longitud= 0.0;
//int year;
//byte month, day, hour, minute, second, hundredths;
//unsigned long chars;
//unsigned short sentences, failed_checksum;


void setup()
{
  Wire.begin(); // Wire communication begin
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  while (!Serial); // Waiting for Serial Monitor

  GPSSerial.begin(9600);
  
  analogReadResolution(12); //LMT86 needes 12 bits of resolution

//  pinMode(pulseLED, OUTPUT);
//  pinMode(readLED, OUTPUT);
//
   //Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
    
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

void loop()
{
  float temp= getTemperature();


  //****************** LATITUDE AND LONGITUDE***************************//
  while (GPSSerial.available()) {
    //Serial.println("leyendo...");
    int c = GPSSerial.read();

    if(gps.encode(c))  
    {
      float auxlat, auxlong;
      gps.f_get_position(&auxlat, &auxlong);

      if(auxlat!=0.00){
        latitud=auxlat;
      }
      if(auxlong!=0.00){
        longitud=auxlong;
      }             
    }

  }
  //***************** END LATITUDE AND LONGITUDE******************//
  
//  //**************** GET HR AND SP02************************//
      //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
//      Serial.print(F("red="));
//      Serial.print(redBuffer[i], DEC);
//      Serial.print(F(", ir="));
//      Serial.print(irBuffer[i], DEC);
//
//      Serial.print(F(", HR="));
//      Serial.print(heartRate, DEC);
//
//      Serial.print(F(", HRvalid="));
//      Serial.print(validHeartRate, DEC);
//
//      Serial.print(F(", SPO2="));
//      Serial.print(spo2, DEC);
//
//      Serial.print(F(", SPO2Valid="));
//      Serial.println(validSPO2, DEC);
    }
    
    if(validHeartRate==1){
      finalheartRate=heartRate;
    }

    if(validSPO2==1){
      finalspo2=spo2;
    } 
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
//************************END****************//

  
  String formattedMessage = "{\"temperature\": ";
  formattedMessage += temp;
  formattedMessage += ", \"latitud\": ";
  formattedMessage += latitud;
  formattedMessage += ", \"longitud\": ";
  formattedMessage += longitud;
  
  formattedMessage += ", \"heart rate\": ";
  formattedMessage += finalheartRate;
  formattedMessage += ", \"SPO2\": ";
  formattedMessage += finalspo2;
  formattedMessage += "}";




  Serial.println(formattedMessage);
  delay(1000);
}

float f(float t, float v) {
    return 1777.3F - (10.888*(t-30)) - 0.00347*(t-30.0F)*(t-30.0F) - v;
}

float df(float t) {
    return -10.888*t - 0.00347*2*(t-30.0F);
}

float nstep(float t, float v) {
    return t - f(t,v) / df(t);
}

#define TOLERANCE 0.005

float newton(float v) {
    float t0 = 30.0F;
    float t;
    uint16_t n = 0;
    while(true) {
        t = nstep(t0, v);
        n++;
        if(fabs(t-t0) < TOLERANCE)
            break;
        t0 = t;
    }
//    Serial.print("loop ");
//    Serial.println(n);
    return t;
}

float getTemperature(){
  int promedio =40;
  float temp=0.0;
  uint16_t val;
  float mv;
  float aux=0;
  for(int i=0; i<promedio; i++)
  {
    val = analogRead(0);
    mv = 3.3 * (float) val * 1000.0 / 4096.0;
    aux= newton(mv);
    temp=temp + aux;
  }
  temp=temp/promedio;
//  Serial.print(temp);
//  Serial.println(" deg C");
  return temp;
}
