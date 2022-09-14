/*
  Azure IoT for Arduino MKR NB 1500

  Author: Matt Sinclair (@mjksinc)

  This sketch securely connects to either Azure IoT Hub or IoT Central using MQTT over NB-IoT/Cat-M1.
  The native NBSSL library is used to securley connect to the Hub, then Username/Password credentials
  are used to authenticate.

  BEFORE USING:
  - Ensure that SECRET_BROKER and CONN_STRING in arduino_secrets.h are completed
  - Change msgFreq as desired
  - Check ttl to change the life of the SAS Token for authentication with IoT Hub

  If using IoT Central:
  - Follow these intructions to find the connection details for a real device: https://docs.microsoft.com/en-us/azure/iot-central/tutorial-add-device#get-the-device-connection-information
  - Generate a connection string from this website: https://dpscstrgen.azurewebsites.net/

  Full Intructions available here: https://github.com/mjksinc/TIC2019

*/

// Libraries to include in the code
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <ArduinoMqttClient.h>
#include <MKRNB.h>
#include "./base64.h"
#include "./Sha256.h"
#include "./utils.h"

//Temperature
#include <Arduino.h>
#include <math.h>

//GPS
#define GPSSerial Serial1
#include <TinyGPS.h>


// Additional file secretly stores credentials
#include "arduino_secrets.h"

// Enter your sensitive data in arduino_secrets.h
const char broker[] = SECRET_BROKER;
// CONN_STRING: connection string from Hub;

String iothubHost;
String deviceId;
String sharedAccessKey;

int msgFreq = 20000; //Message Frequency in millisecods
long ttl = 864000; //Time-to-live for SAS Token (seconds) i.e. 864000 = 1 day (24 hours)

MAX30105 particleSensor;
NB nbAccess;
GPRS gprs;
NBSSLClient sslClient;
MqttClient mqttClient(sslClient);
NBScanner scannerNetworks;
TinyGPS gps;//Declaramos el objeto gps

unsigned long lastMillis = 0;

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

//*** VARIABLES OBTENCIÓN DE DATOS GPS****//
float lat= 0.0;
float lon= 0.0;
float alt= 0.0;

/*
 * Establish connection to cellular network, and parse/augment connection string to generate credentials for MQTT connection
 * This only allocates the correct variables, connection to the IoT Hub (MQTT Broker) happens in loop()
 */
void setup() {
  Wire.begin(); // Wire communication begin
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  while (!Serial); // Waiting for Serial Monitor
  
  GPSSerial.begin(9600);
  //pinMode(LED_BUILTIN, OUTPUT);
  analogReadResolution(12); //LMT86 needes 12 bits of resolution
  
 
  Serial.println("******Splitting Connection String - STARTED*****");
  splitConnectionString();

  //Connects to network to use getTime()
  connectNB();
  

  Serial.println("******Create SAS Token - STARTED*****");
  // create SAS token and user name for connecting to MQTT broker
  String url = iothubHost + urlEncode(String("/devices/" + deviceId).c_str());
  char *devKey = (char *)sharedAccessKey.c_str();
  long expire = getTime() + ttl;
  String sasToken = createIotHubSASToken(devKey, url, expire);
  String username = iothubHost + "/" + deviceId + "/api-version=2018-06-30";

  Serial.println("******Create SAS Token - COMPLETED*****");
  
  // Set the client id used for MQTT as the device id
  mqttClient.setId(deviceId);
  mqttClient.setUsernamePassword(username, sasToken);
  
  // Set the message callback, this function is
  // called when the MQTTClient receives a message
  mqttClient.onMessage(onMessageReceived);
  //Serial.println("Breakpoint");

   //**************Initialize sensor**************************//
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
  Serial.println("Leyendo primeras 100 muestras y determinando el rango de la señal....");
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

//    Serial.print(F("red="));
//    Serial.print(redBuffer[i], DEC);
//    Serial.print(F(", ir="));
//    Serial.println(irBuffer[i], DEC);
  }
  Serial.println("Finalizada lectura de las 100 primeras muestras");
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

/*
 * Connect to Network (if not already connected) and establish connection the IoT Hub (MQTT Broker). Messages will be sent every 30 seconds, and will poll for new messages
 * on the "devices/{deviceId}/messages/devicebound/#" topic
 * This also calls publishMessage() to trigger the message send
 */
void loop() {
  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
    connectNB();
  }

  if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    connectMQTT();
  }

  // poll for new MQTT messages and send keep alives
  mqttClient.poll();

  // publish a message roughly every 30 seconds.
  if (millis() - lastMillis > msgFreq) {
    lastMillis = millis();

    publishMessage();
  }
}

/*
 * Gets current Linux Time in seconds for enabling timing of SAS Token
 */
unsigned long getTime() {
  // get the current time from the cellular module
  return nbAccess.getTime();
}

/*
 * Handles the connection to the NB-IoT Network
 */
void connectNB() {
  Serial.println("\n******Connecting to Cellular Network - STARTED******");

  while ((nbAccess.begin() != NB_READY) ||
         (gprs.attachGPRS() != GPRS_READY)) {
    // failed, retry
    Serial.print(".");
    delay(1000);
  }

  Serial.println("******Connecting to Cellular Network - COMPLETED******");
  Serial.println();
}

/*
 * Establishses connection with the MQTT Broker (IoT Hub)
 * Some errors you may receive:
 * -- (-.2) Either a connectivity error or an error in the url of the broker
 * -- (-.5) Check credentials - has the SAS Token expired? Do you have the right connection string copied into arduino_secrets?
 */
void connectMQTT() {
  Serial.print("Attempting to connect to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");

  while (!mqttClient.connect(broker, 8883)) {
    // failed, retry
    Serial.print(".");
    Serial.println(mqttClient.connectError());
    delay(5000);
    if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
    connectNB();
    }
    //Serial.println("Hola");
  }
  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe("devices/" + deviceId + "/messages/devicebound/#"); //This is for cloud-to-device messages
  mqttClient.subscribe("$iothub/methods/POST/#"); //This is for direct methods + IoT Central commands
  
}

/*
 * Calls getMeasurement() to read sensor measurements (currently simulated)
 * Prints message to the MQTT Client
 */
void publishMessage() {
  Serial.println("Publishing message");

  String newMessage = getMeasurement();

  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
  mqttClient.print(newMessage);
  mqttClient.endMessage();
}

/*
 * Creates the measurements. This currently simulates and structures the data. Any sensor-reading functions would be placed here
 */
String getMeasurement() {

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
        lat=auxlat;
      }
      if(auxlong!=0.00){
        lon=auxlong;
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

      //digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }
    
    if(validHeartRate==1){
      finalheartRate=heartRate;
    }
    else{
      finalheartRate=999;
    }


    if(validSPO2==1){
      finalspo2=spo2;
    } 
    else{
      finalspo2=999;
    }
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
//************************END****************//

  // Begin network scan to get signal strength
  //scannerNetworks.begin();

  //String signalStrength = scannerNetworks.getSignalStrength();

    String formattedMessage = "{\"temperature\": ";
  formattedMessage += temp;
//  formattedMessage += ", \"latitud\": ";
//  formattedMessage += lat;
//  formattedMessage += ", \"longitud\": ";
//  formattedMessage += lon;

  formattedMessage += ", \"DeviceLocation\":{";
    formattedMessage += ", \"lat\": ";
    formattedMessage += lat;
    formattedMessage += ", \"lon\": ";
    formattedMessage += lon;
    formattedMessage += ", \"alt\": ";
    formattedMessage += alt;
    
  formattedMessage += "},\"heartrate\": ";
  formattedMessage += finalheartRate;
  formattedMessage += ", \"SPO2\": ";
  formattedMessage += finalspo2;
  formattedMessage += "}";

  Serial.println(formattedMessage);
  return formattedMessage;
}

/*
 * Handles the messages received through the subscribed topic and prints to Serial
 */
void onMessageReceived(int messageSize) {

  String topic = mqttClient.messageTopic();
  
  // when receiving a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(topic);
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
  }
  Serial.println();

  // Responds with confirmation to direct methods and IoT Central commands 
  if (topic.startsWith(F("$iothub/methods"))) {
    String msgId = topic.substring(topic.indexOf("$rid=") + 5);

    String responseTopic = "$iothub/methods/res/200/?$rid=" + msgId; //Returns a 200 received message

    mqttClient.beginMessage(responseTopic);
    mqttClient.print("");
    mqttClient.endMessage(); 
  }

  
}

/*
 * Split the connection string into individual parts to use as part of MQTT connection setup
 */
void splitConnectionString() {
    String connStr = CONN_STRING;
    int hostIndex = connStr.indexOf("HostName=");
    int deviceIdIndex = connStr.indexOf(F(";DeviceId="));
    int sharedAccessKeyIndex = connStr.indexOf(";SharedAccessKey=");
    iothubHost = connStr.substring(hostIndex + 9, deviceIdIndex);
    deviceId = connStr.substring(deviceIdIndex + 10, sharedAccessKeyIndex);
    sharedAccessKey = connStr.substring(sharedAccessKeyIndex + 17);
    Serial.print("******Splitting Connection String - COMPLETED*****");
}

/*
 * Build a SAS Token to be used as the MQTT authorisation password
 */
String createIotHubSASToken(char *key, String url, long expire){
    url.toLowerCase();
    String stringToSign = url + "\n" + String(expire);
    int keyLength = strlen(key);

    int decodedKeyLength = base64_dec_len(key, keyLength);
    char decodedKey[decodedKeyLength];

    base64_decode(decodedKey, key, keyLength);

    Sha256 *sha256 = new Sha256();
    sha256->initHmac((const uint8_t*)decodedKey, (size_t)decodedKeyLength);
    sha256->print(stringToSign);
    char* sign = (char*) sha256->resultHmac();
    int encodedSignLen = base64_enc_len(HASH_LENGTH);
    char encodedSign[encodedSignLen];
    base64_encode(encodedSign, sign, HASH_LENGTH);
    delete(sha256);

    return "SharedAccessSignature sr=" + url + "&sig=" + urlEncode((const char*)encodedSign) + "&se=" + String(expire);
}

/*
 * Añadido: adquisición de los diferentes valores
*/

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
  //Serial.print(temp);
  //Serial.println(" deg C");
  return temp;
}
