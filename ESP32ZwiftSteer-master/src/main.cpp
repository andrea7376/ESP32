#include "Adafruit_MLX90393.h"
#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//DEBUG
#define DEBUG

//BLE Definitions
#define STEERING_DEVICE_UUID "347b0001-7635-408b-8918-8ff3949ce592"
#define STEERING_ANGLE_CHAR_UUID "347b0030-7635-408b-8918-8ff3949ce592" //notify
#define STEERING_RX_CHAR_UUID "347b0031-7635-408b-8918-8ff3949ce592"    //write
#define STEERING_TX_CHAR_UUID "347b0032-7635-408b-8918-8ff3949ce592"    //indicate

#define POT 32 // Joystick Xaxis to GPIO32

//Angle calculation parametres
//#define MAX_ADC_RESOLUTION 4095 //ESP32 ADC is 12bit
//#define MAX_STEER_ANGLE 35
//#define ZERO_FLOOR 2

#define MAX_ADC_RESOLUTION 190 //ESP32 ADC is 12bit
#define MAX_STEER_ANGLE 35
#define OFF_SET 52
#define ZERO_FLOOR 2

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool auth = false;

float angle = 0;
float x,y,z;
float angle_deviation = 0; //Joystick Calibration
//Sterzo stuff
int FF = 0xFF;
uint8_t authChallenge[4] = {0x03, 0x10, 0xff, 0xff};
uint8_t authSuccess[3] = {0x03, 0x11, 0xff};

BLEServer *pServer = NULL;
BLECharacteristic *pAngle = NULL;
BLECharacteristic *pRx = NULL;
BLECharacteristic *pTx = NULL;
BLEAdvertising *pAdvertising;

Adafruit_MLX90393 sensor = Adafruit_MLX90393();
#define MLX90393_CS 10
sensors_event_t event;


//Server Callbacks
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        BLEDevice::startAdvertising();
        
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

//Characteristic Callbacks
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{

    void onRead(BLECharacteristic *pRx)
    {

    }

    void onWrite(BLECharacteristic *pRx){

        std::string rxValue = pRx->getValue();
        
        if(rxValue.length() == 4){
          delay(250);
          pTx->setValue(authSuccess,3);
          pTx->indicate();
          auth = true;
          #ifdef DEBUG
          Serial.println("Auth Success!");
          #endif
        }
    }
};

//Joystick read angle from axis
float readAngle()
{
   /* int potVal = analogRead(POT);
    #ifdef DEBUG
    Serial.println(potVal - OFF_SET);
    #endif
    /* Old Style calc.
    angle = map(potVal,0,4095,-35,35); //Mapping function
    */
    
    // kwakeham style:
    /*
    angle = (((potVal - OFF_SET) / (float)MAX_ADC_RESOLUTION) * (MAX_STEER_ANGLE * 2)) - MAX_STEER_ANGLE;
    */
    
    //sensor.readMeasurement(&x, &y, &z);
    sensor.getEvent(&event);
    angle = 180*atan2(event.orientation.y, event.orientation.x)/3.14159265358;

    /*if (fabsf(angle) < ZERO_FLOOR){
        angle = 0;
    }*/
   
    return angle - angle_deviation;
}



void setup(void)
{
  Serial.begin(115200);

  /* Wait for serial on USB platforms. */
  while (!Serial) {
      delay(10);
  }

  Serial.println("Starting Adafruit MLX90393 Demo");

  if (! sensor.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! sensor.begin_SPI(MLX90393_CS)) {  // hardware SPI mode
    Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }
  Serial.println("Found a MLX90393 sensor");

  sensor.setGain(MLX90393_GAIN_2X);
  // You can check the gain too
  Serial.print("Gain set to: ");
  switch (sensor.getGain()) {
    case MLX90393_GAIN_1X: Serial.println("1 x"); break;
    case MLX90393_GAIN_1_33X: Serial.println("1.33 x"); break;
    case MLX90393_GAIN_1_67X: Serial.println("1.67 x"); break;
    case MLX90393_GAIN_2X: Serial.println("2 x"); break;
    case MLX90393_GAIN_2_5X: Serial.println("2.5 x"); break;
    case MLX90393_GAIN_3X: Serial.println("3 x"); break;
    case MLX90393_GAIN_4X: Serial.println("4 x"); break;
    case MLX90393_GAIN_5X: Serial.println("5 x"); break;
  }

  // Set resolution, per axis
  sensor.setResolution(MLX90393_X, MLX90393_RES_19);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_19);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  sensor.setOversampling(MLX90393_OSR_2);

  // Set digital filtering
  sensor.setFilter(MLX90393_FILTER_6);


    angle_deviation = readAngle();
   
    //Setup BLE
    #ifdef DEBUG
    Serial.println("Creating BLE server...");
    #endif
    BLEDevice::init("STEERING");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    #ifdef DEBUG
    Serial.println("Define service...");
    #endif
    BLEService *pService = pServer->createService(STEERING_DEVICE_UUID);

    // Create BLE Characteristics
    #ifdef DEBUG
    Serial.println("Define characteristics");
    #endif
    pTx = pService->createCharacteristic(STEERING_TX_CHAR_UUID, BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_READ);
    pTx->addDescriptor(new BLE2902());
    
    pRx = pService->createCharacteristic(STEERING_RX_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    pRx->addDescriptor(new BLE2902());
    pRx->setCallbacks(new MyCharacteristicCallbacks());

    pAngle = pService->createCharacteristic(STEERING_ANGLE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pAngle->addDescriptor(new BLE2902());

    // Start the service
    #ifdef DEBUG
    Serial.println("Staring BLE service...");
    #endif
    pService->start();

    // Start advertising
    #ifdef DEBUG
    Serial.println("Define the advertiser...");
    #endif
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);
    pAdvertising->addServiceUUID(STEERING_DEVICE_UUID);
    pAdvertising->setMinPreferred(0x06); // set value to 0x00 to not advertise this parameter
    #ifdef DEBUG
    Serial.println("Starting advertiser...");
    #endif
    BLEDevice::startAdvertising();
    #ifdef DEBUG
    Serial.println("Waiting a client connection to notify...");
    #endif

}

void loop(void) {
  


if (deviceConnected)
    {    
        
        if(auth){
      
          angle = readAngle();
          pAngle->setValue(angle);
          pAngle->notify();
          #ifdef DEBUG
          Serial.print("TX Angle: ");
          Serial.println(angle);
          #endif
          delay(250);
        } else {
          #ifdef DEBUG
          Serial.println("Auth Challenging");
          #endif
       
          pTx->setValue(authChallenge, 4);
          pTx->indicate();
          delay(250);
        
        }
    }

    //Advertising
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(300);                  // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        #ifdef DEBUG
        Serial.println("Nothing connected, start advertising");
        #endif
        oldDeviceConnected = deviceConnected;
    }
   
    //Connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        oldDeviceConnected = deviceConnected;
        #ifdef DEBUG
        Serial.println("Connecting...");
        #endif
    }

    {
        //Nothing
    }


/*
  // get X Y and Z data at once
  if (sensor.readData(&x, &y, &z)) {
      Serial.print("X: "); Serial.print(x, 4); Serial.println(" uT");
      Serial.print("Y: "); Serial.print(y, 4); Serial.println(" uT");
      Serial.print("Z: "); Serial.print(z, 4); Serial.println(" uT");
  } else {
      Serial.println("Unable to read XYZ data from the sensor.");
  }

  delay(500);
*/
  /* Or....get a new sensor event, normalized to uTesla */
 // sensors_event_t event;
  //sensor.getEvent(&event);
  /* Display the results (magnetic field is measured in uTesla) */
 

 /* Serial.print("X: "); Serial.print(event.orientation.x);
  Serial.print(" \tY: "); Serial.print(event.orientation.y);
  Serial.print(" \tZ: "); Serial.print(event.orientation.z);
  an = 180*atan2(event.orientation.y, event.orientation.x)/3.14;

  Serial.print(" \tangolo: ");Serial.println(an);


  delay(250);*/
}