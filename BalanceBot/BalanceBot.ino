/*Arduino Self Balancing Robot
 * Code by: B.Aswinth Raj
 * Build on top of Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 * Website: circuitdigest.com 
 */
#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TelnetSpy.h>

#define SERIAL_BAUD             115200

#define MOTOR_DRV_IN1           16
#define MOTOR_DRV_IN2           5
#define MOTOR_DRV_IN3           4
#define MOTOR_DRV_IN4           0
#define MOTOR_DRV_ENA           2
#define MOTOR_DRV_ENB           15

#define MPU_INTERRUPT_PIN       13

#define GYRO_OFFSET_X           104
#define GYRO_OFFSET_Y           101
#define GYRO_OFFSET_Z           66
#define ACCEL_OFFSET_Z          (int)1471

#define PID_SAMPLE_TIME         2  // in ms
#define PID_OUTPUT_LIMIT_LO     -1023
#define PID_OUTPUT_LIMIT_HI     1023

#define FIFO_BUFFER_SIZE        64


const char* ssid = "";
const char* password =  "";
#define UPRIGHT_VALUE           189.2 // value at the upright position
#define UPRIGHT_WINDOW          3
#define FALLING_THRESH_LO       (UPRIGHT_VALUE - UPRIGHT_WINDOW)
#define FALLING_THRESH_HI       (UPRIGHT_VALUE + UPRIGHT_WINDOW)
#define FALLEN_THRESH_FWD       250
#define FALLEN_THRESH_BACK      120

#define SETPOINT                UPRIGHT_VALUE  //set the value when the bot is perpendicular to ground using serial monitor. 
#define KP                      100 //21; //Set this first
#define KD                      0.75 //Set this secound
#define KI                      150 //Finally set this 

const char* host = "OTA-KEVIN";



TelnetSpy SERIALAndTelnet;

#define WIFI_SERIAL

#ifdef WIFI_SERIAL
  #define SERIAL  Serial //SERIALAndTelnet
#else
  #define SERIAL  Serial
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[FIFO_BUFFER_SIZE]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


double setpoint = UPRIGHT_VALUE; //set the value when the bot is perpendicular to ground using serial monitor. 

double input, output;
PID pid(&input, &output, &setpoint, KP, KI, KD, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void ICACHE_RAM_ATTR dmpDataReady();

void dmpDataReady()
{
    mpuInterrupt = true;
}

void waitForConnection() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SERIAL.print(".");
  }
  SERIAL.println(" Connected!");
}

void waitForDisconnection() {
  while (WiFi.status() == WL_CONNECTED) {
    delay(500);
    SERIAL.print(".");
  }
  SERIAL.println(" Disconnected!");
}

void telnetConnected() {
  SERIAL.println("Telnet connection established.");
}

void telnetDisconnected() {
  SERIAL.println("Telnet connection closed.");
}

bool isReady = false;

void setup() 
{
    // initialise serial
    SERIAL.begin(SERIAL_BAUD);

#ifdef WIFI_SERIAL
      SERIALAndTelnet.setWelcomeMsg("Welcome to the TelnetSpy example\n\n");
  SERIALAndTelnet.setCallbackOnConnect(telnetConnected);
  SERIALAndTelnet.setCallbackOnDisconnect(telnetDisconnected);
  SERIAL.setDebugOutput(false);
  SERIAL.print("\n\nConnecting to WiFi ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  waitForConnection();

  initOTA();
#endif
  
    
    // initialise device
    SERIAL.println(F("Initializing I2C devices..."));
    pinMode(MPU_INTERRUPT_PIN,INPUT_PULLUP);
    Wire.begin(12, 14);
    mpu.initialize();

     // verify connection
    SERIAL.println(F("Testing device connections..."));
    if(mpu.testConnection())
    {
      SERIAL.println("MPU6050 connection successful");
    }
    else
    {
      SERIAL.println("MPU6050 connection failed");
    }
    
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(GYRO_OFFSET_X);
    mpu.setYGyroOffset(GYRO_OFFSET_Y);
    mpu.setZGyroOffset(GYRO_OFFSET_Z);
    mpu.setZAccelOffset(ACCEL_OFFSET_Z); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        SERIAL.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        SERIAL.println(F("Enabling interrupt detection"));
        attachInterrupt(MPU_INTERRUPT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        SERIAL.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(PID_SAMPLE_TIME);
        pid.SetOutputLimits(PID_OUTPUT_LIMIT_LO, PID_OUTPUT_LIMIT_HI);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        SERIAL.print(F("DMP Initialization failed (code "));
        SERIAL.print(devStatus);
        SERIAL.println(F(")"));
    }

    // initialise the motor driver
    InitMotorDriver();
}

void loop() {    
    // if programming failed, don't try to do anything
    if (!dmpReady) 
    {
        return;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && (fifoCount < packetSize) && isReady)
    {      
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();   
        
        //Print the value of Input and Output on serial monitor to check how it is working.
        SERIAL.print(input); SERIAL.print(" =>"); SERIAL.println(output);

        //If the Bot is falling  
        if ((input < FALLING_THRESH_LO) || (input > FALLING_THRESH_HI))
        {
            // check if we have fallen over completely, if so lets not try and get back up
            if((input < FALLEN_THRESH_BACK) || (input > FALLEN_THRESH_FWD))
            {
              Stop();
            }
            else
            {
              // otherwise try and compensate for the angle that we are at
              if (output > 0) //Falling towards front 
                  Reverse(output); //Rotate the wheels forward 
              else if (output < 0) //Falling towards back
                  Forward(output * -1); //Rotate the wheels backward 
            }


        }
        else //If Bot not falling
        {
            Stop(); //Hold the wheels still
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        SERIAL.println(F("FIFO overflow!"));
        Stop();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

        input = ypr[1] * 180/M_PI + 180;
        isReady = true;
   }
#ifdef WIFI_SERIAL
     //SERIALAndTelnet.handle();
     ArduinoOTA.handle();
#endif

}

void InitMotorDriver()
{
    //Initialise the Motor outpu pins
    pinMode (MOTOR_DRV_IN4, OUTPUT);
    pinMode (MOTOR_DRV_IN3, OUTPUT);
    pinMode (MOTOR_DRV_IN2, OUTPUT);
    pinMode (MOTOR_DRV_IN1, OUTPUT);
    pinMode (MOTOR_DRV_ENA, OUTPUT);
    pinMode (MOTOR_DRV_ENB, OUTPUT);

    //By default turn off both the motors
    digitalWrite(MOTOR_DRV_IN4,LOW);
    digitalWrite(MOTOR_DRV_IN3,LOW);
    digitalWrite(MOTOR_DRV_IN2,LOW);
    digitalWrite(MOTOR_DRV_IN1,LOW);
    analogWrite(MOTOR_DRV_ENA,LOW);
    analogWrite(MOTOR_DRV_ENB,LOW);
}

void Forward(int drive) //Code to rotate the wheel forward 
{
    digitalWrite(MOTOR_DRV_IN4,HIGH);
    digitalWrite(MOTOR_DRV_IN3,LOW);
    digitalWrite(MOTOR_DRV_IN2,HIGH);
    digitalWrite(MOTOR_DRV_IN1,LOW);
    analogWrite(MOTOR_DRV_ENB,drive);
    analogWrite(MOTOR_DRV_ENA,drive);
    
    SERIAL.print("F"); //Debugging information 
}

void Reverse(int drive) //Code to rotate the wheel Backward  
{
    digitalWrite(MOTOR_DRV_IN4,LOW);
    digitalWrite(MOTOR_DRV_IN3,HIGH);
    digitalWrite(MOTOR_DRV_IN2,LOW);
    digitalWrite(MOTOR_DRV_IN1,HIGH);
    analogWrite(MOTOR_DRV_ENB,drive);
    analogWrite(MOTOR_DRV_ENA,drive);
    
    
    SERIAL.print("R");
}

void Stop() //Code to stop both the wheels
{
    digitalWrite(MOTOR_DRV_IN4,LOW);
    digitalWrite(MOTOR_DRV_IN3,LOW);
    digitalWrite(MOTOR_DRV_IN2,LOW);
    digitalWrite(MOTOR_DRV_IN1,LOW);
    analogWrite(MOTOR_DRV_ENB,0);
    analogWrite(MOTOR_DRV_ENA,0);
    
    SERIAL.print("S");
}

void initOTA()
{
  ArduinoOTA.setHostname(host);
  ArduinoOTA.onStart([]() {
    String type;
    Stop(); // stop the motors
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
