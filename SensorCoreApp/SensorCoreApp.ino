//-------------------------------------------------------------------------------
//  TinyCircuits ST BLE TinyShield UART Example Sketch
//  Last Updated 2 March 2016
//
//  This demo sets up the BlueNRG-MS chipset of the ST BLE module for compatiblity
//  with Nordic's virtual UART connection, and can pass data between the Arduino
//  serial monitor and Nordic nRF UART V2.0 app or another compatible BLE
//  terminal. This example is written specifically to be fairly code compatible
//  with the Nordic NRF8001 example, with a replacement UART.ino file with
//  'aci_loop' and 'BLEsetup' functions to allow easy replacement.
//
//  Written by Ben Rose, TinyCircuits http://tinycircuits.com
//
//-------------------------------------------------------------------------------

#include <Wire.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#ifndef ARDUINO_ARCH_SAMD
#include <EEPROM.h>
#endif

RTIMU *imu;             // the IMU object
RTFusionRTQF fusion;    // the fusion object
RTIMUSettings settings; // the settings object
//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL 150 // interval between pose displays
//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define SERIAL_PORT_SPEED 115200
unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

#include <SPI.h>
#include <STBLE.h>

//Debug output adds extra flash and memory requirements!
#ifndef BLE_DEBUG
#define BLE_DEBUG true
#endif

#if defined(ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

#define buffer_size 21 // must be 21?

uint8_t ble_rx_buffer[buffer_size];
uint8_t ble_rx_buffer_len = 0;
uint8_t ble_connection_state = false;
#define PIPE_UART_OVER_BTLE_UART_TX_TX 0

void setup()
{
  int errcode;

  SerialMonitorInterface.begin(SERIAL_PORT_SPEED);
  while (!SerialMonitorInterface)
    ; //This line will block until a serial monitor is opened with TinyScreen+!
  BLEsetup();

  Wire.begin();
  imu = RTIMU::createIMU(&settings); // create the imu object

  SerialMonitorInterface.print("ArduinoIMU starting using device ");
  SerialMonitorInterface.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0)
  {
    SerialMonitorInterface.print("Failed to init IMU: ");
    SerialMonitorInterface.println(errcode);
  }

  if (imu->getCalibrationValid())
    SerialMonitorInterface.println("Using compass calibration");
  else
    SerialMonitorInterface.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.

  fusion.setSlerpPower(0.02);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor

  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
}

void loop()
{
  unsigned long now = millis();
  unsigned long delta;

  aci_loop(); //Process any ACI commands or events from the NRF8001- main BLE handler, must run often. Keep main loop short.
  if (ble_rx_buffer_len)
  { //Check if data is available
    SerialMonitorInterface.print(ble_rx_buffer_len);
    SerialMonitorInterface.print(" : ");
    SerialMonitorInterface.println((char *)ble_rx_buffer);
    ble_rx_buffer_len = 0; //clear afer reading
  }
  if (SerialMonitorInterface.available())
  {            //Check if serial input is available to send
    delay(10); //should catch input
    uint8_t sendBuffer[buffer_size];
    uint8_t sendLength = 0;
    while (SerialMonitorInterface.available() && sendLength < (buffer_size - 2))
    {
      sendBuffer[sendLength] = SerialMonitorInterface.read();
      SerialMonitorInterface.print(sendBuffer[sendLength]);
      sendLength++;
    }
    SerialMonitorInterface.println();

    if (SerialMonitorInterface.available())
    {
      SerialMonitorInterface.print(F("Input truncated, dropped: "));
      if (SerialMonitorInterface.available())
      {
        SerialMonitorInterface.write(SerialMonitorInterface.read());
      }
    }
    sendBuffer[sendLength] = '\0'; //Terminate string
    sendLength++;
    if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t *)sendBuffer, sendLength))
    {
      //SerialMonitorInterface.println(F("TX dropped!"));
    }
    else
    {
      //SerialMonitorInterface.println((char *)sendBuffer);
    }
  }

  if (imu->IMURead())
  { // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    /*if ((delta = now - lastRate) >= 1000)
    {
      SerialMonitor.print("Sample rate: "); SerialMonitor.print(sampleCount);
      if (imu->IMUGyroBiasValid())
        SerialMonitor.println(", gyro bias valid");
      else
        SerialMonitor.println(", calculating gyro bias - don't move IMU!!");

      sampleCount = 0;
      lastRate = now;
    }*/
    if ((delta = now - lastRate) >= 900)
    {
      sampleCount = 0;
      lastRate = now;
    }

    if ((now - lastDisplay) >= DISPLAY_INTERVAL)
    {
      lastDisplay = now;
      RTVector3 accelData = imu->getAccel();
      RTVector3 gyroData = imu->getGyro();
      RTVector3 compassData = imu->getCompass();
      RTVector3 fusionData = fusion.getFusionPose();

      SerialMonitorInterface.print("C");
      SerialMonitorInterface.print(sampleCount);
      SerialMonitorInterface.print("S0");
      displayAxis(accelData.x(), accelData.y(), accelData.z());       // accel data
      displayAxis(gyroData.x(), gyroData.y(), gyroData.z());          // gyro data
      displayAxis(compassData.x(), compassData.y(), compassData.z()); // compass data
      SerialMonitorInterface.print("F");
      displayDegrees(fusionData.x(), fusionData.y(), fusionData.z()); // fused output
      SerialMonitorInterface.println();

      String msg = "";
      msg.concat("C");
      msg.concat(sampleCount);
      msg.concat("S0");
      msg.concat(accelData.x());
      msg.concat(accelData.y());
      msg.concat(accelData.z());
      msg.concat(gyroData.x());
      msg.concat(gyroData.y());
      msg.concat(gyroData.z());
      msg.concat(compassData.x());
      msg.concat(compassData.y());
      msg.concat(compassData.z());
      msg.concat("F");
      msg.concat(fusionData.x());
      msg.concat(fusionData.y());
      msg.concat(fusionData.z());

      //msg.concat("T3260S0x-0.21y-0.90z0.32x0.01y0.00z-0.01x450.45y-247.57z-450.43Fx-70.23y12.19z47.91");
      //msg.concat("message");

      //SerialMonitorInterface.println(msg);

      //uint8_t *sendBuffer[];
      uint8_t sendBuffer[msg.length() + 1];
      //sendBuffer = msg.toCharArray();
      msg.getBytes(sendBuffer, msg.length() + 1);

      //uint8_t sendBuffer[16] = {66, 127, 164, 72, 68, 158, 148, 255, 23, 36, 226, 192, 198, 248, 251, 221};
      /*uint8_t sendBuffer[10] = {0};
      sendBuffer[0] = 'A';
      sendBuffer[1] = 'B';
      sendBuffer[2] = 'C';
      sendBuffer[3] = 'D';
      sendBuffer[4] = 'E';
      sendBuffer[5] = 'F';
      sendBuffer[6] = 'G';
      sendBuffer[7] = 'H';
      sendBuffer[8] = 'I';
      sendBuffer[9] = 'J';
*/
      //SerialMonitorInterface.println((char *)sendBuffer);

      uint8_t sentLength = 0;
      int sizeSendBuffer = sizeof(sendBuffer);
      while (sentLength < sizeSendBuffer)
      {
        int arraySize = buffer_size - 2;
        if ((sizeSendBuffer > sentLength) && ((sizeSendBuffer - sentLength) < (buffer_size - 2)))
        {
          arraySize = (sizeSendBuffer - sentLength);
        }
        uint8_t sendBufferTruncated[arraySize] = {};

        //SerialMonitorInterface.println((char *)sendBufferTruncated);

        uint8_t sendLength = 0;
        while (sendLength < arraySize)
        {
          sendBufferTruncated[sendLength] = sendBuffer[sentLength];
          sendLength++;
          sentLength++;
        }
        sendBufferTruncated[sendLength] = '\0'; //Terminate string
        sendLength++;
        //SerialMonitorInterface.print((char *)sendBufferTruncated);

        lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t *)sendBufferTruncated, sendLength);
        //SerialMonitorInterface.println();
      }

      /*if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t *)sendBuffer, sendLength))
      {
        SerialMonitorInterface.println(F("TX dropped!"));
      }
      else
      {
        SerialMonitorInterface.println((char *)sendBuffer);
      }*/

      /*if (SerialMonitorInterface.available())
      {            //Check if serial input is available to send
        delay(10); //should catch input
        uint8_t sendBuffer[21];
        uint8_t sendLength = 0;
        while (SerialMonitorInterface.available() && sendLength < 19)
        {
          sendBuffer[sendLength] = SerialMonitorInterface.read();
          sendLength++;
        }
        if (SerialMonitorInterface.available())
        {
          SerialMonitorInterface.print(F("Input truncated, dropped: "));
          if (SerialMonitorInterface.available())
          {
            SerialMonitorInterface.write(SerialMonitorInterface.read());
          }
        }
        sendBuffer[sendLength] = '\0'; //Terminate string
        sendLength++;
        if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t *)sendBuffer, sendLength))
        {
          SerialMonitorInterface.println(F("TX dropped!"));
        }
        else
        {
          SerialMonitorInterface.println((char *)sendBuffer);
        }
      }*/
    }
  }
}

void displayAxis(float x, float y, float z)
{
  //SerialMonitorInterface.print("x");
  SerialMonitorInterface.print(x);
  //SerialMonitorInterface.print("y");
  SerialMonitorInterface.print(y);
  //SerialMonitorInterface.print("z");
  SerialMonitorInterface.print(z);
}

void displayDegrees(float x, float y, float z)
{
  /*
  //SerialMonitorInterface.print("x");
  SerialMonitorInterface.print(x * RTMATH_RAD_TO_DEGREE);
  //SerialMonitorInterface.print("y");
  SerialMonitorInterface.print(y * RTMATH_RAD_TO_DEGREE);
  //SerialMonitorInterface.print("z");
  SerialMonitorInterface.print(z * RTMATH_RAD_TO_DEGREE);
  */
  //SerialMonitorInterface.print("x");
  SerialMonitorInterface.print(x);
  //SerialMonitorInterface.print("y");
  SerialMonitorInterface.print(y);
  //SerialMonitorInterface.print("z");
  SerialMonitorInterface.print(z);
}
