/**
  ******************************************************************************
  * @file    neuton_gesturerecognition.ino
  * @author  Leonardo Cavagnis
  * @brief   A Gesture Recognition system (binary classification) with Neuton TinyML
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <stdint.h>

#include "neuton.h"

/* Private define ------------------------------------------------------------*/
#define NUM_SAMPLES         50
#define G                   9.80665f
#define ACC_THRESHOLD       (2.5f*G)          // threshold of significant in G's
#define GESTURE_ARRAY_SIZE  (6*NUM_SAMPLES+1) // 6 measurements (a,g)/sample + target

/* Private variables ---------------------------------------------------------*/
int   samplesRead                       = NUM_SAMPLES;
float gestureArray[GESTURE_ARRAY_SIZE]  = {0};

Adafruit_MPU6050 mpu;

void setup() {
  // init serial port
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // init IMU sensor
  if (!mpu.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      delay(10);
    }
  }

  // configure IMU sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Neuton neural network model: Gesture recognition system");
}

void loop() {
  sensors_event_t a, g, temp;
  
  // wait for significant motion
  while (samplesRead == NUM_SAMPLES) {
    // read the acceleration data
    mpu.getEvent(&a, &g, &temp);
    
    // sum up the absolutes
    float aSum = fabs(a.acceleration.x) + fabs(a.acceleration.y) + fabs(a.acceleration.z);
    
    // check if it's above the threshold
    if (aSum >= ACC_THRESHOLD) {
      // reset the sample read count
      samplesRead = 0;
      break;
    }
  }

  // read samples of the detected motion
  while (samplesRead < NUM_SAMPLES) {
    // read the acceleration and gyroscope data
    mpu.getEvent(&a, &g, &temp);

    // fill gesture array (model input)
    gestureArray[samplesRead*6 + 0] = a.acceleration.x;
    gestureArray[samplesRead*6 + 1] = a.acceleration.y;
    gestureArray[samplesRead*6 + 2] = a.acceleration.z;
    gestureArray[samplesRead*6 + 3] = g.gyro.x;
    gestureArray[samplesRead*6 + 4] = g.gyro.y;
    gestureArray[samplesRead*6 + 5] = g.gyro.z;
    
    samplesRead++;
    
    delay(10);

    // check the end of gesture acquisition
    if (samplesRead == NUM_SAMPLES) {

      // set model inputs
      neuton_model_set_inputs(gestureArray);
      
      // run model inference
      uint16_t predictedClass;
      float* probabilities;
      int returnCode = neuton_model_run_inference(&predictedClass, &probabilities);

      // check if model inference result is valid
      if (returnCode == 0) {

          // check if one of the result has >50% of accuracy
          if (probabilities[predictedClass] > 0.5) {
            Serial.print("Detected gesture: ");
            Serial.print(predictedClass);
            Serial.print(" [Accuracy: ");
            Serial.print(probabilities[predictedClass]);
            Serial.println("]");
          } else {
            // solution is not reliable
            Serial.println("Detected gesture: NONE");
          }
      } else {
        Serial.println("Inference fail to execute");
      }
    }
  }
}
