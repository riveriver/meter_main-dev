/**
 * @file Flatness.h
 * @author Vicky Hung
 * @brief Scan, read, filt and calibrate the distance information from IR sensor and ads1115.
 * @version 0.1
 * @date 2023-07-31
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef Flatness_H
#define Flatness_H
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include "MaxMin.h"
#include "SLED.h"
#include "Measure.h"
#include "MeterManage.h"
#include <Preferences.h>

extern Meter manage;

class Flatness
{
private: 
    const bool debug_mode = 1;
    const bool Serial_Print_Param       = true;  /** @brief True if require printing the I2C scanning result in Setup.*/
    const bool Serial_Print_Raw_Data    = false;   /** @brief True if require printing the Raw Data while Update.*/
    const bool Serial_Print_Distance    = false;   /** @brief True if require printing the Distance while Update.*/
    const int   Cut_Off_Voltage = 2000;/** @brief Cut-off voltage*/
    const float Cut_Off_Distance = 99.9f;/** @brief Cut-off Distance*/
    const byte IIC_1_SCL = 9;   
    const byte IIC_1_SDA = 8;
    const byte IIC_2_SCL = 4;
    const byte IIC_2_SDA = 5;
    const int  ADS_1_ADDR = 0x49;
    const int  ADS_2_ADDR = 0x49;
    
    // stores
    enum StoresError {
      ERROR_SUCCESS = 0,
      ERROR_NULL_POINTER = 1,
      ERROR_BEGIN_FAIL = 2,
    };
    Preferences stores;
    unsigned long action_timeout = 100;

    // calibration
    #define cali_size 11 
    #define sensor_size 7 
    float slopes[8] = {0};
    float zeros[8] = {0}; /** @brief Voltage reading when distance = 0.*/
    const int sample_size  = 20; /** @brief Number of data require for calibrating zeros.*/
    const int sample_threshold = 110; /** @brief Stabilization identification threshold when doing calibration.*/
    int sample_count = 0;
    uint16_t sample_start[8] = {0};
    unsigned int sample_sum[8] = {0};
    const int  Iteration_size = 120;
    float cali_target[cali_size] = {0.01, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    float cali_sample[sensor_size][cali_size] = {0};
    
    Adafruit_ADS1115 ads1115[2];       /** @brief Sensor driver for the Adafruit ADS1115 ADC breakout.*/
    uint16_t ADCread[2][8] = {0}; /** @brief ADS1115 ADC raw value buffer.*/
    bool isUpdate[2] = {0};        /** @brief isUpdate[i] = True if sensor connect to Wire(i) is update.*/
    // Filter Setting
    const int   surges_TH = 50;              /** @brief Surges filter threshold @deprecated &Delta;Distance &asymp; surges_TH * 0.005 mm (@ ADCread &asymp; 8000).*/
    const int   BW_Reset_TH = 500;           /** @brief Reset the Butterworth filter if the raw data exceed the BW_Reset_TH.*/
    const float BW_C[3] = {0.8, 0.1, 0.1}; /** @brief Buttorworth filter coefficient @note c0 + c1 + c2 = 1.0 @note c1 = c2*/
    /*****************************************************************************************************
     * Calibration buffer.
     ******************************************************************************************************/
    void  updateSingleChannel(int id);
    void  collectCaliSample();
    void  caliZero();
    void  doCalibration();
    void  generateResult();
    
    float calcIterativeCoefficientsB0(int ID,float x[], float y[], uint8_t data_num);
    byte getDistanceZero(float *data, int data_size);
    byte putDistanceZero(float *data, int data_size);
    byte getDistanceScale(float *data, int data_size);
    byte putDistanceScale(float *data, int data_size);
    void printRaw();

public:
    Flatness(){
      for (int i = 0; i < 8; i++) {
          slopes[i] = 330000;
          zeros[i] = 10000;
      }
    }
    byte ads_connect[2] = {false};

    /*****************************************************************************************************
     * Basic.
     ******************************************************************************************************/
    
     /** @brief Default distance when ads reading under the cut-off voltage.*/
    uint16_t ADCSurgeOut[20][8] = {0}; /** @brief ADS1115 ADC reading with surge filter.*/
    float ADCfilt[8]   = {0};            /** @brief Filted ADS1115 ADC reading.*/
    float Distance[8]  = {0};           /** @brief Distance value calculate from ADCfilt.*/
    float nDistance[8] = {0};          /** @brief Distance value calculate from ADCfilt.*/
    MaxMin Mm;

    /*****************************************************************************************************
     * CalibrationZero
     ******************************************************************************************************/
    bool Enable_Auto_Reset = false;            /** @brief True if enable auto zero position calibration.*/
    bool Enable_Cali_Slope = false;
    bool Read_Slope_First  = false;
    bool has_set_zero     = Enable_Auto_Reset;         /** @brief True when doing zero position calibration.*/
    float SetZeroProgress = Enable_Auto_Reset; /** @brief Zero position calibration's data collection progerss.*/
    /*****************************************************************************************************
     * Calibration
     ******************************************************************************************************/
    void init();
    void update(byte wire);
    void reset(bool onoff, bool force);
    float get_Mm_Diff(){return Mm.Diff;};
};
#endif