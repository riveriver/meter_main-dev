#ifndef Meter_H_
#define Meter_H_
#include <Arduino.h>
#include <Preferences.h>

enum BIE_COM_DEFINE{

  BATTERY_BASE = 0,//电量，小于1000的值就是电量

  METER_TYPE_500 = 2000,//靠尺类型
  METER_TYPE_600 = 2001,
  METER_TYPE_1000 = 2005,
  METER_TYPE_1200 = 2006,

  HOME_MODE_BASE = 3,
  HOME_MODE_ANGLE = 3000,//测量模式
  HOME_MODE_SLOPE = 3001,
  HOME_MODE_FLAT  = 3002,
  HOME_MODE_FLAT_SLOPE = 3003,

  SLOPE_STD_BASE = 4,
  SLOPE_STD_1000 = 4000,//坡度标准
  SLOPE_STD_1200 = 4001,
  SLOPE_STD_2000 = 4002,

  ANGLE_SEPPD_BASE  = 5,
  ANGLE_SEPPD_STD   = 5000,// 标准
  ANGLE_SEPPD_QUICK = 5001,// 快速
  ANGLE_SEPPD_AUTO  = 5002,// 自动:极快免按

  WARN_BASE = 6,
  WARN_DISABLE = 6000,//无预警
  WARN_ENABLE_NO_LIGHT = 6001,//有预警无灯
  WARN_ENABLE_LIGHT = 6002,// 有预警有灯

  CALIBRATION_BASE = 7,

  VERSION_SOFTWARE_BASE = 8,
  VERSION_HARDWARE_BASE = 9,
};

enum MEASURE_STATE{
  IDLE,
  UNSTABLE,
  MEASURING,
  MEASURE_DONE,
  UPLOAD_DONE,
};


class Meter{
private:
struct Calibration{
  uint8_t step = 0;
  uint8_t status = 0;
};
struct Measure{
  byte state = 0;
  byte progress = 0;
};
struct ClinoMeter{
  float angle_live = 0.0f;
  float angle_hold = 0.0f;
  byte  arrow_live = 0;
  byte  arrow_hold = 0;
  Measure measure;
};
struct FlatnessMeter{
  float flat = 0.0f;
  uint8_t arrow = 0;
};
// ClinoMeter    clino_hold;
FlatnessMeter flat_hold;
// ClinoMeter    clino;
FlatnessMeter flat_live;
Preferences   pref;
public:
  ClinoMeter  clino;
  int software_version = 701;
  int hardware_version = 201;
  int imu_version = 0;
  uint8_t meter_type = 1;
  uint8_t cursor = 0;
  uint8_t page = 0;
  uint8_t pre_page = 0;
  uint8_t minor_page = 0;
  uint8_t pre_minor_page = 0;
  uint8_t home_size = 2;
  uint8_t home_mode = 0;
  uint8_t pre_home_mode = 0;
  Calibration imu_cali;
  Calibration dist_cali;
  int     block_time;
  uint8_t speed_mode  = 1;
  uint8_t warrning_mode = 1;
  float   warrning_angle = 45.0;
  float   warrning_flat  = 3.0;
  float   slope_standard = 1000.0f;
  int     battery;
  bool    has_update_dist = false;
  bool    has_home_change = false;
  bool    has_imu_forward = false;
  bool    has_flat_forward = false;
  String  cali_forward_str = "";

  void set_angle_live(float value){
    byte arrow = clino.arrow_live;
    if (arrow == 0) {clino.angle_live = -value;} 
    else if(arrow == 1){clino.angle_live = value;}
    else{ESP_LOGE("","arrow:%d",arrow);}
  }
  void set_arrow_live(int value){clino.arrow_live = value;}
  void set_progress(int value){
    if(value < 0 || value > 100){
      return;}
    clino.measure.progress = value;
  }
  void hold_clinometer(float angle,int arrow){
    // clino.angle_hold = clino.angle_live;
    clino.angle_hold = angle;
    // clino.arrow_hold = clino.arrow_live;
    clino.arrow_hold = arrow;
  }
  void state_clinometer_measure(){
    if(clino.measure.state == IDLE 
    || clino.measure.state == MEASURE_DONE 
    || clino.measure.state == UPLOAD_DONE){
      clino.measure.state = UNSTABLE;
    }
  }
  void  set_live_flat(float value){flat_live.flat = value;}
  void  set_hold_flat(float value){flat_hold.flat = value;}
  float get_live_flat() {return flat_live.flat;}
  float get_hold_flat() {return flat_hold.flat;}

  void initMeter(){
    while(!pref.begin("Meter",false)){
        Serial.println(F("[initMeter]getMeter Fail"));
    }
    meter_type = pref.getInt("Type",11);
    home_mode  = pref.getInt("Home",0);
    pref.end();
  // init home_size
    if(meter_type > 10){
      home_size = 4;
    }
    else {home_size = 2;}
    // meter_type = 11;
    // home_mode  = 0;
    // home_size  = 2;
    home_mode = home_mode % home_size;
    pinMode(12,OUTPUT); 
    digitalWrite(12,HIGH);
  }

  void putMeterType(){
    while(!pref.begin("Meter",false)){}
    pref.putInt("Type", meter_type);
    pref.end();
  }

  void putMeterHome(){
    has_home_change = 1;
    while(!pref.begin("Meter",false)){
    }
    pref.putInt("Home",home_mode);
    pref.end();
  }

};

#endif 

