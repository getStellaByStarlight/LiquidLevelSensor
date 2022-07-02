#include <QuickMedianLib.h>
#include <Checksum.h>
#include <EEPROM-Storage.h>
#include <InterpolationLib.h>

const unsigned int START_CALIBRATION_CODE = 1;
const unsigned int FINISH_CALIBRATION_CODE = 2;

const unsigned int MAX_MESSAGE_LENGTH = 2; // in bytes
const char MESSAGE_ENDING_CHAR = '\n';

// limit the number of calibration points to 50
// we do this because EEPROM is used for storage
// Arduino Nano only has 1KB of EEPROM Storage
const int MAX_CAL_POINTS = 30;

// calibration data for waterlevel interpolation
int cal_points;
double xValues[MAX_CAL_POINTS];
double yValues[MAX_CAL_POINTS];

// default calibration data 
// Based on measurements on Arduino Nano 5V->A0 with 250mm food-grade eTape in a HydroFlask 64oz bottle
const int DEFAULT_CAL_POINTS = 6;
const double DEFAULT_XVALUES[DEFAULT_CAL_POINTS] = { 515, 535, 593, 675, 775, 795 };
const double DEFAULT_YVALUES[DEFAULT_CAL_POINTS] = {   0,  50,  100,  150,  200,  250 };

const uint8_t EEPROM_CAL_POINTS_ADDR = 0;
const uint8_t EEPROM_FIRST_POINT_ADDR = 2+1; // int is 2 bytes + 1 byte checksum
const uint8_t EEPROM_POINT_ADDR_STEP = 4+1; // double is 4 bytes + 1 byte checksum


void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  loadCalibrationData();
}

void loadCalibrationData() {
  Serial.println("> START LOADING CALIBRATION DATA...");
  
  /* first EEPROM byte holds the number of calibration points */
  EEPROMStorage<int> ee_cal_points(EEPROM_CAL_POINTS_ADDR, DEFAULT_CAL_POINTS); // 2 bytes used
  cal_points = ee_cal_points;
  Serial.print("> Calibration Points: \t");
  Serial.println(cal_points);

  /* next bytes hold calibration data */
  int ee_step = EEPROM_POINT_ADDR_STEP;
  int ee_adr = EEPROM_FIRST_POINT_ADDR;

  for (int i = 0; i < cal_points; i++) {
    double px = DEFAULT_XVALUES[i];
    double py = DEFAULT_YVALUES[i];
    
    EEPROMStorage<double> ee_cal_point_x(ee_adr, px);
    ee_adr += ee_step;
    EEPROMStorage<double> ee_cal_point_y(ee_adr, py);
    ee_adr += ee_step;
    
    xValues[i] = ee_cal_point_x;
    yValues[i] = ee_cal_point_y;

    Serial.print("> Loading Point ");
    Serial.print(i);
    Serial.print(": \t");
    Serial.print("x=");
    Serial.print(xValues[i]);
    Serial.print("   ");
    Serial.print("y=");
    Serial.println(yValues[i]);
  }

  Serial.println("> CALIBRATION DATA LOADED!");
}

int getSerialIntMessage() {
  while(Serial.available() > 0) {
    int msg = Serial.parseInt();
    Serial.println(msg);
    return msg;
  }
}

void sendWaterLevel() {
  int raw = analogRead(0);
  int levelSmooth = Interpolation::SmoothStep(xValues, yValues, cal_points, raw);
  
  Serial.print("raw=");
  Serial.println(raw);
  Serial.print("mm=");
  Serial.println(levelSmooth);
}

void startCalibration() {
  double cal_x[MAX_CAL_POINTS];
  double cal_y[MAX_CAL_POINTS];
  int num_point = 0;
  
  // initialize calibration
  Serial.println("> RECEIVED CALIBRATION SIGNAL");

  int sig = 0;
  while (sig != FINISH_CALIBRATION_CODE && num_point < MAX_CAL_POINTS) {
    sig = getSerialIntMessage();
    
    // we received a point
    if (sig > 2 || num_point == 0) {
      Serial.println("> CALULATING POINT...");

      // do 10s of continous readings to find a good mapping value
      int num_readings = 100;
      int readings[num_readings];
      for (int i = 0; i < num_readings; i++) {
        int raw = analogRead(0);
        readings[i] = raw;
        delay(100);
      }
      int med_read = QuickMedian<int>::GetMedian(readings, num_readings);
      
      // store cal data
      cal_x[num_point] = (double)med_read;
      cal_y[num_point] = (double)sig;

      // send serial response for current point
      Serial.print("> POINT CALCULATED! y=");
      Serial.print(sig);
      Serial.print(" x=");
      Serial.print(med_read);
      Serial.print(" no=");
      Serial.println(num_point);
      
      num_point++;
    }

    delay(500);
  }

  // finished calibration process
  Serial.println("> FINISHING CALIBRATION...");
  
  // store values to EEPROM
  EEPROMStorage<int> ee_cal_points(EEPROM_CAL_POINTS_ADDR, DEFAULT_CAL_POINTS);
  ee_cal_points.set(num_point);

  int ee_step = EEPROM_POINT_ADDR_STEP;
  int ee_adr = EEPROM_FIRST_POINT_ADDR;

  for (int i = 0; i < num_point; i++) {
    double px = cal_x[i];
    double py = cal_y[i];
    
    EEPROMStorage<double> ee_cal_point_x(ee_adr, 0);
    ee_cal_point_x.set(px);
    ee_adr += ee_step;
    
    EEPROMStorage<double> ee_cal_point_y(ee_adr, 0);
    ee_cal_point_y.set(py);
    ee_adr += ee_step;
  }

  delay(5000);
  Serial.println("> CALIBRATION SUCCESSFUL!");

  // refresh calibration data
  loadCalibrationData();
}

void loop() {
  int signal_number = getSerialIntMessage();
  
  if (signal_number == START_CALIBRATION_CODE) {
    startCalibration();
    delay(2000);
  } else {
    sendWaterLevel();
    delay(500);
  }
}
