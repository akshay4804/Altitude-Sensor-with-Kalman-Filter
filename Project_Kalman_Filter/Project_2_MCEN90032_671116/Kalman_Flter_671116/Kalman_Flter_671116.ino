//Code written by Akshay Kumar, 671116, for MCEN90032 Sensor Systems Project 2
//Last editted 23rd September 2018

#include <Matrix.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

double sampling_frequency = 20 ;       // Sampling frequency selected
double pressure_today = 1027.3;       // The sea level pressure for today, as per 23/9/2018 from BOM. 
double T= 1/sampling_frequency;  

// Accelerometer variables
double zmax= 402;
double zmin= 267;
double G = 1;
int analogPinZ = A3;
float valz;
double normalised_acc = 0;
double sensorValuez = 0; 
       
// Pressure sensor variables
Adafruit_BMP280 bmp;
double save;
double height_now;
double relative_height;

// Variances of the accelerometer and pressure sensor respectively 

double var_acc= 0.0000544678; 
double var_pressure= 0.000328;    

// Setting up timer variables
double Current_Time;
double Prev_Time = 0;
double Time_Interval = T;

// Initialising all the matrices needed for the Kalman Filter
double array1[2][2] = {{1, T},{0, 1}}; // The A matrix
double array2[2][1] = {{(T*T)/2.0},{T}}; // The B Matrix
double array3[1][2] = {1, 0};   // The C matrix
double array4[2][1] = {{0},{0}};   // The initial estimate for the position and velocity of the arduino system
double array5[2][2] = {{1,1},{1,1}}; // Initial Error Covariance (indicates there is noise in the system)
double array6[2][2] = {{1,0},{0,1}};

Matrix<double> A(2, 2, (double*)array1);
Matrix<double> B(2, 1, (double*)array2);
Matrix<double> C(1, 2, (double*)array3);
Matrix<double> x_k(2, 1, (double*)array4);
Matrix<double> P(2, 2, (double*)array5);
Matrix<double> x_bar(2, 1);  // State Prediction
Matrix<double> P_bar(2, 2); // Error Covariance Matrix
Matrix<double> K(2, 2);   // Kalman Gain Matrix
Matrix<double> I(2, 2, (double*)array6);  // Identity Matrix
Matrix<double> Q = 0.005*B*Matrix<double>::transpose(B)*var_acc;  // Estimated Error of the Accelerometer
double R = var_pressure*var_pressure;                // Estimated Error of the Barometer

// Other variables used
int i;
int counter = 0;
  
void setup(){
  Serial.begin(9600);
  if(!bmp.begin()){
    Serial.println("Could not find valid BMP280 module");
    while(1);
  }
}

void loop(){
  Current_Time = millis();
  if(Current_Time - Prev_Time >= Time_Interval){
    Prev_Time = Current_Time;
    
    // Accelerometer collects data
    valz = analogRead(analogPinZ);
    // Calibration of Accelerometer from Project 1
    sensorValuez = (map2_float(valz, zmin, zmax, -G, G));
    // Normalisation of acceleration data
    normalised_acc = sensorValuez - G;
     
    // Pressure sensor readings
    bmp.readTemperature();
    bmp.readPressure()/100;
    height_now = bmp.readAltitude(pressure_today);
    // Establishing a baseline for the pressure sensor to measure from
    if (counter == 0){
      save = height_now;
      counter = 1;
    }
    // Determining the relative height from an initial baseline
    relative_height = height_now - save;
    // The following section is the Kalman Filter being implemented

    // Time Update (prediction)
    x_bar = A*x_k + B*normalised_acc;              // Project the state ahead of time
    P_bar = A*P*Matrix<double>::transpose(A) + Q;  // Project the error covariance ahead of time

  // Measurement Update (correction)
    K = P_bar*Matrix<double>::transpose(C)*Matrix<double>::inv((C*P_bar*Matrix<double>::transpose(C) + R)); // Kalman Gain calculation
    x_k = x_bar + K*(relative_height - C*x_bar);  // Update the estimation of the position and velocity of the arduino system
    P = (I - K*C)*P_bar;        // Updating the error covariance

    // Plotting all the data
    Serial.print(normalised_acc);  // Acceleration
    Serial.print("\t");
    Serial.print(relative_height);  //Pressure sensor height
    Serial.print("\t");
    Serial.println(x_k._entity[0][0]); // The Kalman filter position
  }
}
// Calibration function as per Project 1
float map2_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




