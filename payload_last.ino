//gerekn kütüphaneleri ekliyoruz   
#include "falconPayloadLib.h"
#define CURRENT_NODE PAYLOAD_NODE    


#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_AHTX0.h>
#include "math.h"
#include "bmp3_defs.h"
#include "TinyGPS++.h"
#include <HardwareSerial.h>

#include <DFRobot_BMP3XX.h>

/* If using Gravity products, choose these two interfaces and comment subsequent interfaces. */
DFRobot_BMP388_I2C sensor(&Wire, sensor.eSDOGND); 


//her sensör için obje tanımlıyoruz 
// iki tane mpu kullandığımız için adreslerini farklı yapıyoruz bağlantıda ise birinın AD0 pini GND diğerinin AD0 pini Vext bağlanır. 
// 4 sensörun SClsi kartın 22. pinine bağlanır SDAsı ise 21.pinine

Adafruit_AHTX0 aht;
Adafruit_Sensor *aht_humidity, *aht_temp;
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

TinyGPSPlus gps;

HardwareSerial SerialGPS(2);
//hesaplamalar için parameterler tanımlanır

float alpha = 0.5;
float filteredAccelX1 = 0.0;
float filteredAccelY1 = 0.0;
float filteredAccelZ1 = 0.0;
float filteredAccelX2 = 0.0;
float filteredAccelY2 = 0.0; 
float filteredAccelZ2 = 0.0;
float seaLevel;



//#define CALIBRATE_ABSOLUTE_DIFFERENCE

struct GpsDataState_t {
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double distMax = 0;
  double dist = 0;
  double altMax = -999999;
  double altMin = 999999;
  double spdMax = 0;
  double prevDist = 0;
};
GpsDataState_t gpsState = {};

#define TASK_SERIAL_RATE 1000 // ms
uint32_t nextSerialTaskTs = 0;

void setup() {
  //haberleşme ayarları yapılır her sensör için
  SerialGPS.begin(9600,SERIAL_8N1,16,17);
 
  Serial.begin(19200);

  int state = radio.begin(883.0, 250.0, 7, 5, 0xAA, 20, 6, 1);
  if (state == RADIOLIB_ERR_NONE) 
  {
    Serial.println(F("RadioLib success!"));
  } 
  else 
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    //while (true);
  }

  radio.setDio0Action(setFlag); //interrupt 
  Serial.println("setdio action ok") ; 
  
  if (!aht.begin()) {
    Serial.println("AHT10 bulunamadı!");
    
   
  }
  else {
    Serial.println("AHT10 bulundu!");

  }
  aht_temp = aht.getTemperatureSensor();
  aht_temp->printSensorDetails();

  aht_humidity = aht.getHumiditySensor();
  aht_humidity->printSensorDetails();


  
  
  int rslt;
  rslt = sensor.begin(); 
  if(ERR_DATA_BUS == rslt){
    Serial.println("Data bus error!!!");
  }else if(ERR_IC_VERSION == rslt){
    Serial.println("Chip versions do not match!!!");
  }
  delay(500);
  
  Serial.println("Begin ok!");


  if( !sensor.setSamplingMode(sensor.eUltraPrecision) ){
    Serial.println("Set samping mode fail, retrying....");
    delay(100);

  }

  delay(100);
  
  

  /* Get the sampling period of the current measurement mode, unit: us */
  float sampingPeriodus = sensor.getSamplingPeriodUS();
  Serial.print("samping period : ");
  Serial.print(sampingPeriodus);
  Serial.println(" us");

  /* Get the sampling frequency of the current measurement mode, unit: Hz */
  float sampingFrequencyHz = 1000000 / sampingPeriodus;
  Serial.print("samping frequency : ");
  Serial.print(sampingFrequencyHz);
  Serial.println(" Hz");

  Serial.println();


  mpu1.initialize();
  mpu2.initialize();
  Serial.print(",pular ok");
  
  delay(1000);

}

volatile bool transmittedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!

void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we sent a packet, set the flag
  transmittedFlag = true;
}


// Init all the variables.
dataPaket_t     paket;
Variables_t     variable    = { .telemTimer = 0 , .firstinit = true  , .u8_buffer = { 0 } , .u8_counter = 0};
dataStruct_t    data        ;
GcsPaket_t      gcsStructPaket;


void loop() {
  Serial.println("loop girdi  ok")  ;
 
  gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();
  //  long writeValue;

 
  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;
  
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read()); // read data from the GPS module and update the TinyGPS++ object
    
  }
  
  if (gps.location.isValid()) { // if valid GPS data is available
     data.payload_gps_enlem  =  gps.location.lat();
     data.payload_gps_boylam =  gps.location.lng();
     data.payload_gps_irtifa =  gps.altitude.meters();
    
    
  }
  else{

    Serial.print("locarion invalid");

    
  }
  
  // birinci mpu6050 sensöründen ivme verilerinin okunuması
  int16_t rawAccelX1 = mpu1.getAccelerationX();
  int16_t rawAccelY1 = mpu1.getAccelerationY();
  int16_t rawAccelZ1 = mpu1.getAccelerationZ();

  //ivme değerlerini g unite çevirmek
  float accelX1 = (float)rawAccelX1 / 16384.0; 
  float accelY1 = (float)rawAccelY1 / 16384.0;
  float accelZ1 = (float)rawAccelZ1 / 16384.0;
  filteredAccelX1 = (alpha * accelX1) + ((1 - alpha) * filteredAccelX1);
  filteredAccelY1 = (alpha * accelY1) + ((1 - alpha) * filteredAccelY1);
  filteredAccelZ1 = (alpha * accelZ1) + ((1 - alpha) * filteredAccelZ1);

  // ikinci mpu6050 sensöründen ivme verilerinin okunuması
  int16_t rawAccelX2 = mpu2.getAccelerationX();
  int16_t rawAccelY2 = mpu2.getAccelerationY();
  int16_t rawAccelZ2 = mpu2.getAccelerationZ();
  
  //ivme değerlerini g unite çevirmek
  float accelX2 = (float)rawAccelX2 / 16384.0;
  float accelY2 = (float)rawAccelY2 / 16384.0;
  float accelZ2 = (float)rawAccelZ2 / 16384.0;
  filteredAccelX2 = (alpha * accelX2) + ((1 - alpha) * filteredAccelX2);
  filteredAccelY2 = (alpha * accelY2) + ((1 - alpha) * filteredAccelY2);
  filteredAccelZ2 = (alpha * accelZ2) + ((1 - alpha) * filteredAccelZ2);

  //iki sensörden (mpu6050)Z eksendeki ivme değerlerinin serial monitorda gösterilmesi  
  Serial.print("AccelZ1=");
  Serial.print(filteredAccelZ1);
  Serial.print(",");
  Serial.print("AccelZ2=");
  Serial.print(filteredAccelZ2);
  Serial.println();
    

  //AHT10 sensöründen nem ve sıcaklık verilerinin okunması
  sensors_event_t humidity;
  sensors_event_t temp;
  aht_humidity->getEvent(&humidity);
  aht_temp->getEvent(&temp);
    
  ////AHT10 sensöründen nem ve sıcaklık verilerinin serial monitorda gösterilmesi
  Serial.print("Temp:");
  Serial.print(temp.temperature);
  Serial.print(",");
  Serial.print("Humidity:");
  Serial.print(humidity.relative_humidity);
  Serial.println();



  /* Directly read the currently measured pressure data, unit: pa */
  float Pressure = sensor.readPressPa();
  Serial.print("Pressure :******************************* ");
  Serial.println(Pressure);
  Serial.println(" Pa");

  data.accelz1 = filteredAccelZ1;  
  data.accelz2 = filteredAccelZ2;
  data.payload_nem=humidity.relative_humidity;
  data.payload_sicaklik=temp.temperature;
  data.payload_basinc = Pressure;


  if ( variable.firstinit )
    {
        initDataPaket( &paket,  CURRENT_NODE );
        variable.firstinit  = false;
        memset( &data , 0  , sizeof( dataStruct_t ) );
        variable.telemTimer = millis();
        
    }

  if ( millis() - variable.telemTimer  >= 200 )
  {
      veriPaketle( &paket,  &data);
      verileriYolla( paket.u8_array , sizeof( dataPaket_t ) );
      variable.telemTimer = millis();
  }

  
}