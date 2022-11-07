#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include "BluetoothSerial.h"
#include "esp_adc_cal.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "kiroshi.h"


#include <WiFi.h>
#include "time.h"
#include "sntp.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif




#define ADC_EN      14  //ADC_EN is the ADC detection enable port
#define ADC_PIN     34
#define MY_CS       33
#define MY_SCLK     25
#define MY_MISO     27
#define MY_MOSI     26
#define BUTTON_1            35
#define BUTTON_2            0
//#define SEALEVELPRESSURE_HPA(1013.25)

char buff[512];
int vref = 1100;
char c          =' ';
String data;
int timer      = 0; // timer value
int btnCick = false;

int lastState_1 = 0;
int currentState_1;

int lastState_2 = 0;
int currentState_2;

unsigned long delayTime;
int temp  = 0;
int hum   = 0;
String voltage ="";

String SN = "Egis sn:00001 ";
uint32_t chipId = 0;
String employer = "Dancsok Z.";
String ID = "ID:50000157"; 
String online = "Wifi Offlie";
String timestring ="Time: N/A";
int button_billent = 0;

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library


BluetoothSerial SerialBT;
Adafruit_BME280 bme; // I2C

//################# wifi and ntp server ############x
//const char* ssid       = "#BRFKterfigy011";
//const char* password   = "";

const char* ssid       = "Strogg iPhone";
const char* password   = "12345678";

const char* ntpServer1 = "0.hu.pool.ntp.org";
const char* ntpServer2 = "1.hu.pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3"; // europe/budapest
//###################################################

//############# cpu temp
#ifdef __cplusplus

extern "C" {

#endif

uint8_t temprature_sens_read();

#ifdef __cplusplus

}

#endif

uint8_t temprature_sens_read();
//##############################

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
     Serial.println("No time available (yet)");
       timestring = "Time_ N/A"; //view last time
       online = "Wifi Offline";
    return;
  }
  Serial.println(&timeinfo, "%F %H:%M:%S");
char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%F %H:%M:%S", &timeinfo);
  
  timestring = String(timeStringBuff);
 //  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  online = "Wifi Online";
  tft.setCursor(5,20);
  tft.print("SSID:"+String(ssid));
  tft.setCursor(5,30);
  tft.print(WiFi.localIP().toString());
}

void timeavailable(struct timeval *t)
{
  printLocalTime();

}

void ntp_time(){
sntp_set_time_sync_notification_cb( timeavailable );
  sntp_servermode_dhcp(1);    // (optional)
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
}

void showVoltage()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        voltage = "";
         if(battery_voltage >= 4.3){
           voltage = "USB:" + String(battery_voltage) + "V ";
         } else {
         float szazalek = (battery_voltage - 3.2)*100; 
         if(szazalek >= 51){tft.setTextColor(TFT_GREEN, TFT_BLACK);}
         if(szazalek <= 50){tft.setTextColor(TFT_YELLOW, TFT_BLACK);}
         if(szazalek <= 30){tft.setTextColor(TFT_RED, TFT_BLACK);}
         voltage = String(battery_voltage) + "V ";
         voltage += String(szazalek) + "% ";
         }
    }
}


void setup(){
  pinMode(13, OUTPUT);      // declare LED as output
  pinMode(BUTTON_2,INPUT);
  pinMode(BUTTON_1,INPUT);
  pinMode(A0, INPUT);
  pinMode(ADC_EN, OUTPUT);
  Serial.begin(9600);
  SerialBT.begin();
  Serial.println("Egis Pharmaceuticals PLC  1913-2022 Smart Glass");
setCpuFrequencyMhz(80);
  
 
    tft.init();        
     tft.setRotation(1);
  tft.writecommand(TFT_MADCTL); // mirroring command
    tft.writedata(0x39);  // mirroring paramter
    
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(0, 0);
    digitalWrite(ADC_EN, HIGH);


  // Set WiFi to station mode and disconnect from an AP if it was previously connected
// esp_err_t results = esp_wifi_stop();
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect();
//  delay(100);

  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  delayTime = 1000;

   esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV");
    }

//##############get chip id 
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId);
  SN=String(chipId);
welcome();
  delay(5000);
//############################
 
    showVoltage();
    tft_sight();

    
}

void loop()
{

//################ button functions #######################
currentState_1  = digitalRead(BUTTON_2);


if(digitalRead(BUTTON_2)==0){
 if(lastState_1==0){
 lastState_1=1;
    WiFi.begin(ssid, password);
     while (WiFi.status() != WL_CONNECTED) {
      //delay(500);
        tft.setCursor(50,50);
        tft.print("Wifi turn ON"); 
        Serial.println("Wifi turn on"); }
      
 } else {
      lastState_1=0;
      WiFi.mode(WIFI_OFF); WiFi.disconnect();
         tft.setCursor(50,50);
        tft.print("Wifi turn OFF"); 
        Serial.println("Wifi turn off");
 }
 delay(1000);
}


//###################### BT and SERIAL #############

 if (SerialBT.available()){  // Read data from BT
        c = SerialBT.read();
        Serial.write(c); // send BT data to serial
      if (c != -1) {
      data += c; 
      if (c != '\n') { 
        tft.setCursor(5,40);
         tft.setTextColor(TFT_BLUE, TFT_BLACK);
        tft.print("BT: "+data); 
        //data = ""; 
          }
       }
    }
 
   
    if (Serial.available()){ // Read serial data
        c =  Serial.read();
        SerialBT.write(c);  // send serial data to BT.
        Serial.write(c);   //Write raded serial data to serial console
        if (c != -1) {
      data += c; 
      if (c == '\n') { 
        tft.setCursor(5,80);
         tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.print("Serial: "+data); 
        //data = ""; 
          } 
       } 
    } 

 //############################################################### 
static uint64_t timeStamp_2 = 0;
    if (millis() - timeStamp_2 > 10000) {
    timeStamp_2 = millis();


         if(lastState_1 == 1){
            ntp_time();
              } else {
                timestring = "Time_ N/A"; //view last time
                 online = "Wifi Offline";
            
              }
    showVoltage();
    tft_sight();
    data="";
  // tft.print("button1:"+String(lastState_1));
    }
}

void tft_sight(void){

 tft.setTextSize(1);
 tft.setTextColor(TFT_WHITE, TFT_BLACK);
 tft.fillScreen(TFT_BLACK);
 tft.pushImage(80, 27,  111, 80, kiroshiu_tranparent);

 

 tft.setCursor(5,5);
 tft.print(voltage);
  
  //tft.setCursor(70,0);
   temp = bme.readTemperature()-10; //internal temp correction -15C
   if(temp >=30){tft.setTextColor(TFT_RED, TFT_BLACK); }  // Warning: Hight humidity;} // Warning: High temp.
   if(temp <= 0){tft.setTextColor(TFT_BLUE, TFT_BLACK);}  // Warning: Low temp. = ice
  tft.print(String(temp));
  tft.print("C ");
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
 hum = bme.readHumidity();
 if(hum >=70){tft.setTextColor(TFT_YELLOW, TFT_BLACK); }  // Warning: Hight humidity
 tft.print(String(hum));
 tft.print("% ");

tft.setTextColor(TFT_WHITE, TFT_BLACK);

 tft.print(bme.readPressure() / 100.0F);
 tft.print("hPa ");
 
 //tft.setCursor(200,0);
 tft.print(bme.readAltitude(1013.25));
  tft.print("m ");
 
 tft.setCursor(160,20);
tft.print(online);  
   
 tft.setTextColor(TFT_WHITE, TFT_BLACK);
 tft.drawLine(0, 0, 10, 0, TFT_WHITE); //  -
 tft.drawLine(0, 0, 0, 10, TFT_WHITE); // |
 
 
 //tft.setCursor(15,0);
 
 tft.drawLine(0, 125, 0, 135, TFT_WHITE); // |
 tft.drawLine(0, 134, 10, 134, TFT_WHITE); //  -  x,y-x,y
 
 tft.drawLine(230, 1, 240, 1, TFT_WHITE); //- 
 tft.drawLine(239, 0 , 239, 10, TFT_WHITE); // |
 

 tft.drawLine(239, 124 , 239, 134, TFT_WHITE); // |
 tft.drawLine(230, 134, 240, 134, TFT_WHITE); //-

tft.setCursor(5,115);
tft.print(timestring);
 // ############### business data #############
tft.setCursor(15,125);
 tft.setTextColor(TFT_YELLOW, TFT_BLACK);
tft.print("Egis "+ SN+" "+employer+" "+ID);
tft.setCursor(180,115);
tft.setTextColor(TFT_GREEN, TFT_BLACK);
tft.print("CPU:"+String((temprature_sens_read() - 32) / 1.8)+"C");
 //############################################

 //############ internal hall ######
int magnet = 0;
magnet = hallRead();
tft.setCursor(180,105);
tft.setTextColor(TFT_GREEN, TFT_BLACK);
tft.print("Hall:"+String(magnet));
//####################################
  
} // Sight END

/*void welcome(){ // "boot" screen
   tft.setTextColor(TFT_WHITE, TFT_BLACK);
   tft.fillScreen(TFT_BLACK);
   tft.setTextSize(2);
   tft.setCursor(90, 10);
   tft.println("Egis");
   tft.setCursor(5, 30);
   tft.println("Pharmaceuticals PLC");
   tft.setCursor(60, 50);
   tft.println("1913-2022");
   tft.setCursor(15, 70);
   tft.println("Smart Glass ver2.1"); 
   tft.setCursor(45, 90);
   tft.println("ChipId:"+SN); 
}*/

void welcome(){ // "boot" screen
   tft.setTextColor(TFT_WHITE, TFT_BLACK);
   tft.fillScreen(TFT_BLACK);
    tft.setSwapBytes(true);
    tft.pushImage(40, 1,  160, 42, kiroshiu_tranparent_boot);
   //tft.drawBitmap(1, 32, egis_ttgo, 175, 90, TFT_WHITE);
   tft.setTextSize(1);
   tft.setCursor(70, 95);
   tft.println("Kiroshi Opticals");
   tft.setCursor(90, 105);
   tft.println("1990-2013");
   tft.setCursor(65, 115);
   tft.println("Opti-Flash mk2.1"); 
   tft.setCursor(40, 125);
   tft.println("Hacked by Alt Cunningham"); 
}
