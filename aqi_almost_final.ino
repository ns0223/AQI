#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <math.h>
#include <MQUnifiedsensor.h>
#include "MQ135.h"
#include "ThingSpeak.h"
#ifdef U8X8_HAVE_HW_SPI
#endif
#ifdef U8X8_HAVE_HW_I2C
#endif

//include U8g2 library
#include <U8g2lib.h>

//OLED INTERFACING
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


//LED INTERFACING
#define LED1 32  //GREEN LED
#define LED2 25  //YELLOW LED
#define LED3 33  //RED LED



float CO;
float CO2;
float air_quality;



#define channel_id 1693658
#define channel_api_key "TP466I6NSP4P670E"


 

#define WIFI_TIMEOUT_MS 20000
#define WIFI_NETWORK "LAPTOP-72DIUI42 5707"
#define WIFI_PASS "12344321"

void connectToWiFi(){
  Serial.print("Connecting to Wifi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK,WIFI_PASS);

  unsigned long startAttemptTime = millis();

  while(WiFi.status()!=WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
    Serial.print(".");
    delay(100);
  }

  if(WiFi.status() != WL_CONNECTED){
      Serial.println("Failed");
  }
  else{
    Serial.print("Connected");
    Serial.println(WiFi.localIP());
  }
}


#include "esp_adc_cal.h"
 
#define LM35_Sensor1    36
 
int LM35_Raw_Sensor1 = 0;
float LM35_TempC_Sensor1 = 0.0;
float LM35_TempF_Sensor1 = 0.0;
float Voltage = 0.0;


//Declaration of MQ135
#define MQ135_THRESHOLD_1 400 // Fresh Air threshold

WiFiClient client;

void setup()
{
  Serial.begin(9600);
  u8g2.begin();
  connectToWiFi();
  ThingSpeak.begin(client);

  // Set pin mode
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
}


void loop()
{ 


  //Read MQ135 Sensor
  MQ135 gasSensor = MQ135(A0);
  float air_quality = gasSensor.getPPM();
  Serial.print("Air Quality: ");  
  Serial.print(air_quality);
  Serial.print("  PPM");   
  Serial.println();
  Serial.println();
  

 
  // Read LM35_Sensor1 ADC Pin
  LM35_Raw_Sensor1 = analogRead(LM35_Sensor1);  
  // Calibrate ADC & Get Voltage (in mV)
  Voltage = readADC_Cal(LM35_Raw_Sensor1);
  // TempC = Voltage(mV) / 10
  LM35_TempC_Sensor1 = Voltage / 10;
  LM35_TempF_Sensor1 = (LM35_TempC_Sensor1 * 1.8) + 32;
 
  // Print The Readings
  Serial.print("Temperature = ");
  Serial.print(LM35_TempC_Sensor1);
  Serial.print(" °C , ");
  Serial.print("Temperature = ");
  Serial.print(LM35_TempF_Sensor1);
  Serial.println(" °F");
  
  
  // LED CONFIGURE
  if(air_quality <200)
  { Serial.println(" GREEN");
    
    digitalWrite(LED1,HIGH);
    delay(500);

  }
  else if(air_quality>200 && air_quality<400)
  { Serial.println( " YELLOW ");
    
    digitalWrite(LED2,HIGH);
    delay(500);

  }
  else
  { Serial.println(" RED ");
  
    digitalWrite(LED3,HIGH);
    delay(500);

  }
  Serial.println();
  Serial.println();
  
  otherpara();




  //u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(5,10,"AQI : ");  // write something to the internal memory
  u8g2.setCursor(52,20);
  u8g2.print(air_quality,2); 
  u8g2.drawStr(5,40,"TEMPERATURE : ");
  u8g2.setCursor(43,55);
  u8g2.print( LM35_TempC_Sensor1,2); 
  u8g2.drawStr(68,52," o");
  u8g2.drawStr(78,55,"C");
  u8g2.sendBuffer();          // transfer internal memory to the display
  delay(5000); 
  u8g2.clear();

  digitalWrite(LED1,LOW);
  delay(500);  
  digitalWrite(LED2,LOW);
  delay(500);
  digitalWrite(LED3,LOW);
  delay(500);
  
  //sendind data to thingspeak

  ThingSpeak.setField(1, air_quality);
  ThingSpeak.setField(2, LM35_TempC_Sensor1);

  ThingSpeak.writeFields(channel_id, channel_api_key);
  delay(3000);


 
}



void otherpara() {

  //Definitions
  #define placa "ESP-32"
  #define Voltage_Resolution 5
  #define pin A0 //Analog input 0 of your arduino
  #define type "MQ-135" //MQ135
  #define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO
  #define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
  //#define calibration_button 13 //Pin to calibrate your sensor

  //Declare Sensor
  MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init(); 
 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  /*****************************  MQ CAlibration ********************************************/ 
  Serial.println("** Values from MQ-135 ****");
  Serial.println("|    CO   |  Alcohol |   CO2  |  Toluen  |  NH4  |  Aceton  |");
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
  float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
  float Alcohol = MQ135.readSensor(); // SSensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
  float Toluen = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
  float NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
  float Aceton = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  Serial.print("|   "); Serial.print(CO); 
  Serial.print("   |   "); Serial.print(Alcohol);
  // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
 
  Serial.print("   |   "); Serial.print(CO2 + 400); 
  Serial.print("   |   "); Serial.print(Toluen); 
  Serial.print("   |   "); Serial.print(NH4); 
  Serial.print("   |   "); Serial.print(Aceton);
  Serial.println("   |"); 
  Serial.println("");
  Serial.println("");

  ThingSpeak.setField(3, CO2+400);  
  ThingSpeak.setField(4, CO);
  ThingSpeak.writeFields(channel_id, channel_api_key);
  
}


uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}
