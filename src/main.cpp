/*
    Name:       BeeScaleNew.ino
    Created:	7/3/2019 5:50:14 PM
    Author:     DANIJELA-PC\Danijela
	Edited: 22/01/2024 
*/

//#include <RunningMedian.h>
#include <Adafruit_SHT31.h>
#include <avr/sleep.h>
#include <DS3231.h>
#include <SoftwareSerial.h>
#include <Wire.h>
//#include "SparkFunBME280.h"
#include <HX711.h>
#include <HX711-multi.h>

void(* resetFunc) (void) = 0;		//reset function for arduino

//seting up clock variables
//------------------------
int year;
int month;
int day;
int hour;
int min;
int ArduinoWakeInterval=1; //Options are: 1-every minute, 30-every half hour , 60-every hour
int uploadInterval=5;
String reply="";
int errorCount=0;
int battPercent;
int battPercentGsm;
//tmp min value to store in what minute was last upload happend
//initial state is 60 becouse that walue is imposible to get from rtc,
//so it will always be possible to upload at first launch
int tmpMin=60;

//flag for wakeup
int flag=0;

//Stting up initial state of the wakeup switch
//-------------------------------
int wakeupSwitch = 0;
float tare=0;
//-------------------------------

// Setting up interrupt pins
//-------------------------------
#define wakePin 2					// pin used for waking up the Arduino (interrupt 0)
#define wakePin2 3					// pin used for waking up the Arduino (interrupt 1)
//-------------------------------

//setting up pin for waking up GSM module  
//-------------------------------
const byte gsmWakePin = 4;			// pin  used for waking up the GSM module from sleep mode
//-------------------------------

//Setting up variables for measurement of the battery  
//-------------------------------
int batteryMax = 100;
//-------------------------------

//setting up variables for uploading to IOT sites
//-------------------------------
const String thingSpeakUpadate = "GET http://api.thingspeak.com/update?api_key=03SUMGLJ4MO9KAZK&";
//-------------------------------

//ds3231 working variables
//-------------------------------
char print_date[16];						//char array for date and time 
RTCDateTime dt;								//date and time class for ds3231
DS3231 clock;								//define class for DS3231 clock
byte ADay, AHour, AMinute, ASecond, ABits;	//define clock variables
bool ADy, A12h, Apm;						//define clock variables
//-------------------------------

//Initialisation of HX711 ( -Load cell amplifier- )
//----------------------------------------------
#define CLK  A1							//arduino pin for CLK signal
#define DOUT A2							//arduino pin for DOUT signal
HX711 scale(DOUT, CLK);					//initialisation of HX711
float scaleCalibrationFactor = -20350;	//Calibration factor for the scale 
//-------------------------------

//Variables for BME280 sensor
//-------------------------------
//BME280 bme;									// Define Pressure sensor class
//-------------------------------

// initialisation of SHT31 sensor for Temperature and Humid
//----------------------------------------------
Adafruit_SHT31 sht31 = Adafruit_SHT31();
//----------------------------------------------

// initialisation of software serial feature for communicating with gsm module
//----------------------------------------------
SoftwareSerial gsmSerial(8, 7);				// Define pins for communicating with gsm module
//----------------------------------------------

//Setting up median filter library
//----------------------------------------------
//RunningMedian measurements = RunningMedian(10);
//----------------------------------------------


float ReadWeight(int loops)
{
	// for resetting purposes becouse scale have error measurement after big weight change
	//for (int i = 0; i < loops; i++)
	//{
	//	weight = weight + scale.get_units(), 3;
	//	//Serial.println(weight);
	//	delay(100);
	//}
	float raw;
	float weightInGrams;
	for (int i = 1; i <= loops; i++)
	{
		raw = scale.get_units(loops);
		delay(100);
		if (weightInGrams<raw*1000)
			weightInGrams=raw*1000;
	}

	return 	weightInGrams;
}

void ResetScale(int mode )
{
	if (mode == 1)
	{
		dt = clock.getDateTime();
		if (dt.hour == 00 && dt.minute==00)
		{
			Serial.println(F("midnight"));
			scale.tare();
		}
	}
	else if (mode == 2)
	{
		scale.tare();
	}
}
int ResetGSM()
{
	errorCount=errorCount+1;
	Serial.println("GSM ERROR No.: " + String(errorCount)+ " - Resetting");
	gsmSerial.println(F("AT+CFUN=0"));
	delay(4000);
	gsmSerial.println(F("AT+CFUN=1"));
	delay(12000);
	Serial.println(F("Finished resseting"));
	return errorCount;
}
void PrintTimeAndDate()
{
	dt = clock.getDateTime();
	sprintf(print_date, "%02d/%02d/%d %02d:%02d:%02d", dt.day, dt.month, dt.year, dt.hour, dt.minute, dt.second);
	Serial.println(print_date);
}
void ReadTime()
{
	dt = clock.getDateTime();
	year= dt.year;
	month=dt.month;
	day=dt.day;

	hour=dt.hour;
	min=dt.minute;
}
float ReadBattery(int loops)
{
	analogRead(A0);  // used only for A0 pin to settle. Measurement is ignored
	float voltage=0;
	int raw;
	for (int i = 1; i <= loops; i++)
	raw = analogRead(A0);
	{
		if (voltage<raw)
			voltage=raw;
	}
	voltage = (voltage / 1023) * 1100; //number in get.Average function is for calculating average of three middle measurements
//Serial.println(voltage);

	//map min and max voltage values on analog pin to scale 0% to 100%
	/*
	For clarification:
	Analog reading on defined analog pin A0 is measured based on internal voltage of 1.1V
	If voltage on the pin is 1.1V, arduino will measure 1023 value.
	If voltage on pin is 0 arduino will measure 0 value.
	BUT!!!
	LiIon battery can have voltage values between about 4.3V(full charge) and about 3.7V (empty)
	SO!!!
	We have to scale input voltage on that pin to be at maximum 1.1V when battery is full.
	It can be done with voltage divider (see schematics)
	With battery I use maximum value on the pin is 1035mV (on voltage divider), and I consider battery empty when value falls to 882)
	*/
	voltage = map(voltage, 882, 1030, 0, 100);
	//Serial.println(voltage);
	/*if (voltage > batteryMax)
	{
		voltage = batteryMax;
	}
	  else
	{
		*/
		batteryMax = voltage;
	//}

	return voltage;
}

float ReadSoil(int loops)
{
	analogReference(DEFAULT);
	analogRead(A3);	//  used  only for A3 pin to settle. Measurement is ignored
	float soilMoisture;
	float percentRaw;
	float soilRaw;
	for (int i = 1; i <= loops; i++)
	{
		soilRaw = analogRead(A3);
		percentRaw = ((1023 - soilRaw) / 1023) * 100;
		if (soilMoisture<percentRaw)
			soilMoisture=percentRaw;
	}
	analogReference(INTERNAL);
	analogRead(A3);
	return soilMoisture;
}
float ReadSht31Temp()
{
	float t = sht31.readTemperature();

  if (! isnan(t)) 
  {  // check if 'is not a number'
	return t;
  } else { 
    //Serial.println(F("Failed to read temperature"));
	return 0;
  }
}
float ReadSht31Humid()
{
	float h = sht31.readHumidity();
	if (! isnan(h)) 
	{  // check if 'is not a number'
	return h;
	} else { 
	//Serial.println(F("Failed to read humid"));
	return 0;
	}
}
// float ReadBmeTemperature()
// {
// 	float bmeTemp = bme.readTempC();
// 	return bmeTemp;
// }
// float ReadBmeHumid()
// {
// 	float bmeHumid = bme.readFloatHumidity();
// 	return bmeHumid;
// }
// float ReadBmePressure()
// {
// 	float bmePressure = bme.readFloatPressure()/100; // 100 Pa = 1 millibar;
// 	return bmePressure;
// }
// int ReadBattPercent()
// {
//   String batt;
//   int battPercent;
// gsmSerial.println(F("AT+CBC"));		//read battery status
// delay(2000);
// while(gsmSerial.available())
// 	{
// 		char c = gsmSerial.read();  //gets one byte from serial buffer
// 		reply += c;
// 		delay(10);
// 	}
// Serial.println("\n");
// Serial.println("Reply: ");
// Serial.print(reply);
// if(reply.indexOf(',')>0)
// {
// 	int i=reply.indexOf(',');
// 	if(reply.indexOf(',', i+1)>0)
// 	{
// 		int n=reply.indexOf(',', i+1);
// 		battPercent=100-(100-reply.substring(i+1, n).toInt())/0.6; //calculation to mark scale from 100 to 40  as scale from 100 to 0 percent because device stops working at about 40 percent
// 		if (battPercent<0)
// 			battPercent=0;
// 		Serial.println("Batt: "+ String(battPercent));
// 	}
// }
//   return battPercent;
// }

// int ReadBattPercent(int i)
// {
//   String batt;
//   int battPercent;
//   int battPercentRaw;
//   for (int n=0;n<i;n=n+1)
//   	{
// 		gsmSerial.println(F("AT+CBC"));		//read battery status
// 		delay(2000);
// 		while(gsmSerial.available())
// 			{
// 				char c = gsmSerial.read();  //gets one byte from serial buffer
// 				reply += c;
// 				delay(10);
// 			}
// 		Serial.println("\n");
// 		Serial.println("Reply: "+reply);
// 		Serial.println("\n");	
// 		if(reply.indexOf(',')>0)
// 		{
// 			int i=reply.indexOf(',');
// 			if(reply.indexOf(',', i+1)>0)
// 			{
// 				int n=reply.indexOf(',', i+1);
// 				battPercentRaw=100-(100-reply.substring(i+1, n).toInt())/0.6; //calculation to mark scale from 100 to 40  as scale from 100 to 0 percent because device stops working at about 40 percent
// 				if (battPercentRaw<0)
// 				{
// 					battPercentRaw=0;
// 				}
// 				Serial.println("Batt: "+ String(battPercentRaw));
// 			}
// 		}
// 		Serial.println(battPercent + "  " + battPercentRaw);
// 		if (battPercent<battPercentRaw)
// 			battPercent=battPercentRaw;				//take higest measured value
// 	battPercentRaw=0;
// 	}
//   return battPercent;
// }
void DisplayMeasurementsOnSerialMonitor()
{
	PrintTimeAndDate();
	Serial.println("Battery status: " + String(ReadBattery(1))+" %");
	//Serial.println("BME sensor temperature: " + String(ReadBmeTemperature())+" C");
	//Serial.println("BME sensor humid: " + String(ReadBmeHumid())+" %");
	//Serial.println("BME sensor pressure: " + String(ReadBmePressure()) + " mbar");
	Serial.println("SHT31 sensor temperature: " + String(ReadSht31Temp()) + " C");
	Serial.println("SHT31 sensor Humid: " + String(ReadSht31Humid()) + " %");
	Serial.println("Soil moisture: " + String(ReadSoil(10)) + "%");
	Serial.println("Current weight: " + String(ReadWeight(10)) + " gr");
}
void PurgeGsmBuffer(int i)
{
	delay(i);
	while (gsmSerial.available() > 0)
	{
		Serial.write(gsmSerial.read());
	}
}

int ReadGsmBuffer(int i=0)
{
  delay(i);
  if (gsmSerial.available()) 
  {
    while(gsmSerial.available())
    {
      char c = gsmSerial.read();  //gets one byte from serial buffer
      reply += c;
      delay(10);
    }
    Serial.println("\n");
    Serial.println(F("Reply: "));
    Serial.print(reply);
    if(reply.indexOf("OK") > 0)
    {
      Serial.println(F("SUCESS"));
      errorCount=0;
    }
    else if (reply.indexOf("ERROR") > 0)
    {
	ResetGSM();
    }
  }
reply="";
return errorCount;

}

void PutGsmToSleep()
{
	//Serial.println(F("GSM going to sleep"));
	gsmSerial.println(F("AT+CSCLK=1"));	//prepare for sleep mode when gsmWakePin is High
	PurgeGsmBuffer(1000);
	//gsmSerial.println(F("AT+CFUN=0"));	//prepare for gsm module for minimum funcionality
	//PurgeGsmBuffer(3000);
	digitalWrite(gsmWakePin, HIGH);
}
void WakeUpGsm()
{
	digitalWrite(gsmWakePin, LOW);	//Awake Gsm
	delay(1000);
	//gsmSerial.println(F("AT+CFUN=1"));	//prepare for gsm module for full funcionality
	//PurgeGsmBuffer(10000);
	gsmSerial.println(F("ATE1"));		//Switch on Echo
	PurgeGsmBuffer(1000);
}
void PutScaleToSleep()
{
	//Serial.println(F("Scale going to sleep"));
	scale.power_down();		// Put scale in sleep mode
}
void WakeUpScale()
{
	scale.power_up(); //Awake scale
	//Serial.println(F("Scale awake"));
}
void wakeUp()
{
	//Serial.println(F("Interrupt 1 Fired"));
	sleep_disable();
	detachInterrupt(0);
	detachInterrupt(1);

}
void wakeUp2()
{
	//Serial.println(F("Interrupt 2 Fired"));
	sleep_disable();
	detachInterrupt(0);
	detachInterrupt(1);

	if (wakeupSwitch == 0)
	{
		wakeupSwitch=1;
	}
	else if (wakeupSwitch == 1)
	{
		wakeupSwitch=2;
	}
}
void PutArduinoToSleepFull()
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);		// setting the sleep mode
	sleep_cpu();
	//Serial.println(F("Arduino going to sleep"));
	digitalWrite(LED_BUILTIN, HIGH);
	delay(500);
	digitalWrite(LED_BUILTIN, LOW);
	delay(1000);

	attachInterrupt(0, wakeUp, LOW);
	attachInterrupt(1, wakeUp2, LOW);
	sleep_enable();
	sleep_mode();

	clock.clearAlarm1();						// Clear the DS3231 alarm (ready for the next triggering)
	clock.clearAlarm2();						// Clear the DS3231 alarm (ready for the next triggering)

}
void PutArduinoToSleep()
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);		// setting the sleep mode
	sleep_cpu();
	//Serial.println(F("Waiting button press"));
	digitalWrite(LED_BUILTIN, HIGH);
	delay(500);
	digitalWrite(LED_BUILTIN, LOW);
	delay(500);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(500);
	digitalWrite(LED_BUILTIN, LOW);
	delay(1000);

	//attachInterrupt(0, wakeUp, LOW);
	attachInterrupt(1, wakeUp2, LOW);
	sleep_enable();
	sleep_mode();

	clock.clearAlarm1();						// Clear the DS3231 alarm (ready for the next triggering)
	clock.clearAlarm2();						// Clear the DS3231 alarm (ready for the next triggering)

}
void SignalForWakeUp()
{
	for (int i=0;i<10;i++)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		delay(50);
		digitalWrite(LED_BUILTIN, LOW);
	}
}
void SetupWakeUpAlarm(int i)
{
	if (i == 1)
	{
		clock.setAlarm1(0, 0, 0, 0, DS3231_MATCH_S);
	}
	else if (i == 30)
	{
		clock.setAlarm1(0, 0, 30, 0, DS3231_MATCH_M_S);
		clock.setAlarm2(0, 0, 00, DS3231_MATCH_M);
	}
	else if (i == 60)
	{
		clock.setAlarm2(0, 0, 00, DS3231_MATCH_M);
	}
	else
	{
		clock.setAlarm1(0, 0, 0, 0, DS3231_MATCH_S);
	}
}
void InitialGsmSetup()
{
  	gsmSerial.println(F("ATE0"));		//
	PurgeGsmBuffer(1000);
	gsmSerial.println(F("ATE1"));		//Switch on Echo
	PurgeGsmBuffer(1000);
    gsmSerial.println(F("AT+CBC"));	
    PurgeGsmBuffer(1000);
	gsmSerial.println(F("AT&D2"));		
	PurgeGsmBuffer(1000);
	gsmSerial.println(F("AT+CMGF=1"));	// put SMS module into Text mode
	PurgeGsmBuffer(1000);
	gsmSerial.println(F("AT+CIPSHUT"));	//close the GPRS PDP context
	PurgeGsmBuffer(1000);
	gsmSerial.println(F("AT+CSCLK=1"));	//prepare for sleep mode when gsmWakePin is High
	PurgeGsmBuffer(1000);
	//gsmSerial.println(F("AT+CFUN=1"));	//prepare for gsm module for full funcionality
	//PurgeGsmBuffer(1000);
}

void SetUpDs3231()
{
	clock.armAlarm1(false);
	clock.armAlarm2(false);
	clock.clearAlarm1();
	clock.clearAlarm2();
	clock.begin();
	//clock.setDateTime(__DATE__, __TIME__);

	// Disable square wave output (use alarm triggering)
	// setting 0Eh register on ds3231. very important! see technical specs of ds3231!!
	Wire.beginTransmission(0x68);
	Wire.write(0x0e);
	Wire.write(0b00110111);
	Wire.endTransmission();
}

void LedSignal(int ledRepeat, int ledDelay)
{
	if (ledRepeat==0 && ledDelay==0)
	{
		digitalWrite(6, LOW);
	}
	else if (ledRepeat==1 && ledDelay==1)
	{
		digitalWrite(6, HIGH);
	}
	else
	{
		for (int i=0;i<ledRepeat;i++)
			{
				digitalWrite(6, HIGH);
				delay(ledDelay);
				digitalWrite(6, LOW);
				delay(ledDelay);
			}
	}

}

 int UploadToIot()
{
	Serial.println(F("Uploading:..."));
	//gsmSerial.println(F("AT+CBC"));		//read battery status
	//battPercentGsm=ReadBattPercent();
	gsmSerial.println(F("AT+CREG?"));
	PurgeGsmBuffer(1000);
	gsmSerial.println(F("AT+CGATT?"));
	PurgeGsmBuffer(1000);
	gsmSerial.println(F("AT+CIPSHUT"));
	PurgeGsmBuffer(1000);
	gsmSerial.println(F("AT+CIPSTATUS"));
	PurgeGsmBuffer(1000);
	gsmSerial.println(F("AT+CIPMUX=0"));
	PurgeGsmBuffer(1000);
	gsmSerial.println(F("AT + CSTT = \"internet\",\"\",\"\""));
	//PurgeGsmBuffer(1000);
  ReadGsmBuffer(1000);
	gsmSerial.println(F("AT+CIICR"));
  PurgeGsmBuffer(1000);
	//ReadGsmBuffer(1000);
	gsmSerial.println(F("AT+CIFSR"));
  PurgeGsmBuffer(1000);
	//ReadGsmBuffer(1000);
	gsmSerial.println(F("AT+CIPSPRT=0"));
  PurgeGsmBuffer(1000);
	//ReadGsmBuffer(1000);
  gsmSerial.println(F("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\""));
  //PurgeGsmBuffer(2000);
  ReadGsmBuffer(2000);
  if (errorCount>0)
    return errorCount;
	gsmSerial.println(F("AT+CIPSEND"));
	ReadGsmBuffer(2000);
  if (errorCount>0)
    return errorCount;
  PurgeGsmBuffer(2000);
	gsmSerial.println(thingSpeakUpadate + "&field1=" + String(ReadSht31Temp())+ "&field2=" + String(ReadSht31Humid()) + "&field3="+ String(battPercent) + String(tare + ReadWeight(10)) + "&field8=" + String(ReadSoil(10)));	
	//gsmSerial.println(thingSpeakUpadate + "&field1=" + String(ReadSht31Temp())+ "&field2=" + String(ReadSht31Humid()) + "&field3="+ String(battPercent) + "&field4=" + String(ReadBmeTemperature()) + "&field5=" + String(ReadBmePressure()) + "&field6=" + String(ReadBmeHumid())+ "&field7=" + String(tare + ReadWeight(10)) + "&field8=" + String(ReadSoil(10)));
	gsmSerial.println(String(char(26)));
  	PurgeGsmBuffer(2000);
	//ReadGsmBuffer(2000);
  if (errorCount>0)
    return errorCount;
	gsmSerial.println(F("AT+CIPSHUT"));
	ReadGsmBuffer(5000);
  if (errorCount>0)
    return errorCount;
	//Serial.println(F("Finished uploading!"));
	return errorCount;
}

void setup()
{
	LedSignal(1,1);
	//clock.setDateTime(__DATE__, __TIME__);
	//setting up pins
	//----------------------------------------------
	pinMode(6, OUTPUT);		//led pin set for output
	digitalWrite(6, LOW);	//led set to low state

	Serial.begin(9600);					//Start serial communication
	analogReference(INTERNAL);

	//Setting up software serial communication
	//---------------------------------------------
	gsmSerial.begin(9600);
	//---------------------------------------------

	// setting up rtc communication trough i2c port (on A4 and A5)
	//---------------------------------------------
	Wire.begin();
	//---------------------------------------------

	//setting up DS3231 (alarms (clear), SQW, clock...)
	//---------------------------------------------
	SetUpDs3231();
	//---------------------------------------------

	//setting up gsm module initial state
	//---------------------------------------------
	InitialGsmSetup();
	//----------------------------------------------

	//Initial display on serial port
	//----------------------------------------------
	//PrintTimeAndDate();
	//----------------------------------------------

	//setting up pins
	//----------------------------------------------
	pinMode(LED_BUILTIN, OUTPUT);		//led pin set for output
	digitalWrite(LED_BUILTIN, LOW);	//integrated led set to low state

	pinMode(wakePin, INPUT_PULLUP);		//wakePin(2) set to input with pullUp resistor
	pinMode(wakePin2, INPUT_PULLUP);	//wakePin(3) set to input with pullUp resistor

	pinMode(gsmWakePin, OUTPUT);		//gsmWakePin set to Output
	digitalWrite(gsmWakePin, LOW);		//set gsmWakePin to low state (gsm module awake)
	//----------------------------------------------

	//Setting up HX711 scale
	//----------------------------------------------
	scale.set_scale(scaleCalibrationFactor);	//Calibration Factor obtained from calibrating sketch
	scale.tare();								//Reset the scale to 0  
	//----------------------------------------------


	//sht31 check
	//----------------------------------------------
	if (!sht31.begin(0x44))  // Set to 0x45 for alternate i2c addr
	{   
		Serial.println(F("Couldn't find valid SHT31"));
	}
	//----------------------------------------------

	//bme280 check
	//----------------------------------------------
	// if (!bme.begin())
	// {
	// 	Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
	// }
	//----------------------------------------------
  //DisplayMeasurementsOnSerialMonitor();
LedSignal(0,0);
}

void loop()
{
	//clock.setDateTime(__DATE__, __TIME__);
	if (wakeupSwitch==0)
	{
		LedSignal(1,1);
		if (flag==0)
		{
		PutScaleToSleep();
		PutGsmToSleep();
		flag=1;
		}
	SetupWakeUpAlarm(ArduinoWakeInterval);		//Options are: 1-every minute, 30-every half hour , 60-every hour
	LedSignal(0,0);
	PutArduinoToSleepFull();
	}
	//----------------------------------------------------------
	//Waking arduino if alarm or button (interrupt 1 or 2) is fired/pressed
	//----------------------------------------------------------
PrintTimeAndDate();
if (wakeupSwitch==0)
{
	ReadTime();
	if (min%uploadInterval==0)
	{
		Serial.println(F("Time for upload"));
		if (flag==1)
		{
			//Serial.println("flag=1");
			if (tmpMin!=min)
			{
				LedSignal(10,100);
				digitalWrite(6, HIGH);
				//Serial.println(F("preparing to upload"));
				WakeUpGsm();
				WakeUpScale();
				SignalForWakeUp();
				ResetScale(0);
				// do
				// {
				battPercent=ReadBattery(3);
				// } while (battPercentGsm<=0||battPercentGsm>100);
				
				//DisplayMeasurementsOnSerialMonitor();
				
				do
				{
					UploadToIot();
					//Serial.println("Error count: "+String(errorCount));
				}
				while (errorCount>0&&errorCount<10);
				if (errorCount==10)
				{
					errorCount=0;
					resetFunc();		//Reset arduino
				}
				flag=0;
				tmpMin=min;
				LedSignal(10,100);
			}
		}
	}
}
	else if(wakeupSwitch==1)
	{
		//Serial.println(F("Button pressed"));
		LedSignal(1,1);
		WakeUpGsm();
		WakeUpScale();
		SignalForWakeUp();
		tare=tare + ReadWeight(10);	//keep value before beekeeper changes anything. If there are previous values add them also
		Serial.println(tare);
		wakeupSwitch=2;
		attachInterrupt(1, wakeUp2, LOW);
		LedSignal(10,200);
		LedSignal(0,0);
		PutArduinoToSleep();

		//TODO make some led lights go red or green to indicate staus of the scale

		//DisplayMeasurementsOnSerialMonitor();
		//UploadToIot();
	}
	else if (wakeupSwitch==2)
	{
		LedSignal(10,200);
		LedSignal(1,1);
		//Serial.println(F("Second time button pressed"));
		WakeUpGsm();
		WakeUpScale();
		SignalForWakeUp();
		scale.tare();	//tare scales to cancel any changes it came from the beekeeper. In float tare we are keeping changes from the last measure and readings from the scale from this point will be added to that value
    do
    {
      battPercent=ReadBattery(1);
    } while (battPercent<=0||battPercent>100);
    //DisplayMeasurementsOnSerialMonitor();
    do
    {
      UploadToIot();
      //Serial.println("Error count: "+String(errorCount));
    }
    while (errorCount!=0);
		wakeupSwitch=0;
		LedSignal(0,0);
		//Serial.println(F("Continuing normal operation"));
	}

}