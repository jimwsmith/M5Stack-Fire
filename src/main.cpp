/************************************************************************

  M5StackFire Discovery simple Spectrum example

  The microphone signal is sampled and a frequency analysis is performed.


  Please install the arduinoFFT library.
  You will find it in the library manager or you can get it from github:
  https://github.com/kosme/arduinoFFT


  M5StackFire         September 2018, ChrisMicro

************************************************************************/
#include "Version.h"
#include <M5Stack.h>
#include "arduinoFFT.h"
#include <Adafruit_NeoPixel.h>
#include "utility/MPU9250.h"
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "Adafruit_PM25AQI.h"

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
Adafruit_SGP30 sgp;

/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

MPU9250 IMU; // new a MPU9250 object

#define M5STACK_FIRE_NEO_NUM_LEDS 10
#define M5STACK_FIRE_NEO_DATA_PIN 15

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(M5STACK_FIRE_NEO_NUM_LEDS, M5STACK_FIRE_NEO_DATA_PIN, NEO_GRB + NEO_KHZ800);

//#include <WiFi.h>

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

#define M5STACKFIRE_MICROPHONE_PIN 34
#define M5STACKFIRE_SPEAKER_PIN 25 // speaker DAC, only 8 Bit

#define HORIZONTAL_RESOLUTION 320
#define VERTICAL_RESOLUTION   240
#define POSITION_OFFSET_Y      20
#define SIGNAL_LENGTH 512

double oldSignal[SIGNAL_LENGTH];
double adcBuffer[SIGNAL_LENGTH];
double vImag[SIGNAL_LENGTH];
int mode = 0; //Which App are we running in main loop

#define SAMPLINGFREQUENCY 40000
#define SAMPLING_TIME_US     ( 1000000UL/SAMPLINGFREQUENCY )
#define ANALOG_SIGNAL_INPUT        M5STACKFIRE_MICROPHONE_PIN

void setup_pm25() {
  Serial.println("Adafruit PMSA003I Air Quality Sensor");

  // Wait one second for sensor to boot up!
  delay(1000);

  if (! aqi.begin_I2C()) {      // connect to the sensor over I2C
    M5.Lcd.println("Could not find PM 2.5 sensor!");
    delay(5000);
  }

  M5.Lcd.println("PM25 found!");
}


void setup_scope()
{
  dacWrite(M5STACKFIRE_SPEAKER_PIN, 0); // make sure that the speaker is quite
  M5.Lcd.fillScreen( BLACK );
  M5.Lcd.fillRect(10, 1, 150, 160, BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);  //M5.Lcd.setTextSize(3);
  M5.Lcd.setTextSize(1);

  M5.Lcd.println("MIC Oscilloscope");
  M5.Lcd.print("sampling frequency: "); M5.Lcd.print(1000000 / SAMPLING_TIME_US); M5.Lcd.println(" Hz");
}

void setup_spectrum()
{
  M5.Lcd.fillScreen( BLACK );
  M5.Lcd.fillRect(10, 1, 150, 160, BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);  //M5.Lcd.setTextSize(3);
  M5.Lcd.setTextSize(1);

  M5.Lcd.println("MIC Spectrum");
  M5.Lcd.print("max. frequency: "); M5.Lcd.print(SAMPLINGFREQUENCY / 2); M5.Lcd.println(" Hz");
  M5.Lcd.setTextSize(2);
}

void setup_sgp30(){
  M5.Lcd.fillScreen( BLACK );
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);  //M5.Lcd.setTextSize(3);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("SGP30 test");

  if (! sgp.begin()){
    M5.Lcd.println("Sensor not found :(");
    delay(3000);
  }
  M5.Lcd.print("Found SGP30 serial #");
  M5.Lcd.print(sgp.serialnumber[0], HEX);
  M5.Lcd.print(sgp.serialnumber[1], HEX);
  M5.Lcd.println(sgp.serialnumber[2], HEX);

  // If you have a baseline measurement from before you can assign it to start, to 'self-calibrate'
  //sgp.setIAQBaseline(0x8E68, 0x8F41);  // Will vary for each sensor!

}
void setup()
{
  dacWrite(M5STACKFIRE_SPEAKER_PIN, 0); // make sure that the speaker is quite
  M5.Lcd.begin();
  M5.Power.begin();
  mode=0; //Start in scope mode
  pixels.begin(); //RGB Bars
  IMU.initMPU9250(); //Gyro with temp sensor
}

void waitForAutoTrigger()
{
  uint32_t triggerLevel = 512;
  uint32_t hysteresis   =  10;
  uint32_t timeOut_ms   = 100;

  uint32_t timeOutLimit = millis() + timeOut_ms;
  uint32_t timeOutFlag = false;

  uint32_t adcValue = 0;

  adcValue = analogRead( ANALOG_SIGNAL_INPUT );

  // wait for low level
  while ( ( adcValue > triggerLevel - hysteresis ) && !timeOutFlag )
  {
    adcValue = analogRead( ANALOG_SIGNAL_INPUT );

    if ( millis() > timeOutLimit ) timeOutFlag = 1 ;
  }

  if ( !timeOutFlag )
  {
    // wait for high level
    while ( ( adcValue < triggerLevel + hysteresis ) && ( millis() < timeOutLimit ) )
    {
      adcValue = analogRead(ANALOG_SIGNAL_INPUT);
    }
  }

}
void showSpectrumSignal()
{
  int n;
  int oldx;
  int oldy;
  int oldSig;
  int x, y;

  for (n = 0; n < SIGNAL_LENGTH/2; n++)
  {
    x = n;
    y = map(adcBuffer[n], 0, 512, VERTICAL_RESOLUTION, POSITION_OFFSET_Y);

    if (n > 0)
    {
      // delete old line element
      M5.Lcd.drawLine(oldx , oldSig, x, oldSignal[n], BLACK );

      // draw new line element
      if (n < SIGNAL_LENGTH - 1) // don't draw last element because it would generate artifacts
      {
        M5.Lcd.drawLine(oldx,    oldy, x,            y, GREEN );
      }
    }
    oldx = x;
    oldy = y;
    oldSig = oldSignal[n];
    oldSignal[n] = y;
  }
}

void showScopeSignal()
{
  int n;
  int oldx;
  int oldy;
  int oldSig;
  int x, y;

  for (n = 0; n < SIGNAL_LENGTH; n++)
  {
    x = n;
    y = map(adcBuffer[n], 0, 4096, VERTICAL_RESOLUTION, POSITION_OFFSET_Y);

    if (n > 0)
    {
      // delete old line element
      M5.Lcd.drawLine(oldx , oldSig, x, oldSignal[n], BLACK );

      // draw new line element
      if (n < SIGNAL_LENGTH - 1) // don't draw last element because it would generate artifacts
      {
        M5.Lcd.drawLine(oldx,    oldy, x,            y, GREEN );
      }
    }
    oldx = x;
    oldy = y;
    oldSig = oldSignal[n];
    oldSignal[n] = y;
  }
}

double AdcMeanValue = 0;
int counter = 0;

void loop(void)
{
  // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
  //float temperature = 22.1; // [°C]
  //float humidity = 45.2; // [%RH]
  //sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

  int n;
  static int r, g, b;
  static int pixelNumber=0;// = random(0, M5STACK_FIRE_NEO_NUM_LEDS - 1);
  uint32_t nextTime = 0;

  M5.update();   //Check button status to switch device mode
  if ((M5.BtnA.wasPressed())||(M5.BtnB.wasPressed())||(M5.BtnC.wasPressed())) {
    M5.Lcd.fillScreen( BLACK );
    if(M5.BtnA.wasPressed()) {
        mode  += 1; //Increment App
        if (mode > 4) mode = 0;
    }
    if(M5.BtnB.wasPressed()) {
        M5.Lcd.printf("B");
        mode -= 1; //Decrement App
        if (mode < 0) mode = 4;
    }
    if(M5.BtnC.wasPressed()) {
        mode = 0;
    }
    switch (mode) {
      case 0:
        setup_scope();
        break;
      case 1:
        setup_spectrum();
        break;
      case 2:
        //Battery SOC builtin
        break;
      case 3:
        setup_sgp30();
        break;
      case 4:
        setup_pm25();
        break;
    }
  }
  switch (mode) {
    case 0:
      waitForAutoTrigger();
      // record signal
      for (n = 0; n < SIGNAL_LENGTH; n++)
      {
        adcBuffer[n] = analogRead( ANALOG_SIGNAL_INPUT );

        // wait for next sample
        while (micros() < nextTime);
        nextTime = micros() + SAMPLING_TIME_US;
      }
      showScopeSignal();
      break;
    case 1:
      {
      // record signal
      for (n = 1; n < SIGNAL_LENGTH; n++) {
        double v = analogRead( ANALOG_SIGNAL_INPUT );
        AdcMeanValue += (v - AdcMeanValue) * 0.001;
        adcBuffer[n] = v - AdcMeanValue;
        // wait for next sample
        while (micros() < nextTime);
        nextTime = micros() + SAMPLING_TIME_US;
      }

      FFT.Windowing(adcBuffer, SIGNAL_LENGTH, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
      FFT.Compute(adcBuffer, vImag, SIGNAL_LENGTH, FFT_FORWARD); /* Compute FFT */
      FFT.ComplexToMagnitude(adcBuffer, vImag, SIGNAL_LENGTH); /* Compute magnitudes */
      //int x = FFT.MajorPeak(adcBuffer, SIGNAL_LENGTH, 1000000UL / SAMPLING_TIME_US);//SAMPLINGFREQUENCY
      int x = FFT.MajorPeak(adcBuffer, SIGNAL_LENGTH, SAMPLINGFREQUENCY);

      int maxAmplitudeDB = 0;
      for (n = 1; n < SIGNAL_LENGTH; n++) {
        int a = log10(adcBuffer[n]) * 20 - 54.186; // convert amplitude to dB scale, dB relative to log10(512samples)*20=54.186dB
        if (a > maxAmplitudeDB) maxAmplitudeDB = a;
        adcBuffer[n] = (a + 30) * 5; // scale for TFT display
        vImag[n] = 0; // clear imaginary part
      }

      showSpectrumSignal();
      //Display peak information
      M5.Lcd.fillRect(200, 0, 119, 40, BLUE);
      M5.Lcd.setCursor(210, 1);
      M5.Lcd.print(x); M5.Lcd.print(" Hz");
      M5.Lcd.setCursor(210, 21);
      M5.Lcd.print(maxAmplitudeDB); M5.Lcd.print(" dB");
      break;
      }
    case 2:
      pixels.setPixelColor(pixelNumber, pixels.Color(0, 0, 0));  //Turn off last pixel   
      if (pixelNumber == 0) {
        r = 1<<random(0, 7);
        g = 1<<random(0, 7);
        b = 1<<random(0, 7);
      }
      pixelNumber++;
      if(pixelNumber>9)pixelNumber=0;
      pixels.setPixelColor(pixelNumber, pixels.Color(r, g, b));     
      pixels.show();
      
      M5.Lcd.fillScreen(BLUE);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0,0);
      M5.lcd.println("Battery Status");
      M5.lcd.print("State of Charge ");
      M5.lcd.println(M5.Power.getBatteryLevel());
      M5.lcd.print("Full Charge? ");
      M5.lcd.println(M5.Power.isChargeFull());
      M5.lcd.print("Charging now? ");
      M5.lcd.println(M5.Power.isCharging());
      M5.lcd.print("Temp ");
      IMU.tempCount = IMU.readTempData();
      IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;
      M5.lcd.println(IMU.temperature,1);
      delay(100);
      break;
    case 3:
      // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
      //float temperature = 22.1; // [°C]
      //float humidity = 45.2; // [%RH]
      //sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0,0);

      if (! sgp.IAQmeasure()) {
        M5.lcd.println("Measurement failed");
        return;
      }
      M5.lcd.print("TVOC "); M5.lcd.print(sgp.TVOC); M5.lcd.print(" ppb  ");
      M5.lcd.print("eCO2 "); M5.lcd.print(sgp.eCO2); M5.lcd.println(" ppm");

      if (! sgp.IAQmeasureRaw()) {
        M5.lcd.println("Raw Measurement failed");
        return;
      }
      M5.lcd.print("Raw H2 "); M5.lcd.println(sgp.rawH2);
      M5.lcd.print("Raw Ethanol "); M5.lcd.print(sgp.rawEthanol); M5.lcd.println("");
    
      delay(1000);

      counter++;
      if (counter == 30) {
        counter = 0;

        uint16_t TVOC_base, eCO2_base;
        if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
          M5.lcd.println("Failed to get baseline readings");
          return;
        }
        M5.lcd.print("****Baseline values: eCO2: 0x"); M5.lcd.println(eCO2_base, HEX);
        M5.lcd.print(" & TVOC: 0x"); M5.lcd.println(TVOC_base, HEX);
      }

      break;
    case 4:
      PM25_AQI_Data data;
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0,0);
      
      if (! aqi.read(&data)) {
        Serial.println("Could not read from AQI");
        delay(500);  // try again in a bit!
        break;
      }
//      M5.lcd.println("AQI reading success");

      M5.lcd.print(F("Std PM 1.0: ")); M5.lcd.println(data.pm10_standard);
      M5.lcd.print(F("Std PM 2.5: ")); M5.lcd.println(data.pm25_standard);
      M5.lcd.print(F("Std PM 10: ")); M5.lcd.println(data.pm100_standard);
      M5.lcd.print(F("Env PM 1.0: ")); M5.lcd.println(data.pm10_env);
      M5.lcd.print(F("Env PM 2.5: ")); M5.lcd.println(data.pm25_env);
      M5.lcd.print(F("Env PM 10: ")); M5.lcd.println(data.pm100_env);
      M5.lcd.print(F(" > 0.3um / 0.1L air:")); M5.lcd.println(data.particles_03um);
      M5.lcd.print(F(" > 0.5um / 0.1L air:")); M5.lcd.println(data.particles_05um);
      M5.lcd.print(F(" > 1.0um / 0.1L air:")); M5.lcd.println(data.particles_10um);
      M5.lcd.print(F(" > 2.5um / 0.1L air:")); M5.lcd.println(data.particles_25um);
      M5.lcd.print(F(" > 5.0um / 0.1L air:")); M5.lcd.println(data.particles_50um);
      M5.lcd.print(F(" > 10 um / 0.1L air:")); M5.lcd.println(data.particles_100um);
      delay(1000);
      break;
  //  default:
  }
}
