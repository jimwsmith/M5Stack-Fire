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
int mode = 0;

#define SAMPLINGFREQUENCY 40000
#define SAMPLING_TIME_US     ( 1000000UL/SAMPLINGFREQUENCY )
#define ANALOG_SIGNAL_INPUT        M5STACKFIRE_MICROPHONE_PIN

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
void setup()
{
  dacWrite(M5STACKFIRE_SPEAKER_PIN, 0); // make sure that the speaker is quite
  M5.Lcd.begin();
  M5.Power.begin();
  setup_scope();
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

void loop(void)
{
  int n;
  static int r, g, b;
  static int pixelNumber=0;// = random(0, M5STACK_FIRE_NEO_NUM_LEDS - 1);
  uint32_t nextTime = 0;

  //Check button status to switch device mode
  M5.update();
  M5.Lcd.setCursor(200, 0);
    if(M5.BtnA.wasPressed()) {
        M5.Lcd.printf("A");
        mode = 0;
        setup_scope();
    }
    if(M5.BtnB.wasPressed()) {
        M5.Lcd.printf("B");
        mode = 1;
        setup_spectrum();
    }
    if(M5.BtnC.wasPressed()) {
        M5.Lcd.fillScreen( BLACK );
        mode = 2;
    }

switch (mode) {
  case 0:
//  {
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
//  }
  case 1:
  {
    // record signal
    for (n = 1; n < SIGNAL_LENGTH; n++)
    {
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
    for (n = 1; n < SIGNAL_LENGTH; n++)
    {
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
    }
    pixelNumber++;
    if(pixelNumber>9)pixelNumber=0;
    pixels.setPixelColor(pixelNumber, pixels.Color(r, g, b));     
    pixels.show();
    delay(100);
    break;
//  default:
  }
}
