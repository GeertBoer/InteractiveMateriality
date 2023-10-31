#include <SimpleFOC.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 9);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 4, 3, 1);

float target_voltage = 24;

// AUDIO CODE
const int myInput = AUDIO_INPUT_MIC;

AudioInputI2S audioInput;  // audio shield: mic or line-in
// AudioSynthWaveformSine sinewave;
AudioAnalyzeFFT1024 myFFT;
AudioOutputI2S audioOutput;  // audio shield: headphones & line-out

AudioConnection patchCord1(audioInput, 0, myFFT, 0);
AudioControlSGTL5000 audioShield;
// END AUDIO


void setup() {
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 5;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  Serial.begin(115200);
  motor.init();
  motor.initFOC();

  // AUDIO CODE
  AudioMemory(12);
  // Enable the audio shield and set the output volume.
  audioShield.enable();
  audioShield.inputSelect(myInput);
  audioShield.volume(0.5);
  // Configure the window algorithm to use
  myFFT.windowFunction(AudioWindowHanning1024);
  // END AUDIO
}

unsigned long oldMillis = 0;
unsigned long waittime = 1000;
bool modus = false;
float fftdata;
void loop() {
  motor.loopFOC();
  if (modus) {
    motor.move(target_voltage);
  } else {
    motor.move(-2);
  }


  if (millis() > (oldMillis + waittime)) {
    modus = !modus;
    // Serial.print(modus);
    oldMillis = millis();

    if (waittime < 1500) {
      waittime += 20;
    }
  }

  if (myFFT.available()) {
    fftdata = myFFT.read(26);

    if (fftdata > 0.10) {
      Serial.println(fftdata);
      // delay(5000);
      waittime = 50;
    }
  }
}