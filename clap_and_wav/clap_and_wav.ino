#include <SimpleFOC.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <TeensyVariablePlayback.h>

MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 9);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 4, 3, 1);

float target_voltage = 24;

// AUDIO CODE
#define SDCARD_CS_PIN 10
#define SDCARD_MOSI_PIN 11  // Teensy 4 ignores this, uses pin 11
#define SDCARD_SCK_PIN 13   // Teensy 4 ignores this, uses pin 13

const int myInput = AUDIO_INPUT_MIC;

// GUItool: begin automatically generated code
AudioPlaySdResmp playSdWav1;  //xy=248,492
AudioInputI2S audioInput;     //xy=249,395
AudioAmplifier amp1;          //xy=410,459
AudioAmplifier amp2;          //xy=413,518
AudioAnalyzeFFT1024 myFFT;    //xy=430,383
AudioOutputI2S audioOutput;   //xy=611,494
AudioConnection patchCord1(playSdWav1, 0, amp1, 0);
AudioConnection patchCord2(playSdWav1, 1, amp2, 0);
AudioConnection patchCord3(audioInput, 0, myFFT, 0);
AudioConnection patchCord4(amp1, 0, audioOutput, 0);
AudioConnection patchCord5(amp2, 0, audioOutput, 1);
AudioControlSGTL5000 audioShield;  //xy=423,316
// END AUDIO

float audiogain = 0.4;
float playbackrate = 1.0;
void setup() {

  // AUDIO CODE
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }

  playSdWav1.enableInterpolation(true);
  // playSdWav1.setPlaybackRate(playbackrate);

  AudioMemory(120);
  // Enable the audio shield and set the output volume.
  audioShield.enable();
  audioShield.inputSelect(myInput);
  audioShield.volume(0.5);
  // Configure the window algorithm to use
  myFFT.windowFunction(AudioWindowHanning1024);


  // amp1.gain(audiogain);
  // amp2.gain(audiogain);
  playSdWav1.playWav("/TEUN8.WAV");

  // END AUDIO
  AudioNoInterrupts();
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
  AudioInterrupts();
}

unsigned long oldMillis = 0;
unsigned long waittime = 2500;
bool modus = false;
float fftdata;
void loop() {
  AudioNoInterrupts();
  motor.loopFOC();
  if (modus) {
    motor.move(target_voltage);
  } else {
    motor.move(-1.5);
  }
  AudioInterrupts();


  if (millis() > (oldMillis + waittime)) {
    // playSdWav1.playWav("/TEUN8.WAV");
    modus = !modus;
    Serial.print(modus);
    oldMillis = millis();
    // playbackrate -= 0.1;
    playSdWav1.setPlaybackRate(0.8);

    // Serial.print("Playback rate: ");
    // Serial.println(playbackrate);
    playSdWav1.playWav("/TEUN8A.WAV");

    if (waittime < 2500) {
      waittime += 20;
    }
  }

  if (myFFT.available()) {
    fftdata = myFFT.read(29);

    if (fftdata > 0.2) {
      Serial.println(fftdata);
      waittime = 20;
    }
  }
}