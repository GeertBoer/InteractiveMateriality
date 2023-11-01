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
// #define SDCARD_MOSI_PIN 11  // Teensy 4 ignores this, uses pin 11
// #define SDCARD_SCK_PIN 13   // Teensy 4 ignores this, uses pin 13

const int myInput = AUDIO_INPUT_MIC;
const char* filename = "/TEUN8A.WAV";

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

float audiogain = 1.0;
float playbackrate = 0.5;
void setup() {
  // AUDIO CODE
  SPI.setMOSI(11);
  SPI.setSCK(13);
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
  audioShield.volume(1.0);
  // Configure the window algorithm to use
  myFFT.windowFunction(AudioWindowHanning1024);
  // amp1.gain(audiogain);
  // amp2.gain(audiogain);
  // playSdWav1.playWav(filename);

  // END AUDIO
  // AudioNoInterrupts();

  Serial.begin(115200);
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
  // AudioInterrupts();
}

unsigned long oldMillis = 0;
unsigned long waittime = 2500;
bool modus = false;
float fftdata;

unsigned long olddifdelay = 0;
unsigned long difdelay = 20;
float oldAngle = 0.0;
float dif = 0.0;
void loop() {
  AudioNoInterrupts();
  motor.loopFOC();
  sensor.update();
  if (modus) {
    motor.move(target_voltage);
  } else {
    motor.move(-1.0);
  }
  delay(1);


  if (millis() > (oldMillis + waittime)) {
    modus = !modus;
    Serial.print(modus);
    oldMillis = millis();

    AudioInterrupts();
    if (modus) {
      playSdWav1.playWav(filename);
      Serial.println("Start");
    }

    // if (!playSdWav1.isPlaying()) {
    //   playSdWav1.playWav("/TEUN8A.WAV");
    // }

    delay(1);

    if (waittime < 2500) {
      waittime += 100;
    }
  }

  AudioInterrupts();

  if (myFFT.available()) {
    fftdata = myFFT.read(29);

    if (fftdata > 0.12) {
      Serial.print("fft trigger: ");
      Serial.println(fftdata);
      waittime = 80;
    }
  }

  if (millis() > (olddifdelay + difdelay)) {
    olddifdelay = millis();
    dif = oldAngle - sensor.getAngle();
    dif = dif * -1.0;
    // if (dif < 0.0) {
    //   dif = dif * -1.0;
    // }

    // playbackrate = map(dif, 0.0, 0.7, 0.3, 2.0); // dit is de OG

    if (modus) {
      playbackrate = map(dif, 0.0, 0.7, 0.3, 2.0); // dit is de OG
    }
    else {
      Serial.println("else");
      playbackrate = map(dif, 0.0, 0.7, 0.3, 1.0); // dit is de OG
    }
    

    playSdWav1.setPlaybackRate(playbackrate);
    Serial.print("Playback rate: ");
    Serial.println(playbackrate);
    // Serial.println(sensor.getAngle());
    oldAngle = sensor.getAngle();
  }

  if (!playSdWav1.isPlaying()) {
    playSdWav1.playWav(filename);
  }
}