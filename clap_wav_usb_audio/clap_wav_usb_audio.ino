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

float target_voltage = 8;

// AUDIO CODE
#define SDCARD_CS_PIN 10
// #define SDCARD_MOSI_PIN 11  // Teensy 4 ignores this, uses pin 11
// #define SDCARD_SCK_PIN 13   // Teensy 4 ignores this, uses pin 13

const int myInput = AUDIO_INPUT_MIC;
const char* filename = "/TEUN8A.WAV";


// GUItool: begin automatically generated code
AudioPlaySdResmp           playSdWav1;     //xy=310,384
AudioInputI2S            audioInput;     //xy=311,287
AudioAmplifier           amp1;           //xy=472,351
AudioAmplifier           amp2;           //xy=475,410
AudioAnalyzePeak         peak1;          //xy=490,282
AudioOutputUSB           usb1;           //xy=668,378
AudioConnection          patchCord1(playSdWav1, 0, amp1, 0);
AudioConnection          patchCord2(playSdWav1, 1, amp2, 0);
AudioConnection          patchCord3(audioInput, 0, peak1, 0);
AudioConnection          patchCord4(amp1, 0, usb1, 0);
AudioConnection          patchCord5(amp2, 0, usb1, 1);
AudioControlSGTL5000     audioShield;    //xy=485,208
// GUItool: end automatically generated code



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
  audioShield.micGain(1);

  // Configure the window algorithm to use

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

  Serial.println("RESTARTED.");
}

unsigned long oldMillis = 0;
unsigned long maxwait = 2000;
unsigned long waittime = maxwait;
unsigned long waitdecay = 20;
bool modus = false;
float fftdata;

unsigned long olddifdelay = 0;
unsigned long difdelay = 20;
float oldAngle = 0.0;
float dif = 0.0;

float n;
int i;

float maxpeak = 0.0;
float peak = 0.0;
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
    // Serial.print(modus);
    oldMillis = millis();

    AudioInterrupts();
    if (modus) {
      playSdWav1.playWav(filename);
      maxpeak = 0.0;
    }

    delay(1);

    if (waittime < maxwait) {
      waittime += waitdecay;
    }
  }

  AudioInterrupts();

  if (peak1.available()) {
    peak = peak1.read();
    if (peak < 0.95) {
      if (peak > maxpeak) {
        maxpeak = peak;
        Serial.println(maxpeak);
      }

      if (peak >= 0.6) {
        waittime = random(5, 150);
      }
    }
  }

  if (millis() > (olddifdelay + difdelay)) {
    olddifdelay = millis();
    dif = oldAngle - sensor.getAngle();
    dif = dif * -1.0;

    if (modus) {
      playbackrate = map(dif, 0.0, 0.7, 0.3, 2.0);  // dit is de OG
    } else {
      playbackrate = map(dif, 0.0, 0.7, 0.3, 1.0);  // dit is de OG
    }

    playSdWav1.setPlaybackRate(playbackrate);

    oldAngle = sensor.getAngle();
  }

  if (!playSdWav1.isPlaying()) {
    playSdWav1.playWav(filename);
  }
}