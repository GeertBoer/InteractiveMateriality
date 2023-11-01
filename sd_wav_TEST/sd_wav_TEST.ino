// Plays a Wav (16-bit signed) PCM audio file at slower or faster rate
// this example requires an uSD-card inserted to teensy 3.6 with a file called DEMO.WAV
#include <Arduino.h>
#include <Audio.h>
#include <SD.h>
#include <TeensyVariablePlayback.h>

// GUItool: begin automatically generated code
AudioPlaySdResmp         playSdWav1;     //xy=324,457
AudioOutputI2S           i2s2;           //xy=840.8571472167969,445.5714416503906
AudioConnection          patchCord1(playSdWav1, 0, i2s2, 0);
AudioConnection          patchCord2(playSdWav1, 0, i2s2, 1);
AudioControlSGTL5000     audioShield;
// GUItool: end automatically generated code


char* _filename = "/TEUN2.WAV";
unsigned long lastSamplePlayed = 0;

void setup() {
    Serial.begin(57600);

    if (!(SD.begin(10))) {
        // stop here if no SD card, but print a message
        while (1) {
            Serial.println("Unable to access the SD card");
            delay(500);
        }
    }

    AudioMemory(24);

    audioShield.enable();
    audioShield.volume(0.5);

    playSdWav1.enableInterpolation(true);

    Serial.println("playing...");
}

void loop() {

    unsigned currentMillis = millis();
    if (currentMillis > lastSamplePlayed + 500) {
        if (!playSdWav1.isPlaying()) {
            playSdWav1.playWav(_filename);
            lastSamplePlayed = currentMillis;
            
            Serial.print("all=");
            Serial.print(AudioProcessorUsage());
            Serial.print(",");
            Serial.print(AudioProcessorUsageMax());
            Serial.print("    ");
            Serial.print("Memory: ");
            Serial.print(AudioMemoryUsage());
            Serial.print(",");
            Serial.print(AudioMemoryUsageMax());
            Serial.println();
        }
    }
    delay(10);
}


namespace std {
    void __throw_bad_function_call() {}
    void __throw_length_error(char const*) {}
}