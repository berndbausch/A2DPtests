/*
  Streaming of sound data with Bluetooth to other Bluetooth device.
  We generate 2 tones which will be sent to the 2 channels.
  
  Copyright (C) 2020 Phil Schatzmann
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "AudioTools.h"
#include "BluetoothA2DPSource.h"
#include <math.h> 

#define DEVICE "H800 Logico"

AudioInfo info(44100, 2, 16);
SineWaveGenerator<int16_t> waveForm(32000);      // subclass of SoundGenerator with max amplitude of 32000
GeneratedSoundStream<int16_t> input(waveForm);      // Stream takes a generator as parameter

/****************************  Bluetooth stuff starts here ****************************/
BluetoothA2DPSource a2dp_source;

// for esp_a2d_connection_state_t see 
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_a2dp.html#_CPPv426esp_a2d_connection_state_t
esp_a2d_connection_state_t conn_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;

/***************************  data callback  **********************************/

// The supported audio codec in ESP32 A2DP is SBC. SBC audio stream is encoded
// from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data

// This function simply copies samples of the current note to the audio frame
int32_t get_data_frames(uint8_t *frame, int32_t framecount) {  
  uint8_t *frameptr;
  size_t nread = 0;
    
  frameptr = frame;
  do {
    size_t n = input.readBytes(frameptr, framecount);
    frameptr += n;
    nread += n;
  }
  while (nread<framecount);
  
  return framecount;
}

/***********************  Connection callback  ***************************/

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr){
  conn_state = state;
  Serial.println(a2dp_source.to_str(state));
}

/****************************  End of Bluetooth stuff  *******************************/

/****************  setup(): Configure callbacks for connection state and data, 
                   set volume and connect to device                         **********/
void setup() {
  char out[256];

  Serial.begin(115200);

  //Serial.println("**************************** Waiting three seconds...");
  //delay(3000);

/**********************  Audio setup: Generator and stream  **************************/
  //AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Info);

  // Setup wave form
  // Activate the stream with the AudioInfo defined above: 44kHz, 2 channels, 16 bit samples
  input.begin(info);
  // Generate a sound wave with the same AudioInfo and the frequency of B4 
  waveForm.begin(info, 440.0); 

/*********  Bluetooth setup: Callbacks for connection state change and data,  **********/
/*********                   define volume and launch the connection process  **********/
  // Configure automatic reconnection to the most recent device.
  // Default is true.
  // a2dp_source.set_auto_reconnect(false);

  // Set up the connection and data callbacks
  a2dp_source.set_on_connection_state_changed(connection_state_changed);
  a2dp_source.set_data_callback(get_data_frames);

  a2dp_source.set_volume(30);
  a2dp_source.start(DEVICE);   
}

/*************  loop() ramps the frequency up and down continuously  *************/
int lastmillis = 0;
float frequency = 400.0;
float deltafreq = 0.3;
void loop() {
  int currmillis;
  if (conn_state==ESP_A2D_CONNECTION_STATE_CONNECTED) {
    currmillis = millis();
    if (currmillis-lastmillis>10) {
      char out[256];

      lastmillis = currmillis;
      frequency += deltafreq;
      if (frequency>500.0 || frequency<400.0)
        deltafreq = -deltafreq;
      waveForm.setFrequency(frequency);
      sprintf(out,"freq %.0f, delta %.0f, lastmillis %d", frequency, deltafreq, lastmillis);
      Serial.println(out);
    }
  }
     
  // to prevent watchdog in release > 1.0.6
  delay(1);
}
