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
#include "BluetoothA2DPSource.h"
#include <math.h> 

#define c3_freq  130.81
#define DEVICE "H800 Logico"

struct sound {
  Frame *frame;
  int currsample;
  int nsamples;
  float frequency;
};

//                 0=C      1=C#    2=D     3=D#    4=E     5=F    6=F#     7=G    8=G#     9=A    10=A#    11=B
float freq3[] = { 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 184.99, 195.99, 207.64, 219.99, 233.07, 246.93 };
float freq4[] = { 261.61, 277.17, 293.65, 311.11, 329.61, 349.20, 369.97, 391.97, 415.27, 439.96, 466.12, 493.84 };

struct sound octave3[12];
struct sound octave4[12];
int note_to_play = -1;

/**********************  Create the waveforms  *****************************/

// Helper function to build one stereo sine wave
void make_frame(Frame *frame, int nsamples, float frequency) {
  int sample;
  float m_time = 0;
  float m_phase = 0;
  float m_deltatime = 1.0/44100.0;
  float pi_2 = PI * 2.0;
  float m_amplitude = 10000.0;  // -32,768 to 32,767

  for (sample = 0; sample < nsamples; ++sample) {
    float angle = pi_2 * frequency * m_time + m_phase;
    frame[sample].channel1 = m_amplitude * sin(angle);
    frame[sample].channel2 = frame[sample].channel1;
    m_time += m_deltatime;
    /*if (angle>=pi_2) 
      return;*/
  }
}

void setup_sounds() {
  char out[256];

  Serial.println("************* octave 3:");
  for (int i=0; i<12; i++) {
    octave3[i].frequency = freq3[i];
    octave3[i].currsample = 0;
    octave3[i].nsamples = int(44100.0/octave3[i].frequency) + 1;
    octave3[i].frame = (Frame *)malloc(sizeof(Frame)*octave3[i].nsamples);
    make_frame(octave3[i].frame, octave3[i].nsamples, octave3[i].frequency);
    sprintf(out, "i %d: F %f, ns %d, frame 0x%x; ", i, octave3[i].frequency, octave3[i].nsamples, octave3[i].frame);
    Serial.print(out);
  }
  Serial.println();

  Serial.println("************* octave 4:");
  for (int i=0; i<12; i++) {
    octave4[i].frequency = freq4[i];
    octave4[i].currsample = 0;
    octave4[i].nsamples = int(44100.0/octave4[i].frequency) + 1;
    octave4[i].frame = (Frame *)malloc(sizeof(Frame)*octave4[i].nsamples);
    make_frame(octave4[i].frame, octave4[i].nsamples, octave4[i].frequency);
    sprintf(out, "F %f, ns %d, frame 0x%x; ", octave4[i].frequency, octave4[i].nsamples, octave4[i].frame);
    Serial.print(out);
  }
  Serial.println();
}

/************************  Waveforms end  ********************************/

int keytonote(int key) {
  switch(key) {
    case 'z': return 0;
    case 's': return 1;
    case 'x': return 2;
    case 'd': return 3;
    case 'c': return 4;
    case 'v': return 5;
    case 'g': return 6;
    case 'b': return 7;
    case 'h': return 8;
    case 'n': return 9;
    case 'j': return 10;
    case 'm': return 11;
    default: return -1;
  }
}

/****************************  Bluetooth stuff starts here ****************************/
BluetoothA2DPSource a2dp_source;

// for esp_a2d_connection_state_t see 
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_a2dp.html#_CPPv426esp_a2d_connection_state_t
esp_a2d_connection_state_t conn_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;

/***************************  data callback  **********************************/

// The supported audio codec in ESP32 A2DP is SBC. SBC audio stream is encoded
// from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data

// This function simply copies samples of the current note to the audio frame
int32_t get_data_frames(Frame *frame, int32_t framecount) {  
  char out[256];
  int f;
  int n = note_to_play;
  static int lastnote = -1;

  if (n>=0 && n<12) {
    struct sound *note = &octave3[n];

    if (n!=lastnote) {
      /*
       * TODO:
       * Allow the previous note to complete until its last sample
       */
      lastnote = n;
      note->currsample = 0; // This is a new note. Ensure we start at the first sample.
      sprintf(out, "n %d, frame_count %d, currsample %d, nsamples %d", n, framecount, note->currsample, note->nsamples);
      Serial.println(out);
    }
  
    for (f = 0; f < framecount; f++) {
      if (note->currsample >= note->nsamples) // wrap around
        note->currsample = 0;
      frame[f] = note->frame[note->currsample];
      note->currsample++;
    }
  }
  else
    // to prevent watchdog in release > 1.0.6
    delay(1);
  
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

  Serial.println("**************************** Waiting three seconds...");
  delay(3000);

  setup_sounds();

  // Configure automatic reconnection to the most recent device.
  // Default is true.
  // a2dp_source.set_auto_reconnect(false);

  // Set up the connection and data callbacks
  a2dp_source.set_on_connection_state_changed(connection_state_changed);
  a2dp_source.set_data_callback_in_frames(get_data_frames);
  
  a2dp_source.set_volume(30);
  a2dp_source.start(DEVICE);   
}

/****************  loop() changes to the next note every second  *******************/
int lastmillis = 0;
void loop() {
  if (conn_state==ESP_A2D_CONNECTION_STATE_CONNECTED) {
    int now = millis();
    if (now-lastmillis>1000) {  // After one second, go to the next note
      note_to_play++;
      if (note_to_play<0 || note_to_play>=12)
        note_to_play = 0;
      Serial.print("Playing note "); Serial.println(note_to_play);
      lastmillis = now;
    }
  }

  // to prevent watchdog in release > 1.0.6
  delay(1);
}
