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

// The frequencies for octave 0
//                 0=C      1=C#    2=D     3=D#    4=E     5=F    6=F#     7=G    8=G#     9=A    10=A#    11=B
float freq[] =  { 16.35,	 17.32,  18.35,  19.45,	 20.6,	21.83,	23.12,	24.5,	 25.96,	   27.5,	 29.14, 	30.87 };

// 12 notes for octave 0
// Notes for higher octaves are created when playing, by changing the step through the samples.
// oct 0 step 1,     oct 1 step 2,      oct 2 step 4   etc.
struct sound notes[12];

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
  }
}

// This function generates the samples and other information for the 12 notes in octave 0
void setup_sounds() {
  char out[256];
  static int totalbytes = 0;

  Serial.println("************* notes:");
  for (int i=0; i<12; i++) {
    int nbytes;

    notes[i].frequency = freq[i];
    notes[i].currsample = 0;
    notes[i].nsamples = int(44100.0/notes[i].frequency) + 1;
    nbytes = sizeof(Frame)*notes[i].nsamples;
    totalbytes += nbytes;
    notes[i].frame = (Frame *)malloc(nbytes);
    make_frame(notes[i].frame, notes[i].nsamples, notes[i].frequency);
    sprintf(out, "i %d: F %f, ns %d, frame 0x%x, nbytes %d, totalbytes %d\n", 
                  i, notes[i].frequency, notes[i].nsamples, notes[i].frame, nbytes, totalbytes);
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
  static int prevnote = -1; // prevnote will be completed before the next note is played
  static struct sound *note = NULL;
  
  if (n>=0 && n<12) {

    if (note==NULL) {
      lastnote = n;
      note = &notes[n];
    }
    
    for (f = 0; f < framecount; f++) {
      
      if (note->currsample >= note->nsamples) {  
        // we are at the end of the waveform
        // switch to a new note if necessary
        // set current sample number to 0
        
        if (n!=lastnote) {   // need to switch to different note
          lastnote = n;
          sprintf(out, "new note n=%d, frame_count %d, currsample %d, nsamples %d", n, framecount, note->currsample, note->nsamples);
          Serial.println(out);
          note = &notes[n];
        }
        note->currsample = 0;
      }

      frame[f] = note->frame[note->currsample];
      note->currsample+=8;   // step 8 for octave 3
    }
  }
  else // n is out of range. Do nothing.

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
