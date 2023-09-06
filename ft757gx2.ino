//////////////////////////////////
// FT-757 GX2 control program
//////////////////////////////////

/* Based on VK2TRP work for FT-757 GX v1
   The main differences between these two models are
   - Serial programming PLL instead of parallel bus
   - Modulation modes are managed by the CPU instead of the manual switch

   Please note that this is an early version, it lacks of some features.
*/

#include <EEPROM.h>
#include <avr/wdt.h>

#define dial_clk_div 0
#define q64_2 1
#define q64_1 2
#define q64_0 3
#define q64_port PORTB
#define q64_mask 14 // To access only to the PIN 1, 2, 3
#define disp_irq 4
#define disp_bit0 5
#define disp_bit1 6
#define disp_bit2 7
#define disp_bit3 10
#define disp_port PORTB
#define disp_mask 224 // PB5 PB6 PB7
#define disp_PB0 11
#define disp_PB1 12
#define disp_PB2 13
#define hg_btn 14
#define ptt_in 15 // Down when TX
#define scan_stop 16
#define fast_btn 17
#define scan_mode_btn 18
#define count_direction 19
#define mic_down_btn 20
#define mic_up_btn 21
#define dial_clk 22
#define data_3 23
#define data_2 31
#define data_1 30
#define data_0 29
#define data_port PORTA
#define data_mask 224 // Bits 5/6/7
#define pll_data 28
#define pll_clk 27
#define agc_comp 26


byte disp_pins[]={disp_bit3, disp_bit2, disp_bit1, disp_bit0}; // Array of display pins
byte data_pins[]={data_0, data_1, data_2, data_3}; // Data pins on bus
//byte q64_pins[]={q64_0, q64_1, q64_2};

// Strobes values
#define Q64_PLL2 1 // Q1 to PLL2 Q42
#define Q64_PLL1 2 // Q2 to PLL1 Q31
#define Q64_MODE 3 // Q3 to Q65 latch
#define Q64_COUNTER_PE 4// Q4 to Q66 counter
#define Q64_BAND 5 // Q5 to Q69 latch
#define Q64_BUZZER 6 // Q6 to buzzer


enum selected_vfo { VFO_A, VFO_B };
enum modes { MODE_LSB, MODE_USB, MODE_CW_W, MODE_CW_N, MODE_AM, MODE_FM }; 
enum keys { KEY_NONE, KEY_VFO_A_B, KEY_SPLIT, KEY_MR_VFO, KEY_VFO_M, KEY_VFO_TO_M, KEY_M_TO_VFO, KEY_CLAR, KEY_D_LOCK, KEY_DOWN, KEY_UP, KEY_HG, KEY_MODE };
uint32_t ham_freqs[] = {100000, 1500000, 3500000, 7000000, 10000000, 14000000, 18000000, 21000000, 24500000, 28000000};

// Defines the frequencies limits
#define MIN_FREQ 150000
#define MAX_FREQ 30000000

byte last_key = KEY_NONE; // To debounce keys
uint32_t disp_freq; // To refresh display only when significant change
uint32_t save_delay = 0;
bool tx_mode = false;

#define CONF_BASE 0
#define CONF_VERSION 1
// A structure describing the transceiver config
// We fill with default values
struct mainConfig {
  uint8_t confver = CONF_VERSION; // Config version, to avoid issues
  bool mem_mode = false; // By default we are in vfo mode
  uint8_t current_mem = 0; // Current memory
  bool vfo = VFO_A;
  uint32_t freq[2] = {7000000, 7000000};
  uint32_t clar_freq = 7000000;
  uint8_t mode = MODE_LSB;
  uint8_t band = 3;
  bool dl = false; // Dial lock
  bool gen = false; // we are in ham mode
  bool clar = false;
  bool split = false;
};

struct memEntry {
  uint32_t freq = 7000000;
  uint8_t mode = MODE_LSB;
};

mainConfig rig;
memEntry memories[10]; // memories array

void pin_setup() {
  
  pinMode (dial_clk_div, INPUT);
  pinMode (q64_2, OUTPUT);
  pinMode (q64_1, OUTPUT);
  pinMode (q64_0, OUTPUT);
  pinMode (disp_irq, OUTPUT);
  pinMode (disp_bit0, OUTPUT);
  pinMode (disp_bit1, OUTPUT);
  pinMode (disp_bit2, OUTPUT);
  pinMode (disp_bit3, OUTPUT);
  pinMode (disp_PB0, INPUT);
  pinMode (disp_PB1, INPUT);
  pinMode (disp_PB2, INPUT);
  pinMode (hg_btn, INPUT);
  pinMode (ptt_in, INPUT);
  pinMode (scan_stop, INPUT);
  pinMode (fast_btn, INPUT);
  pinMode (scan_mode_btn, INPUT);
  pinMode (count_direction, INPUT);
  pinMode (mic_down_btn, INPUT);
  pinMode (mic_up_btn, INPUT);
  pinMode (dial_clk, INPUT);
  pinMode (data_3, OUTPUT);
  pinMode (data_2, OUTPUT);
  pinMode (data_1, OUTPUT);
  pinMode (data_0, OUTPUT);
  pinMode (pll_data, OUTPUT);
  pinMode (pll_clk, OUTPUT);
  pinMode (agc_comp, INPUT);

  Serial.println("I/O ports configuration done");

}

void reboot() {
  wdt_enable(WDTO_15MS); // activer le watchdog
  while (1) {};          // et attendre ...
}

// Hardware related functions

void write_q64(byte data) {
  byte tmp = 0;
  bitWrite(tmp, 3, bitRead(data, 0)); bitWrite(tmp, 2, bitRead(data, 1)); bitWrite(tmp, 1, bitRead(data, 2)); // Reverse and position the word
  q64_port = q64_port & !q64_mask; // We clear the q64 bits
  q64_port = q64_port | ( tmp & q64_mask); // We write the word
}

void reset_q64() {
  if(rig.mem_mode || rig.dl) {
    q64_port = q64_port | q64_mask;
  } else {
    q64_port = q64_port & !q64_mask;
  }
}

void clear_disp_bits() {
    disp_port = disp_port & !disp_mask;
    digitalWrite(disp_bit3, LOW);
}

void display_send(byte val) {
  clear_disp_bits();

  byte tmp = 0;
  bitWrite(tmp, 5, bitRead(val,3));
  bitWrite(tmp, 6, bitRead(val,2));
  bitWrite(tmp, 7, bitRead(val,1));

  disp_port = disp_port | tmp;
  digitalWrite(disp_bit3, bitRead(val, 0));

  digitalWrite(disp_irq, HIGH);
  delayMicroseconds(50);
  digitalWrite(disp_irq, LOW);
  delay(1);

  clear_disp_bits();
}

void write_data(byte val) {
  byte tmp = 0;
  bitWrite(tmp, 5, bitRead(val,0));
  bitWrite(tmp, 6, bitRead(val,1));
  bitWrite(tmp, 7, bitRead(val,2));

  data_port = data_port & !data_mask;
  data_port = data_port | tmp;
  digitalWrite(data_3, bitRead(val, 3));
}


void do_key(byte key) {
  bool need_update = false;
  uint32_t t_freq;
  uint8_t t_mode;
  buzzer();
  switch (key) {
    case KEY_MODE:
      if(rig.mem_mode) break;
      rig.mode++;
      if (rig.mode>5) rig.mode=0;
      set_mode(rig.mode);
      //update_pll();
      autosave();
      break;
    case KEY_VFO_A_B:
      rig.vfo = !rig.vfo;
      need_update = true;
      break;
    case KEY_MR_VFO:
      rig.mem_mode = !rig.mem_mode;
      need_update = true;
      break;
    case KEY_VFO_M: // current mem & current vfo contents exchange
      t_freq = memories[rig.current_mem].freq;
      t_mode = memories[rig.current_mem].mode;
      memories[rig.current_mem].freq = rig.freq[rig.vfo];
      memories[rig.current_mem].mode = rig.mode;
      rig.freq[rig.vfo] = t_freq;
      rig.mode = t_mode;
      need_update = true;
      break;
    case KEY_UP:
      if (rig.mem_mode) {
        rig.current_mem++;
        if (rig.current_mem > 9) rig.current_mem = 0;
      } else {
        if (rig.gen) {
          rig.freq[rig.vfo] += 500000;
          if (rig.freq[rig.vfo] > MAX_FREQ)
            rig.freq[rig.vfo] = MIN_FREQ;
        } else {
          if (rig.band == 9) {
            rig.freq[rig.vfo] = ham_freqs[1];
          } else {
            rig.freq[rig.vfo] = ham_freqs[rig.band+1];
          }     
        }
      }
      need_update = true;
      break;
    case KEY_DOWN:
      if (rig.mem_mode) {
        if (rig.current_mem == 0) rig.current_mem = 9;
        else rig.current_mem--;
      } else {
        if (rig.gen) {
          if ((rig.freq[rig.vfo] - MIN_FREQ ) < 500000) {
            rig.freq[rig.vfo] = MAX_FREQ;
          } else {
            rig.freq[rig.vfo] -= 500000;
          }
        } else {
          if (rig.band == 1) {
            rig.freq[rig.vfo] = ham_freqs[9];
          } else {
            rig.freq[rig.vfo] = ham_freqs[rig.band-1];
          }   
        }
      }
      need_update = true;
      break;
    case KEY_VFO_TO_M:
      if(rig.mem_mode) break;
      memories[rig.current_mem].freq = rig.freq[rig.vfo];
      memories[rig.current_mem].mode = rig.mode;
      save_config();
      break;
    case KEY_M_TO_VFO:
      rig.freq[rig.vfo] = memories[rig.current_mem].freq;
      rig.clar_freq = memories[rig.current_mem].freq;
      rig.mode = memories[rig.current_mem].mode;
      if(!rig.mem_mode) need_update = true;
      break;
    case KEY_CLAR:
      if (rig.mem_mode || tx_mode ) break;
      if (rig.clar) {
        //rig.freq[rig.vfo] = rig.clar_freq;
        rig.clar = false;
        update_display();
        update_pll();
      } else {
     //   rig.clar_freq = rig.freq[rig.vfo];
        rig.clar = true;
        update_display();
        update_pll();
      }
      save_config();
      break;
    case KEY_D_LOCK: // Dial lock support
      rig.dl = !rig.dl;
      reset_q64();
      update_display();
      save_config();
      break;
    case KEY_SPLIT:
      if(!rig.mem_mode && !tx_mode) {
        rig.split = !rig.split;
        update_display();
        save_config();
      }
      break;
  }

  if (need_update) {
      update_display();
      update_mode();
      update_pll();
      save_config();
  }
}

void key_scan(){

  byte key = KEY_NONE;

  for (byte i=0; i<4; i++) {
    digitalWrite(disp_pins[i], HIGH);
    if (digitalRead(disp_PB0)==HIGH) {
      switch(i) {
        case 0:
          key = KEY_SPLIT;
          break;
        case 1:
          key = KEY_D_LOCK;
          break;
        case 2:
          key = KEY_UP;
          break;
      }
    }
    if (digitalRead(disp_PB1)==HIGH) {
      switch(i) {
        case 0:
          key = KEY_MR_VFO;
          break;
        case 1:
          key = KEY_VFO_A_B;
          break;
        case 2:
          key = KEY_DOWN;
          break;
        case 3:
          key = KEY_VFO_M;
          break;
      }
    }
    if (digitalRead(disp_PB2)==HIGH) {
      switch(i) {
        case 0:
          key = KEY_VFO_TO_M;
          break;
        case 1:
          key = KEY_M_TO_VFO;
          break;
        case 2:
          key = KEY_CLAR;
          break;
        case 3:
          key = KEY_MODE;
          break;
      }
    }
    digitalWrite(disp_pins[i], LOW);
  }

  // Now we check if a key is pressed
  if (key != KEY_NONE) {
    if (key != last_key) {
      do_key(key);
      last_key = key;
    }
  } else {
    if (last_key != KEY_NONE) last_key = KEY_NONE;
  }
  
}




void strobe_q64(byte val) {
  write_q64(val);

  delayMicroseconds(10);

  reset_q64();
}

void buzzer() {
 /* for (byte i=0; i<3; i++) {
    byte state = bitRead(Q64_BUZZER, i);
    digitalWrite(q64_pins[i], state);
  } */

  write_q64(Q64_BUZZER);

  delay(100);

  reset_q64();

}

void calculate_band(uint32_t freq) {
  uint8_t band = 10;
  if (freq < 1500000)
    band = 0;
  if (freq >= 1500000 && freq < 2500000)
    band = 1;
  if (freq >= 2500000 && freq < 4000000)
    band = 2;
  if (freq >= 4000000 && freq < 7500000)
    band = 3;
  if (freq >= 7500000 && freq < 10500000)
    band = 4;
  if (freq >= 10500000 && freq < 14500000)
    band = 5;
  if (freq >= 14500000 && freq < 18500000)
    band = 6;
  if (freq >= 18500000 && freq < 21500000)
    band = 7;
  if (freq >= 21500000 && freq < 25000000)
    band = 8;
  if (freq >= 25000000)
    band = 9;

  if (rig.band != band) {
    rig.band = band;
    set_band(rig.band);
  }
}

void update_mode() {
  if (rig.mem_mode) {
    set_mode(memories[rig.current_mem].mode);
  } else {
    set_mode(rig.mode);
  }
}

void set_mode(byte mode) {
  write_data(mode);
  strobe_q64(Q64_MODE);
  write_data(0);
}

void set_band(byte band) {
  write_data(band);
  strobe_q64(Q64_BAND);
  write_data(0);
}

void set_100hz(uint8_t x) {
    write_data(x);
}

void set_10hz(uint8_t x) {
  write_data(x);
  strobe_q64(Q64_COUNTER_PE);
  write_data(0);
}

/* 
 * Display related functions
 * 
  */
 


void update_display() {

  int tmpe;
  
  display_send(10);
  display_send(rig.gen); // 1 to activate gen LED
  display_send(15);
  display_send(15);

  if (rig.mem_mode) {
    display_send(2); // MR
    disp_freq = memories[rig.current_mem].freq;
  } else {
    if (rig.vfo == VFO_A) {
      display_send(1); // vfo a
    } else {
      display_send(12); // vfo b
    }
  }

  if (!rig.mem_mode) {
    if (rig.clar && !tx_mode) {
      disp_freq = rig.clar_freq;
    } else {
      disp_freq = rig.freq[rig.vfo];
    }
  }
  
  if ((rig.dl || rig.clar || rig.split) && !rig.mem_mode) {
    uint8_t tmp = 0;
    if (rig.dl)
      tmp = 1;
    if (rig.clar)
      tmp += 2;
    if (rig.split)
      tmp += 12;
    if (rig.dl && rig.split)
      tmp = 7;
    if (rig.dl && rig.clar && rig.split)
      tmp = 4;
    display_send(tmp);
  } else {
    display_send(15); // no clar / split / dl
  }
  
  char cstr[7];
  sprintf(cstr, "%06lu", disp_freq/100);

  bool first = true;
  for (uint8_t i=0; i<6; i++) {
     uint8_t digit = String(cstr[i]).toInt();
     if (digit == 0 && first) {
      display_send(15);
      first = false;
     } else {
      display_send(digit);
      first = false;
     }
  }

 if (rig.mem_mode) {
  display_send(rig.current_mem);
 } else {
  display_send(15);
 }

}

void send_to_pll(bool pll, int tosend, bool control) {
  for (int i=13; i>=0; i--) {
    byte state = bitRead(tosend, i);
    digitalWrite(pll_data, state);
    digitalWrite(pll_clk, HIGH);
 //   delayMicroseconds(10);
    digitalWrite(pll_clk, LOW);
  }
  digitalWrite(pll_data, control);
  digitalWrite(pll_clk, HIGH);
//  delayMicroseconds(10);
  digitalWrite(pll_clk, LOW);
  digitalWrite(pll_data, LOW);

  if(!pll) {
    strobe_q64(Q64_PLL1);
  } else {
    strobe_q64(Q64_PLL2);
  }
}


// 1st lo runs 47.560 (500khz) to 77.060 (30.000Mhz) in 1khz steps
// pll1 runs n code /560 to /1059 to provide 1khz steps up to 499khz
// pll2 runs n codes 13 to 40 (0.5 -> 14) and 11 to 42 (14.5 -> 30) steping in 500khz steps with
// 2 different loop mixer frequency ranges
void update_pll() {   // PLL part

  uint32_t freq;

  if (rig.mem_mode) {
    freq = memories[rig.current_mem].freq;
  } else {
    if (rig.clar && !tx_mode) {
      freq = rig.clar_freq;
    } else {
      freq = rig.freq[rig.vfo];
    }
  }

  calculate_band(freq);
  
  unsigned int tmpb, tmpe;

  uint32_t cmhz = freq / 1000000;

  uint16_t c100k = (freq-(cmhz*1000000))/1000;
  uint8_t c100 = (freq-((freq/1000)*1000))/100;
  uint8_t c10 = (freq-((freq/100)*100))/10;
 // Serial.println(c100k);
 // Serial.println(c10);
  if (c100k > 499)     // where are we in 500khz? for pll1
    {tmpb = c100k - 500;}
  else
    {tmpb = c100k;}
  tmpe = tmpb + 560;
//  Serial.println(tmpe);
  //PLL1
  send_to_pll(0, tmpe, false);

  //PLL2
  if (rig.band > 5) {
    tmpe = 10 + (2*(cmhz-14));
  } else {
    tmpe = 12 + (2*cmhz);
  }
  if (c100k > 499)
     {tmpe = tmpe + 1;}        // up 500khz  removed tmpc == 5
 // Serial.println(tmpe);
  send_to_pll(1, tmpe, false);
 // Serial.println(tmpe);
 set_10hz(c10);
 set_100hz(c100);
  
}

void set_pll_reference() {
  send_to_pll(0, 1500, true); // Reference divider latch
  send_to_pll(1, 30, true);
}

uint16_t calc_inc_step() { // To be improved...
  uint16_t inc = 10;
  if (digitalRead(fast_btn) == LOW) {
    inc = 1000; // 1KHz
  }
  return inc;
}

void increment_vfo(uint16_t inc_step) {
  
  rig.freq[rig.vfo] += inc_step; // Increment freq
  if (rig.freq[rig.vfo] > MAX_FREQ) {
    rig.freq[rig.vfo] = MIN_FREQ;
  }
   
  // We update the display only if necessary
  if ((rig.freq[rig.vfo]/100) != (disp_freq/100)) {
    update_display();
  }
  if(!rig.clar) {
    rig.clar_freq = rig.freq[rig.vfo];
  }
  update_pll();
  autosave();
}

void decrement_vfo(uint16_t dec_step) {

  rig.freq[rig.vfo] -= dec_step; //decrement freq
  if (rig.freq[rig.vfo] < MIN_FREQ) {
    rig.freq[rig.vfo] = MAX_FREQ;
  }

  // We update the display only if necessary
  if ((rig.freq[rig.vfo]/100) != (disp_freq/100)) {
    update_display();
  }
  if(!rig.clar) {
    rig.clar_freq = rig.freq[rig.vfo];
  }    
  update_pll();
  autosave();
}

void increment_clarifier(uint16_t inc_step) {
    rig.clar_freq += inc_step; // Increment freq
  if (rig.clar_freq > MAX_FREQ) {
    rig.clar_freq = MIN_FREQ;
  }
  
  if ((rig.clar_freq/100) != (disp_freq/100)) {
    update_display();
  }
  
  update_pll();
  autosave();
}

void decrement_clarifier(uint16_t dec_step) {
  rig.clar_freq -= dec_step; //decrement freq
  if (rig.clar_freq < MIN_FREQ) {
    rig.clar_freq = MAX_FREQ;
  }
  if ((rig.clar_freq/100) != (disp_freq/100)) {
    update_display();
  }
    
  update_pll();
  autosave();
}

void autosave() {
  save_delay = millis() + 1000; // 1 second since the last VFO change
}

void read_config() {
    EEPROM.get(CONF_BASE, rig);
    EEPROM.get(CONF_BASE + sizeof(rig), memories);
}

void save_config() {
  EEPROM.put(CONF_BASE, rig);
  EEPROM.put(CONF_BASE + sizeof(rig), memories);
  save_delay = 0; // Cancel the planned conf save
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200); //Debug port
  Serial.println("FT-757 GX2 Revival, v0.1");

  // Setup pin modes
  pin_setup();
  digitalWrite(disp_irq, LOW);

  // FACTORY RESET
  if (digitalRead(hg_btn) == LOW) { // We do a factory reset of the config
   EEPROM.put(CONF_BASE, 255);
   buzzer();
   buzzer();
   buzzer();
   while (digitalRead(hg_btn) == LOW); // Wait for button release
  }

  // We try to read the configuration from eeprom
  if (EEPROM.read(CONF_BASE) == CONF_VERSION) { // If the eeprom has a valid config
    Serial.println("Config valid, reading it");
    read_config();
  } else {
    Serial.println("No valid config, creating new");
    save_config();
  }
  update_display(); // Display the last freq
  set_pll_reference();
  update_pll(); // To apply the reference setting
  set_band(rig.band);
  set_mode(rig.mode); // Set the mode
}

void loop() {
  // put your main code here, to run repeatedly:

  if(digitalRead(hg_btn) == LOW) {
    if (!rig.mem_mode) {
      buzzer();
      rig.gen = !rig.gen;
      update_display();
      save_config();
      delay(100);
      while(digitalRead(hg_btn) == LOW);
    } else {
      uint32_t tmp = millis();
      while (digitalRead(hg_btn) == LOW) { // Wait for button release
        if ((millis() - tmp) > 3000) { // If this is a long push, reboot
          buzzer();
          delay(3000);
          reboot();
        } else { // Short push
          // Do Nothing at this time, reserve for future func ?
        }
      }
    }
    
  }

  if(digitalRead(disp_PB2)==HIGH) { // Display sends a pulse to do keyscan
    while (digitalRead(disp_PB2)); // Wait for pulse ends
    key_scan();
  }

  if(digitalRead(dial_clk)==LOW) { // If the dial button is emetting a pulse
    uint16_t inc = calc_inc_step();
    if(digitalRead(count_direction)==HIGH) { // If we count up
      if(rig.clar) {
        increment_clarifier(inc);
      } else {
        increment_vfo(inc);
      }
    } else {
      if(rig.clar) {
        decrement_clarifier(inc);
      } else {
        decrement_vfo(inc);
      }
    }
    while(!digitalRead(dial_clk));
  }

  if(digitalRead(mic_up_btn) == LOW) {
    buzzer();
    if(rig.mem_mode) return;
    uint16_t inc = calc_inc_step();
    if(rig.clar) {
      increment_clarifier(inc);
    } else {
      increment_vfo(inc);
    }
    delay(50);
    while (digitalRead(mic_up_btn) == LOW);
  }

  if(digitalRead(mic_down_btn) == LOW) {
    buzzer();
    if(rig.mem_mode) return;
    uint16_t inc = calc_inc_step();
    if(rig.clar) {
      decrement_clarifier(inc);
    } else {
      decrement_vfo(inc);
    }
    delay(50);
    while (digitalRead(mic_up_btn) == LOW);
  }

  if (save_delay!=0 && save_delay < millis()) { // It's time to save config
    save_config();
  }

  // Basic clarifier support

  if(digitalRead(ptt_in) == LOW && tx_mode == false) {
    tx_mode = true;
    if(!rig.mem_mode) {
      if (rig.split) {
        rig.vfo = !rig.vfo;
      }
      if (rig.clar || rig.split) {
          update_pll();
          update_display();
      }
    }
  }

  if(digitalRead(ptt_in) == HIGH && tx_mode == true) {
    tx_mode = false;
    if(!rig.mem_mode) {
      if (rig.split) {
        rig.vfo = !rig.vfo;
      }
      if (rig.clar || rig.split ) {
        update_pll();
        update_display();
      }
    }
  }

}
