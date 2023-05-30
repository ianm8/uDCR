/*
 * uDCR - Uses the Seeed XIAO RP2040
 * Copyright 2023 Ian Mitchell VK7IAN
 * Version 1.0
 * 
 * Uses:
 * https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
 *
 * Libraries
 * =========
 * https://github.com/pavelmc/Si5351mcu
 * https://github.com/brianlow/Rotary
 *
 */

#include <Wire.h>
#include <Rotary.h>
#include <si5351mcu.h>
#include "agc.h"

#define PIN_SDA    D4
#define PIN_SCL    D5
#define PIN_ENCBUT D2
#define PIN_ENCA   D1
#define PIN_ENCB   D0
#define PIN_1LED   D7
#define PIN_AGCIN  A3
#define PIN_AGCOUT D6

#define DEFAULT_VOLUME    128u
#define MIN_VOLUME        90u
#define MAX_VOLUME        255u
#define DEFAULT_FREQUENCY 7041000ul
#define DEFAULT_STEP      1000ul
#define DEFAULT_BAND      BAND_40M
#define VOLUME_STEP       10u
#define LONG_PRESS_TIME   3000u
#define MIN_FREQUENCY     3500000ul
#define MAX_FREQUENCY     15000000ul
#define XTAL_FREQ         27000000ul
#define DEC_PERIOD        200u

// for built in LED, low is on
#define LED_BUILTIN_1 16u
#define LED_BUILTIN_2 17u
#define LED_BUILTIN_3 25u

enum band_t {BAND_80M,BAND_40M,BAND_20M};

volatile static struct
{
  uint32_t frequency;
  uint32_t tuning_step;
  uint32_t volume;
  band_t band;
} radio = {DEFAULT_FREQUENCY,DEFAULT_STEP,DEFAULT_VOLUME,DEFAULT_BAND};

Rotary r = Rotary(PIN_ENCB,PIN_ENCA);
Si5351mcu OSC;

volatile static int32_t sig_level_pwm = 0u;
volatile static bool setup_complete = false;

void setup()
{
  pinMode(LED_BUILTIN_1,OUTPUT);
  pinMode(LED_BUILTIN_2,OUTPUT);
  pinMode(LED_BUILTIN_3,OUTPUT);
  digitalWrite(LED_BUILTIN_1,HIGH);
  digitalWrite(LED_BUILTIN_2,HIGH);
  digitalWrite(LED_BUILTIN_3,HIGH);
#ifdef uDCR_DEBUG
  for (int i=0;i<5;i++)
  {
    delay(500);
    digitalWrite(LED_BUILTIN_1,LOW);
    digitalWrite(LED_BUILTIN_2,LOW);
    digitalWrite(LED_BUILTIN_3,LOW);
    delay(100);
    digitalWrite(LED_BUILTIN_1,HIGH);
    digitalWrite(LED_BUILTIN_2,HIGH);
    digitalWrite(LED_BUILTIN_3,HIGH);
  }
  delay(5000);
#endif
  analogWriteResolution(8);
  analogWriteFreq(500000ul);
  analogWrite(PIN_AGCOUT,0);
  pinMode(PIN_ENCBUT,INPUT_PULLUP);
  pinMode(PIN_ENCA,INPUT_PULLUP);
  pinMode(PIN_ENCB,INPUT_PULLUP);
  pinMode(PIN_1LED,OUTPUT);
  digitalWrite(PIN_1LED,LOW);
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  analogReadResolution(12);
  r.begin();
  OSC.init(XTAL_FREQ);
/*
// SIOUT_2mA
// SIOUT_4mA
// SIOUT_6mA
// SIOUT_8mA
  OSC.setPower(0u,SIOUT_8mA);
*/  
  OSC.setFreq(0u,radio.frequency);
  OSC.enable(0u);
  analogWrite(PIN_AGCOUT,radio.volume);
  setup_complete = true;
}

void setup1()
{
  // wait for setup() to complete
  while (!setup_complete)
  {
    // setup() not done yet
  }
}

void loop()
{
  // for remembering the current frequency and button press
  volatile static uint32_t current_frequency = 0;
  volatile static uint32_t button_start_time = 0;
  volatile static enum
  {
    STATE_TUNING,
    STATE_TUNING_STEP,
    STATE_BUTTON_PRESS,
    STATE_VOLUME,
    STATE_WAIT_RELEASE
  } state = STATE_TUNING;

  // process AGC
  const int32_t peak_sig = AGC::peak((int16_t)analogRead(PIN_AGCIN)-3102l);
  int32_t agc_pwm_temp = radio.volume - AGC::attenuation(peak_sig);
  if (agc_pwm_temp<116)
  {
    // if the AGC takes us below 1.5V then limit this to 1.4V
    agc_pwm_temp = 108;
  }
  // if the volume is still less than the AGC then the volume is the value
  if (radio.volume<agc_pwm_temp)
  {
    agc_pwm_temp = radio.volume;
  }
  const int32_t agc_pwm = constrain(agc_pwm_temp,MIN_VOLUME,MAX_VOLUME);
  sig_level_pwm = peak_sig * 2;
  analogWrite(PIN_AGCOUT,agc_pwm);

  // what's the rotary encoder doing?
  const uint8_t rotary = r.process();
  
  switch (state)
  {
    case STATE_TUNING:
    {
      // tuning
      switch (rotary)
      {
        case DIR_CW:
        {
          radio.frequency += radio.tuning_step;
          radio.frequency -= radio.frequency%radio.tuning_step;
          break;
        } 
        case DIR_CCW:
        {
          radio.frequency -= radio.tuning_step;
          radio.frequency -= radio.frequency%radio.tuning_step;
          break;
        }
      }
      if (digitalRead(PIN_ENCBUT)==LOW)
      {
        button_start_time = millis();
        state = STATE_BUTTON_PRESS;
        delay(50);
      }
      break;
    }
    case STATE_BUTTON_PRESS:
    {
      volatile const uint32_t press_time = millis()-button_start_time;
      if (press_time>LONG_PRESS_TIME)
      {
        // change the band and wait for button release
        switch (radio.band)
        {
          case BAND_80M: radio.band = BAND_40M; radio.frequency = 7041000u;  break;
          case BAND_40M: radio.band = BAND_20M; radio.frequency = 14200000u; break;
          case BAND_20M: radio.band = BAND_80M; radio.frequency = 3600000u;  break;
        }
        state = STATE_WAIT_RELEASE;
        break;
      }
      if (rotary==DIR_CW)
      {
        radio.volume += VOLUME_STEP;
        state = STATE_VOLUME;
      }
      else if (rotary==DIR_CCW)
      {
        radio.volume -= VOLUME_STEP;
        state = STATE_VOLUME;
      }
      else if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        state = STATE_TUNING_STEP;
      }
      break;
    }
    case STATE_TUNING_STEP:
    {
      // change step
      switch (radio.tuning_step)
      {
        case 1000: radio.tuning_step = 100;  break;
        case 100:  radio.tuning_step = 10;   break;
        case 10:   radio.tuning_step = 1000; break;
      }
      state = STATE_WAIT_RELEASE;
      break;
    }
    case STATE_VOLUME:
    {
      switch (rotary)
      {
        case DIR_CW:  radio.volume += VOLUME_STEP; break;
        case DIR_CCW: radio.volume -= VOLUME_STEP; break;
      }
      if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        state = STATE_WAIT_RELEASE;
      }
      break;
    }
    case STATE_WAIT_RELEASE:
    {
      // debounce
      delay(50);
      if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        state = STATE_TUNING;
      }
      break;
    }
  }

  radio.volume = constrain(radio.volume,MIN_VOLUME,MAX_VOLUME);
  radio.frequency = constrain(radio.frequency,MIN_FREQUENCY,MAX_FREQUENCY);
  if (radio.frequency!=current_frequency)
  {
    current_frequency = radio.frequency;
    OSC.setFreq(0u,radio.frequency);
  }
}

void loop1(void)
{
  // display the frequency on the 1LED when it changes
  // the decimal table has the following representation:
  // short flash counts as one
  // long flash counts as five
  // two long flashes represents zero (5 + 5 = 10, drop the carry)
  // so 7 is long, short, short
  // and 4 is short, short, short, short
  static const uint8_t decimal_tab[] =
  {
    0b00000100, // 0
    0b00000011, // 1
    0b00000111, // 2
    0b00001111, // 3
    0b00011111, // 4
    0b00000010, // 5
    0b00000110, // 6
    0b00001110, // 7
    0b00011110, // 8
    0b00111110  // 9
  };
  static uint32_t decimal_frequency = 0;
  const uint32_t f = radio.frequency;
  if (decimal_frequency==f)
  {
    analogWrite(PIN_1LED,sig_level_pwm);
  }
  else
  {
    pinMode(PIN_1LED,OUTPUT);
    digitalWrite(PIN_1LED,LOW);
    delay(1000);
    decimal_frequency = f;
    char dec[16] = "";
    memset(dec,0,sizeof(dec));
    ultoa(decimal_frequency/1000UL,dec,10);
    char *p = dec;
    while (*p)
    {
      const char i = *p++ - 48;
      if (i<0 || i>9) break;
      uint8_t decimal = decimal_tab[i];
      while (decimal>1)
      {
        digitalWrite(PIN_1LED,HIGH);
#ifdef uDCR_DEBUG
        digitalWrite(LED_BUILTIN_1,LOW);
        digitalWrite(LED_BUILTIN_2,LOW);
        digitalWrite(LED_BUILTIN_3,LOW);
#endif
        if (decimal&1)
        {
          delay(DEC_PERIOD);
        }
        else
        {
          delay(DEC_PERIOD*3);
        }
        digitalWrite(PIN_1LED,LOW);
#ifdef uDCR_DEBUG
        digitalWrite(LED_BUILTIN_1,HIGH);
        digitalWrite(LED_BUILTIN_2,HIGH);
        digitalWrite(LED_BUILTIN_3,HIGH);
#endif
        delay(DEC_PERIOD);
        decimal >>= 1;
      }
      delay(DEC_PERIOD*5);
    }
  }
}
