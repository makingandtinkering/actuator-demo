/**
 * Library dependencies:
 *  - u8glib
 *  - SimpleRotary
 */
#include <Arduino.h>
#include <U8glib.h>
#include <SimpleRotary.h>

U8GLIB_ST7920_128X64_4X u8g(23, 17, 16);
SimpleRotary rotary(31, 33, 35);

void setup() {
  u8g.setFont(u8g_font_profont11);
}

typedef enum {
  STATE_SERVO = 0,
  STATE_STEPPER_GRANULARITY,
  STATE_STEPPER_STEPS,
  STATE_PWM,
  STATE_MAX
} state_t;

state_t state = STATE_SERVO;

struct {
  uint8_t delta = 32;
  uint16_t pulse_min = 1000;
  uint16_t pulse_max = 2000;

  uint16_t pulse_width = 1500;
} state_servo_data;

struct {
  uint8_t delta_coarse = 32;
  uint8_t delta_fine = 1;
  bool coarse = true;
  int32_t steps = 0;
} state_stepper_data;

struct {
  uint8_t delta = 10;
  int8_t duty_min = 0;
  int8_t duty_max = 100;
  uint8_t duty_cycle = 0;
} state_pwm_data;

void loop() {
  constexpr uint8_t LEN_BUF = 32;
  char buf[LEN_BUF];

  static bool redraw_lcd = true;
  static uint32_t last_lcd_redraw = millis();

  if (redraw_lcd && (millis() - last_lcd_redraw) > 200) {
    last_lcd_redraw = millis();
    u8g.firstPage();  
    do {
      snprintf(buf, LEN_BUF, "%cServo: Pulse=%d", state == STATE_SERVO ? '>':' ', state_servo_data.pulse_width);
      u8g.drawStr(0, 8, buf);

      snprintf(buf, LEN_BUF, "%cStepper Delta: %d",
        state == STATE_STEPPER_GRANULARITY ? '>':' ',
        state_stepper_data.coarse ? state_stepper_data.delta_coarse : state_stepper_data.delta_fine
      );
      u8g.drawStr(0, 20, buf);
      
      snprintf(buf, LEN_BUF, "%cStepper Steps=%d", state == STATE_STEPPER_STEPS ? '>':' ', state_stepper_data.steps);
      u8g.drawStr(0, 28, buf);

      snprintf(buf, LEN_BUF, "%cPWM Duty=%d%", state == STATE_PWM ? '>':' ', state_pwm_data.duty_cycle);
      u8g.drawStr(0, 44, buf);
    } while( u8g.nextPage() );
  }

  if (rotary.push()) {
    state = (uint8_t) state + 1;
    if (state >= STATE_MAX) {
      state = (state_t) 0;
    }

    redraw_lcd = true;
  }

  uint8_t rotate = rotary.rotate();
  if (rotate) {
    switch(state) {
      case STATE_SERVO: {
        if (rotate == 1) {
          state_servo_data.pulse_width = min(state_servo_data.pulse_width + state_servo_data.delta, state_servo_data.pulse_max);
        } else if (rotate == 2) {
          state_servo_data.pulse_width = max(state_servo_data.pulse_width - state_servo_data.delta, state_servo_data.pulse_min);
        }
    
        redraw_lcd = true;
        break;
      }
      case STATE_STEPPER_GRANULARITY: {
        state_stepper_data.coarse = !state_stepper_data.coarse;

        redraw_lcd = true;
        break;
      }
      case STATE_STEPPER_STEPS: {
        uint8_t delta = state_stepper_data.coarse ? state_stepper_data.delta_coarse : state_stepper_data.delta_fine;
        if (rotate == 1) {
          state_stepper_data.steps += delta;
        } else if (rotate == 2) {
          state_stepper_data.steps -= delta;
        }

        redraw_lcd = true;
        break;
      }
      case STATE_PWM: {
        if (rotate == 1) {
          state_pwm_data.duty_cycle = min(state_pwm_data.duty_cycle + state_pwm_data.delta, state_pwm_data.duty_max);
        } else if (rotate == 2) {
          state_pwm_data.duty_cycle = max(state_pwm_data.duty_cycle - state_pwm_data.delta, state_pwm_data.duty_min);
        }
        
        redraw_lcd = true;
        break;
      }
    }
  }
  
}
