/// @reference: https://forum.arduino.cc/t/pwm-channel-in-esp32/695510/2
///             https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html


#include <HardwareSerial.h>
#include "driver/mcpwm.h"
// #include "ESP32Servo.h"

const int pwm_channel = 0;
const int pwm_freq = 4096;
const int pwm_resolution = 12;

// const int max_duty_cycle = (int)(pow(2, PWM_RESOLUTION) - 1);

const int max_duty_cycle = 100;
int current_duty_cycle = 0;
const int signal_pin = 17;

const int delay_ms = 100;

void fMoveActuonix( int MoveTo )
{
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MoveTo);
  vTaskDelay( 12 );
}

void setup()
{
  Serial.begin(115200);

  /// @note: This is the "LED PWM" way of controlling the linear actuators but there is also a PWM library 
  ///        specifically for motors.
  pinMode(signal_pin, OUTPUT);
  // Configure LED PWM functionalities:
  ledcSetup(pwm_channel, pwm_freq, pwm_resolution);
  // Attach channel to the GPIO pin to be controlled
  ledcAttachPin(signal_pin, pwm_channel); 

  // analogWrite(signal_pin, 255);

  // /// @note: Making use of the MCPWM (Motor control PWM) library:
  // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_NUM_4 ); // Azimuth
  // mcpwm_config_t pwm_config = {};
  // pwm_config.frequency = 1000;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
  // pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
  // pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
  // pwm_config.counter_mode = MCPWM_UP_COUNTER;
  // pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  // log_i( "MCPWM complete" );

  // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A timer 0 with above settings

  // fMoveActuonix(500);

}

void loop()
{    

  ledcWrite(pwm_channel, 400);                     //approx 2mS
  delay(2000);    

  // while (current_duty_cycle <= 50)
  // {
  //   ledcWrite(pwm_channel, current_duty_cycle);
  //   // analogWrite(signal_pin, 
  //   current_duty_cycle += 2;
  //   delay(100);
  //   Serial.println(current_duty_cycle);
  // }
  // while (current_duty_cycle != 0)
  // {
  //   ledcWrite(pwm_channel, current_duty_cycle);
  //   current_duty_cycle -= 2;
  //   delay(100);
  //   Serial.println(current_duty_cycle);

    
  // }
  
}

