#include "pins_arduino.h"
#include "AngularSpeedSupervisor.h"
#include "US_Sensor.h"
#include <avr/wdt.h>

#define GET_SENSOR_5 0xff
#define GET_SENSOR_4 0xfe
#define GET_SENSOR_3 0xfd
#define GET_SENSOR_2 0xfc
#define GET_SENSOR_1 0xfb

#define GET_TICKS_L 0xfa
#define GET_TICKS_R 0xf9

#define SET_ANGULAR_VELOCITY_L 0xf8
#define SET_ANGULAR_VELOCITY_R 0xf7

#define SET_OBJECTS 0xf6

#define SET_K_P_CONST 0xf5
#define SET_K_I_CONST 0xf4
#define SET_K_D_CONST 0xf3

#define RESET_ARDUINO 0xf2

#define TRIG_PIN_0 22
#define ECHO_PIN_0 23
#define TRIG_PIN_1 24
#define ECHO_PIN_1 25
#define TRIG_PIN_2 26
#define ECHO_PIN_2 27
#define TRIG_PIN_3 28
#define ECHO_PIN_3 29
#define TRIG_PIN_4 30
#define ECHO_PIN_4 31

#define SIZE 4

long buf[SIZE];
volatile short pos = 0;
volatile byte command;
volatile bool processSpiData;



double k_p_angular_speed = 2489.0;
double k_i_angular_speed = 18.5;
double k_d_angular_speed = 9.14;


unsigned int cpr = 930;
int lower_pwm = 80;
int max_rpm = 130;
/**/
unsigned int cpr_left = 980;
unsigned int cpr_right = 984;
int lower_pwm_left = 80;
int lower_pwm_right = 50;
int max_rpm_left = 160;
int max_rpm_right = 165;

Encoder* enc_left = new Encoder(2, 3, cpr_left);
Encoder* enc_right = new Encoder(18, 19, cpr_right);
Motor* motor_left = new Motor(10, 11, lower_pwm_left, max_rpm_left);
Motor* motor_right = new Motor(9, 8, lower_pwm_right, max_rpm_right);
PIDController* pid_angular_speed_left = new PIDController(k_p_angular_speed, k_i_angular_speed, k_d_angular_speed);
PIDController* pid_angular_speed_right = new PIDController(k_p_angular_speed, k_i_angular_speed, k_d_angular_speed);

/*Encoder* enc_left = new Encoder(2, 3, cpr);
Encoder* enc_right = new Encoder(18, 19, cpr);
Motor* motor_left = new Motor(10, 11, lower_pwm, max_rpm);
Motor* motor_right = new Motor(9, 8, lower_pwm, max_rpm);
PIDController* pid_angular_speed_left = new PIDController(k_p_angular_speed, k_i_angular_speed, k_d_angular_speed);
PIDController* pid_angular_speed_right = new PIDController(k_p_angular_speed, k_i_angular_speed, k_d_angular_speed);*/

Sensor* sensor_0 = new Sensor(TRIG_PIN_0, ECHO_PIN_0);
Sensor* sensor_1 = new Sensor(TRIG_PIN_1, ECHO_PIN_1);
Sensor* sensor_2 = new Sensor(TRIG_PIN_2, ECHO_PIN_2);
Sensor* sensor_3 = new Sensor(TRIG_PIN_3, ECHO_PIN_3);
Sensor* sensor_4 = new Sensor(TRIG_PIN_4, ECHO_PIN_4);

AngularSpeedSupervisor* angular_speed_supervisor_left = new AngularSpeedSupervisor(enc_left, motor_left, pid_angular_speed_left);
AngularSpeedSupervisor* angular_speed_supervisor_right = new AngularSpeedSupervisor(enc_right, motor_right, pid_angular_speed_right);

void softwareReset(uint8_t prescaller) 
{
  // start watchdog with the provided prescaller
  wdt_enable(prescaller);
  // wait for the prescaller time to expire
  // without sending the reset signal by using
  // the wdt_reset() method
  while(1){}
}

void enc_left_A()
{
  enc_left->A_interrupt();
}

void enc_left_B()
{
  enc_left->B_interrupt();
}

void enc_right_A()
{
  enc_right->A_interrupt();
}

void enc_right_B()
{
  enc_right->B_interrupt();
}

long tmp_left = 0;
long tmp_right = 0;

unsigned long last_time = 0L;
unsigned long time = 0L;
double dt_left = 0.0;
double dt_right = 0.0;

void watch_time()
{
  last_time = time;
  time = millis();
  dt_left = (time - last_time) / 1000.0;
  //dt_right = dt_left + 0.012;
  if(dt_left == 0.0)
  {
    dt_left = 0.001;
    //dt_right = dt_left + 0.012;
    delay(1);
  }
}

ISR(SPI_STC_vect)
{
  byte c = SPDR;
  if(command == GET_TICKS_L || command == GET_TICKS_R || command == GET_SENSOR_5 || command == GET_SENSOR_4 || command == GET_SENSOR_3 || command == GET_SENSOR_2 || command == GET_SENSOR_1)
  {
    SPDR = buf[pos++];
    if(pos == SIZE)
    {
      //Serial.println("Saljem na RPi");
      pos = 0;
      command = 0;
    }
  }
  else if(command == SET_ANGULAR_VELOCITY_L || command == SET_ANGULAR_VELOCITY_R)
  {
      buf[pos++] = c;
      if(pos == SIZE)
      {
        long value = buf[3] | (buf[2] | (buf[1] | (buf[0]) << 8) << 8) << 8;
        set_angular_velocity(command, value);
        pos = 0;
        command = 0;
      }
  }
  else if(command == SET_OBJECTS)
  {
    Serial.print("Inicijaliziram objekte...  ");
    pid_angular_speed_left = new PIDController(k_p_angular_speed, k_i_angular_speed, k_d_angular_speed);
    /*Serial.print("left wheel: ");
    Serial.print(pid_angular_speed_left->k_p);
    Serial.print(" ");
    Serial.print(pid_angular_speed_left->k_i);
    Serial.print(" ");
    Serial.println(pid_angular_speed_left->k_d);*/
    pid_angular_speed_right = new PIDController(k_p_angular_speed, k_i_angular_speed, k_d_angular_speed);
    /*Serial.print("right wheel: ");
    Serial.print(pid_angular_speed_right->k_p);
    Serial.print(" ");
    Serial.print(pid_angular_speed_right->k_i);
    Serial.print(" ");
    Serial.println(pid_angular_speed_right->k_d);*/
    angular_speed_supervisor_left = new AngularSpeedSupervisor(enc_left, motor_left, pid_angular_speed_left);
    /*Serial.print("left wheel supervisor: ");
    Serial.print(angular_speed_supervisor_left->pid->k_p);
    Serial.print(" ");
    Serial.print(angular_speed_supervisor_left->enc->cpr);
    Serial.print(" ");
    Serial.println(angular_speed_supervisor_left->motor->lower_pwm);*/
    angular_speed_supervisor_right = new AngularSpeedSupervisor(enc_right, motor_right, pid_angular_speed_right);
    /*Serial.print("right wheel supervisor: ");
    Serial.print(angular_speed_supervisor_right->pid->k_p);
    Serial.print(" ");
    Serial.print(angular_speed_supervisor_right->enc->cpr);
    Serial.print(" ");
    Serial.println(angular_speed_supervisor_right->motor->lower_pwm);*/
    Serial.println("Završio!");
    command = 0;
  }
  else if(command == SET_K_P_CONST || command == SET_K_I_CONST || command == SET_K_D_CONST)
  {
      //Serial.println("Postavljam PID constante");
      buf[pos++] = c;
      if(pos == SIZE)
      {
        long value = buf[3] | (buf[2] | (buf[1] | (buf[0]) << 8) << 8) << 8;
        set_PID_consts(command, value);
        pos = 0;
        command = 0;
      }
  }
  else if(command == RESET_ARDUINO)
  {
    Serial.println("reset");
    softwareReset(WDTO_1S);
  }
  else if(c >= RESET_ARDUINO)
  {
    command = c;
    if(c == GET_TICKS_L)
    {
      long ticks_l = enc_left->count;
      value_to_bytes(ticks_l);
      SPDR = buf[pos++];
    }
    else if(c == GET_TICKS_R)
    {
      long ticks_r = enc_right->count;
      value_to_bytes(ticks_r);
      SPDR = buf[pos++];
    }
    else if(c == GET_SENSOR_1)
    {
      long distance = sensor_0->get_distance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
      /*Serial.print("Senzor1: ");
      Serial.println(distance);*/
    }
    else if(c == GET_SENSOR_2)
    {
      long distance = sensor_1->get_distance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
      /*Serial.print("Senzor2: ");
      Serial.println(distance);*/
    }
    else if(c == GET_SENSOR_3)
    {
      long distance = sensor_2->get_distance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
      /*Serial.print("Senzor3: ");
      Serial.println(distance);*/
    }
    else if(c == GET_SENSOR_4)
    {
      long distance = sensor_3->get_distance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
      /*Serial.print("Senzor4: ");
      Serial.println(distance);*/
    }
    else if(c == GET_SENSOR_5)
    {
      long distance = sensor_4->get_distance();
      value_to_bytes(distance);
      SPDR = buf[pos++];
      /*Serial.print("Senzor5: ");
      Serial.println(distance);*/
    }
  }
}

void spi_init()
{
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);
  pos = 0;
  processSpiData = false;
}

void motor_init()
{
  attachInterrupt(digitalPinToInterrupt(enc_left->pinA), enc_left_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_left->pinB), enc_left_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_right->pinA), enc_right_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_right->pinB), enc_right_B, CHANGE);
}

void set_PID_consts(byte command, long value)
{
  if(command == SET_K_P_CONST)
  {
    k_p_angular_speed = value / 100.0;
    Serial.print("postavljam k_p: ");
    Serial.println(k_p_angular_speed);
    /*Serial.print(",  value: ");
    Serial.println(value);*/
  }
  else if(command == SET_K_I_CONST)
  {
    k_i_angular_speed = value / 100.0;
    Serial.print("postavljam k_i: ");
    Serial.println(k_i_angular_speed);
    /*Serial.print(",  value: ");
    Serial.println(value);*/
  }
  else if(command == SET_K_D_CONST)
  {
    k_d_angular_speed = value / 100.0;
    Serial.print("postavljam k_d: ");
    Serial.println(k_d_angular_speed);
    /*Serial.print(",  value: ");
    Serial.println(value);*/
  }
}

void set_angular_velocity(byte command, long value)
{
  //Serial.print(value);
  //Serial.print(" ");
  if(command == SET_ANGULAR_VELOCITY_L)
  {
    angular_speed_supervisor_left->set_ref_by_speed(value, false);
  }
  else if(command == SET_ANGULAR_VELOCITY_R)
  {
    angular_speed_supervisor_right->set_ref_by_speed(value, false);
    //Serial.println();
  }
}

void value_to_bytes(long value)
{
  buf[0] = value & 0xff;
  buf[1] = value >> 8 & 0xff;
  buf[2] = value >> 16 & 0xff;
  buf[3] = value >> 24 & 0xff;
}

void setup() {
  Serial.begin(115200);
  Serial.print("command1: ");
  Serial.println(command);
  motor_init();
  spi_init();
  Serial.print("command2: ");
  Serial.println(command);
  Serial.print(enc_left->count);
  Serial.print("   ");
  Serial.println(enc_right->count);
  delay(200);
}

void loop() {
  if(tmp_left != enc_left->count)
  {
    tmp_left = enc_left->count;
  }
  if(tmp_right != enc_right->count)
  {
    tmp_right = enc_right->count;
  }
  watch_time();
  angular_speed_supervisor_left->execute(dt_left);
  angular_speed_supervisor_right->execute(dt_left + 0.038);
  //Serial.println();
  delay(10);
}
