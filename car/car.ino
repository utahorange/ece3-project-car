#include <ECE3.h>

// Car No. 75

// Pin Assignments
const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin = 39;

const int LED_RF = 41;

#define NUM_SAMPLES 5
#define NUM_SENSORS 8
#define DEFAULT_SPEED 75 
#define TURN_SPEED 50 
#define U_TURN_SPEED 60

uint32_t calibrations[2][NUM_SENSORS] = {
    635,	626,	522,	612,	499,	635,	589,	681,
    1865,	1874,	1978,	1886,	1902,	1865,	1911,	1819
};

uint16_t sensor_values[NUM_SENSORS];
float calibrated_sensor[NUM_SENSORS];
// float weights[] = {-15, -14, -12, -8, 8, 12, 14, 15};
float weights[] = {-25, -14, -12, -8, 8, 12, 14, 25};
bool start_car = false;
float fused_sensors = 0;
float prev_fused_sensors = 0;

int basic_speed = 75;
float kd_constant = basic_speed * 0.5 / 1000; // 0.007; 
float kp_constant = kd_constant / 5; // btwn 5 to 10
uint8_t num_black = 0;
uint8_t num_complete_turns = 0;
int left_spd = 0;
int right_spd = 0;

/// --- Helpers ---
void sensor_calibrate() {
  for (uint32_t i = 0; i < NUM_SENSORS; i++)
  {
    calibrated_sensor[i] = (sensor_values[i] - calibrations[0][i]) * 1000.0 / calibrations[1][i];
    if (calibrated_sensor[i] > 3000) {
      calibrated_sensor[i] = 0;
    }
  }
}

/// positive output is the left sensor responding, neg output is right sensor responding
float sensor_weighting() {
    float ans = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        ans += weights[i] * calibrated_sensor[i];
    }
    return (ans / 8);
}

bool check_black() {
    uint8_t num = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      if (calibrated_sensor[i] > 800) {
        num++; 
      }
    }
    return (num > 5);
}

bool check_white() {
    uint8_t num = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      if (calibrated_sensor[i] < 300) {
        num++; 
      }
    }
    return (num > 7);
}

int abs(int n) {
  if (n >= 0) {
    return n;
  } else {
    return -1 * n;
  }
}

///////////////////////////////////
void setup() {
  // Pin Settings
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(LED_RF, OUTPUT);

  // Setting Initial Values
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,LOW);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,LOW);

  ECE3_Init(); // Used for encoder functionality

  Serial.begin(9600); // data rate for serial data transmission
  
  delay(2000); //Wait 2 seconds before starting 

  start_car = true;
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);
}

void do_u_turn() {
  basic_speed = U_TURN_SPEED;
  digitalWrite(left_dir_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  analogWrite(left_pwm_pin, basic_speed);
  analogWrite(right_pwm_pin, basic_speed);
  delay(1000);
}

void loop() {
  // Serial.print("--- Starting loop\n");
  uint16_t summed_values[NUM_SENSORS] = {0};

  // Take the average of 5 consecutive values for each sensor
  for (uint16_t j = 0; j < NUM_SAMPLES; j++){
    // Read raw sensor values
    ECE3_read_IR(sensor_values);

    // Add the current sensor values using a for loop
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      summed_values[i] += sensor_values[i];
    }
  }

  // Serial.print("sensor_values: ");
  // for (uint8_t i = 0; i < NUM_SENSORS; i++)
  // {
  //   sensor_values[i] = summed_values[i] / NUM_SAMPLES;
  //   Serial.print(sensor_values[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("\n");

  sensor_calibrate();

  // Serial.print("calibrated_sensor: ");
  // for (uint8_t i = 0; i < NUM_SENSORS; i++)
  // {
  //   Serial.print(calibrated_sensor[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("\n");
 
  bool is_black = check_black();
  if (is_black == true) {
    num_black++;
  } else {
    num_black = 0;
  }

  bool is_white = check_white();

  if (start_car) {
    if (num_black >= 2) {
      if (num_complete_turns == 0) {
        do_u_turn();
        num_complete_turns++;
      } else {
        digitalWrite(left_nslp_pin,LOW);
        digitalWrite(right_nslp_pin,LOW);
        start_car = false; // stop car for now
      }
    } else if (is_white) {
      digitalWrite(left_dir_pin,HIGH);
      digitalWrite(right_dir_pin,HIGH);
      analogWrite(left_pwm_pin, basic_speed);
      analogWrite(right_pwm_pin, basic_speed);
  
    } else {
      // digitalWrite(LED_RF, HIGH);
      // delay(250);
      // digitalWrite(LED_RF, LOW);
      // delay(250);
      fused_sensors = sensor_weighting();
      
      if (abs(fused_sensors) < 750) { // speedup
        if (basic_speed <= 175) {
             basic_speed = basic_speed * 1.05;
        }
        else {
          basic_speed = 175;
        }
        kd_constant = basic_speed * 0.25 / 1000;
        kp_constant = kd_constant * 3;
      } else if (abs(fused_sensors) > 2000) { // turns
        if (basic_speed >= TURN_SPEED) {
             basic_speed = basic_speed * 0.8;
        }
        else {
          basic_speed = TURN_SPEED;
        }
        kd_constant = basic_speed * 0.5 / 1000;
        kp_constant = kd_constant / 5;
      } else { // not turning and not on straight
        if (basic_speed >= DEFAULT_SPEED) {
             basic_speed = basic_speed * 0.95;
        }
        else {
          basic_speed = DEFAULT_SPEED;
        }
        kd_constant = basic_speed * 0.25 / 1000;
        kp_constant = kd_constant;
      }
      
      // --- Turn our car ---
      left_spd = basic_speed - (fused_sensors * kd_constant) + ((prev_fused_sensors - fused_sensors) * kp_constant);
      right_spd = basic_speed + (fused_sensors * kd_constant) - ((prev_fused_sensors - fused_sensors) * kp_constant);
      
      // flip direction of motor
      if (left_spd < 0) {
        digitalWrite(left_dir_pin,HIGH);
      } else {
        digitalWrite(left_dir_pin,LOW);
      }
      if (right_spd < 0) {
        digitalWrite(right_dir_pin,HIGH);
      } else {
        digitalWrite(right_dir_pin,LOW);
      }

      // set speed pins to abs value of their speeds
      analogWrite(left_pwm_pin, abs(left_spd));
      analogWrite(right_pwm_pin, abs(right_spd));
      
      // check if multiple
      prev_fused_sensors = fused_sensors;
    }
  }
  // delay(5000);
}
