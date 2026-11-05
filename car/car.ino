#include <ECE3.h> // Used for encoder functionality

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
uint32_t calibrations[2][NUM_SENSORS] = {
    635,	626,	522,	612,	499,	635,	589,	681,
    1865,	1874,	1978,	1886,	1902,	1865,	1911,	1819
};

uint16_t sensor_values[NUM_SENSORS];
double calibrated_sensor[NUM_SENSORS];
double weights[] = {-15, -14, -12, -8, 8, 12, 14, 15};
bool start_car = false;
double fused_sensors = 0;
double kv_constant = 0.007; 
int basic_speed = 30;
int left_spd = 0;
int right_spd = 0;

/// --- Helpers ---
void sensor_calibrate() {
  for (uint32_t i = 0; i < NUM_SENSORS; i++)
  {
    // if (calibrations[0][i] > sensor_values[i]) {
    //   calibrated_sensor[i] = 0;
    // } else {
      calibrated_sensor[i] = (sensor_values[i] - calibrations[0][i]) * 1000.0 / calibrations[1][i];
    // }
    if (calibrated_sensor[i] > 3000) {
      calibrated_sensor[i] = 0;
    }
  }
}

/// positive output is the left sensor responding, neg output is right sensor responding
double sensor_weighting() {
    double ans = 0;
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
    return (num > 6);
}

int abs(int n) {
  if (n >= 0) {
    return n;
  } else {
    return -1 * n;
  }
}

// Encoder Variables
long enc_bin_cnt;
const unsigned long enc_bin_len = 50; // 50 ms bins
    // Encoder Speed Calculation Explanation:
    // We wait for a set amount of time (enc_bin_len), and find how many
    // times the encoder has incremented in that period. We call 
    // this period a bin when refering to the encoder. The number 
    // encoder counts per bin is a proportional to speed.

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
}

// void print_arr(uint8_t arr[NUM_SENSORS]) {
//    // TODO
//    for (uint8_t i = 0; i < NUM_SENSORS; i++)
//    {
//     Serial.print(arr[i]);
//     Serial.print(" ");
//    }
// }

void loop() {
  // Report encoder counts and speed
  Serial.print("--- Starting loop\n");
  uint16_t summed_values[8] = {0};

  // Take the average of 5 consecutive values for each sensor
  for (uint16_t j = 0; j < NUM_SAMPLES; j++){
    // Read raw sensor values
    ECE3_read_IR(sensor_values);

    // Add the current sensor values using a for loop
    for (unsigned char i = 0; i < 8; i++)
    {
      summed_values[i] += sensor_values[i];
    }
  }

  Serial.print("sensor_values: ");
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    sensor_values[i] = summed_values[i] / NUM_SAMPLES;
    Serial.print(sensor_values[i]);
    Serial.print(" ");
  }
  Serial.print("\n");

  sensor_calibrate();

  Serial.print("calibrated_sensor: ");
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(calibrated_sensor[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
 
  bool is_black = check_black();
  if (!start_car && is_black) {
    Serial.print("--- Start car set to true\n");
    start_car = true;
    digitalWrite(left_nslp_pin,HIGH);
    digitalWrite(right_nslp_pin,HIGH);
  }

  if (start_car) {
    // digitalWrite(LED_RF, HIGH);
    // delay(250);
    // digitalWrite(LED_RF, LOW);
    // delay(250);
    fused_sensors = sensor_weighting();
   
    // --- Turn our car ---
    left_spd = basic_speed - fused_sensors * kv_constant;
    right_spd = basic_speed + fused_sensors * kv_constant;
    
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
  }
  // delay(5000);
}

