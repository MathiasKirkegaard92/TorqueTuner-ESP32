#include <Wire.h>
#include <WiFi.h>
#include <cmath>
#include "mapper.h"
#include "Filter.h"
#include <TinyPICO.h>

#include "haptics.h"
#include "TorqueTuner.h"

// For disabling power saving
#include "esp_wifi.h"

// --------  Wifi Credentials   ----------

// Home
// const char* SSID     = "Peberfrugt";
// const char* PASSWORD = "un3bout3ill3alam3r";

// Home
// const char* SSID     = "tstick_network_SPCL";
// const char* PASSWORD = "mappings";

// Sommerhus
const char* SSID     = "Fibernet-IA01816468";
const char* PASSWORD = "HJ0Knbr9";


// Mathias K Iphone
// const char* SSID     = "Graveyard";
// const char* PASSWORD = "yoloyolo";

// Mathias B smartphone
// const char* SSID     = "MathiasB";
// const char* PASSWORD = "yoloyolo";


// #define TSTICKJOINT 1

const int SEL_PIN = 0;

#ifdef TSTICKJOINT
const int SDA_PIN = 26;
const int SCL_PIN = 25;
#else
const int SDA_PIN = 21;
const int SCL_PIN = 22;
#endif

// I2C variables
const uint8_t I2C_BUF_SIZE = 10;
const uint8_t CHECKSUMSIZE = 2;
uint8_t tx_data[I2C_BUF_SIZE];
uint8_t rx_data[I2C_BUF_SIZE];
uint16_t checksum_rx = 0;
uint16_t checksum_tx = 0;

// Timing variables
const uint32_t HAPTICS_UPDATE_RATE = 500 ; // 2 KHz
const uint32_t I2CUPDATE_FREQ = 3400000; // high speed mode;
const uint32_t DEBOUNCE_TIME = 10000; // 10 ms
const uint32_t MAINTENANCE_RATE = 30000000; // 30 s
const uint32_t GUI_RATE = 33000; //  30 FPS

// Initialise the TinyPICO library
TinyPICO tp = TinyPICO();

// Initialize TorqueTuner
TorqueTuner knob;

// Init i2c device
HapticDev i2c;

// State flags
int connected = 0;
bool is_playing = true;

// Libmapper variables
mapper_signal in_sig_scale;
mapper_signal in_sig_stretch;
mapper_signal in_sig_mode;
mapper_signal in_sig_target_velocity;
mapper_signal in_sig_offset;
mapper_signal in_sig_damping;

mapper_signal out_sig_angle;
mapper_signal out_sig_velocity;
mapper_signal out_sig_trigger;
mapper_signal out_sig_speed;
mapper_signal out_sig_quant_angle;
mapper_signal out_sig_acceleration;
mapper_device dev;

int pressure = 0;
int sel = 0;
int sel_min = 0;
int sel_max = 0;

// System variables
int err = 0;
int err_count = 0;
uint32_t last_time = 0;
uint32_t last_time_errprint = 0;
uint32_t last_time_maintenance = 0;
uint32_t last_time_gui = 0;
uint32_t now = 0;

uint16_t calcsum(uint8_t buf[], uint8_t length) {
  uint16_t val = 0;
  for (int k = 0; k < length; k++) {
    val += buf[k];
  }
  return val;
}

int init_wifi(const char* ssid, const char* password, int timeout_ms) {
  int time = millis();
  printf("Connecting to: %s", ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    printf(".");
    if (millis() - time > timeout_ms) {
      printf(" Timeout waiting for connection \n");
      return 0;
    }
    delay(500);
  }
  IPAddress ip = WiFi.localIP();
  printf("\n Wifi Connected \n IP adress:  %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return 1;
}


bool update_btn(const int pin) {
  static bool last_val;
  static bool has_changed;
  static int32_t last_change;
  // Read button pin
  int32_t now = esp_timer_get_time();
  bool val =  !digitalRead(pin);
  if (val != last_val) {
    last_val = val;
    last_change = now;
    has_changed = true;
  }

  // Debounce button and trigger on release
  if (has_changed && (now - last_change) > DEBOUNCE_TIME  && !val) {
    has_changed = false;
    return true;
  } else {
    return false;
  }
}

esp_err_t receiveI2C(TorqueTuner * knob_) {
  esp_err_t err = i2c.read(rx_data, I2C_BUF_SIZE + CHECKSUMSIZE);
  memcpy(&knob_->angle, rx_data, 2);
  memcpy(&knob_->velocity, rx_data + 4, 4);

  // Wire.requestFrom(8, I2C_BUF_SIZE + CHECKSUMSIZE);
  // uint8_t k = 0;
  // while (Wire.available()) {
  //   rx_data[k] = Wire.read();
  //   k++;
  // }
  // if (k != I2C_BUF_SIZE + CHECKSUMSIZE) { // check if all data is recieved
  //   printf("Error in recieved data. Bytes missing :  %i\n", I2C_BUF_SIZE + CHECKSUMSIZE - k);
  //   return 1;
  // }
  // else {
  //   memcpy(&checksum_rx, rx_data + I2C_BUF_SIZE, 2); // read checksum
  //   if (checksum_rx != calcsum(rx_data, I2C_BUF_SIZE)) { // error in recieved data
  //     return 2;
  //   }
  //   else { // Succesfull recieve
  //     memcpy(&knob_->angle, rx_data, 2);
  //     memcpy(&knob_->velocity, rx_data + 4, 4);
  //     return 0; //Return 0 if no error has occured
  //   }
  // }

  return err;
}


esp_err_t sendI2C(TorqueTuner * knob_) {
  // Wire.beginTransmission(8); // transmit to device #8
  memcpy(tx_data, &knob_->torque, 2);
  memcpy(tx_data + 2, &knob_->target_velocity, 4);
  memcpy(tx_data + 6, &knob_->active_mode->pid_mode, 1);
  checksum_tx = calcsum(tx_data, I2C_BUF_SIZE);
  memcpy(tx_data + I2C_BUF_SIZE, &checksum_tx, 2);
  esp_err_t err = i2c.write(tx_data, I2C_BUF_SIZE + CHECKSUMSIZE);
  // int n = Wire.write(tx_data, I2C_BUF_SIZE + CHECKSUMSIZE);
  // Wire.endTransmission();    // stop transmitting
  return err;
}

void in_sig_scale_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.scale =  *((float*)value);
}

void in_sig_stretch_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.stretch =  *((float*)value);
}

void in_sig_offset_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.active_mode->offset = (*((float*)value));
}

void in_sig_mode_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.set_mode(*((int32_t*)value));
}

void in_sig_target_velocity_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.target_velocity = (*((float*)value));
}

void in_sig_damping_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.active_mode->damping = (*((float*)value));
}

void init_mapper_signals() {
  dev = mapper_device_new("TorqueTuner", 0, 0);

  // Init libmapper inputs
  float scale_min = -230;
  float scale_max = 230;
  in_sig_scale = mapper_device_add_input_signal(dev, "Scale", 1, 'f', "Ncm", &scale_min, &scale_max, in_sig_scale_callback, NULL);

  float angle_scale_min = 0;
  float angle_scale_max = 30;
  in_sig_stretch = mapper_device_add_input_signal(dev, "Stretch", 1, 'f', "ratio", &angle_scale_min, &angle_scale_max, in_sig_stretch_callback, NULL);

  int mode_min = 0;
  int mode_max = knob.num_modes - 1;
  sel_max = mode_max;
  in_sig_mode = mapper_device_add_input_signal(dev, "Mode", 1, 'i', "mode", &mode_min, &mode_max, in_sig_mode_callback, NULL);

  float vel_min = -700;
  float vel_max = 700;
  in_sig_target_velocity = mapper_device_add_input_signal(dev, "TargetVelocity", 1, 'f', "Rpm", &vel_min, &vel_max, in_sig_target_velocity_callback, NULL);

  float offset_min = -1800;
  float offset_max = 1800;
  in_sig_offset = mapper_device_add_input_signal(dev, "Offset", 1, 'f', "degrees", &offset_min, &offset_max, in_sig_target_velocity_callback, NULL);

  float damping_min = -1;
  float damping_max = 1;
  in_sig_damping = mapper_device_add_input_signal(dev, "Damping", 1, 'f', "ratio", &offset_min, &offset_max, in_sig_target_velocity_callback, NULL);

  // Init libmapper outputs
  int angle_min = 0;
  int angle_max = 3600;
  out_sig_angle = mapper_device_add_output_signal(dev, "Angle", 1, 'i', 0, &angle_min, &angle_max);

  out_sig_velocity = mapper_device_add_output_signal(dev, "Velocity", 1, 'f', 0, &vel_min, &vel_max);

  int trig_min = 0;
  int trig_max = 1;
  out_sig_trigger = mapper_device_add_output_signal(dev, "Trigger", 1, 'i', 0, &trig_min, &trig_max);

  float speed_min = 0;
  float speed_max = vel_max;
  out_sig_speed = mapper_device_add_output_signal(dev, "Speed", 1, 'f', 0, &speed_min, &speed_max);

  out_sig_quant_angle = mapper_device_add_output_signal(dev, "QuantAngle", 1, 'i', 0, &angle_min, &angle_max);

  float acc_min = -100;
  float acc_max = 100;
  out_sig_acceleration = mapper_device_add_output_signal(dev, "Acceleration", 1, 'f', 0, &acc_min, &acc_max);

}

void setup() {
  Serial.begin(115200);
  connected = init_wifi(SSID, PASSWORD, 30000);
  // if (connected) {
  init_mapper_signals();
  // }
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Wire.begin(SDA_PIN, SCL_PIN);
  // Wire.setClock(I2CUPDATE_FREQ); // Fast mode plus
  i2c.init(8, SDA_PIN, SCL_PIN);
  knob.set_mode(TorqueTuner::LINSPRING);

  // Make a reading for initilization
  // int err = 1;
  // while (err) {
  //   // printf("%i \n", sendI2C(&knob) );
  //   knob.torque = 100;
  //   printf("Send %i \n", sendI2C(&knob));
  //   delay(500);
  //   printf("Recieved: %i \n", receiveI2C(&knob) );
  //   printf("%i\n", knob.angle);
  // }

  pinMode(SEL_PIN, INPUT);
}

void loop() {

  // Update libmapper connections
  mapper_device_poll(dev, 0);

  now = micros();
  if (now - last_time > HAPTICS_UPDATE_RATE) {


    // Recieve Angle and velocity from servo
    err = receiveI2C(&knob);

    if (err != ESP_OK) {printf("i2c error \n");}
    else {

      // Update torque if valid angle measure is recieved.
      if (is_playing) {
        knob.update();
      } else {
        // OBS: Consider not updating? assign last last value instead? //
        knob.torque = 0;
        knob.target_velocity = 0;
      }
      sendI2C(&knob);

      // Update libmapper outputs
      mapper_signal_update(out_sig_angle, &knob.angle_out, 1, MAPPER_NOW);
      mapper_signal_update(out_sig_velocity, &knob.velocity, 1, MAPPER_NOW);
      mapper_signal_update(out_sig_trigger, &knob.trigger, 1, MAPPER_NOW);
      float speed = abs(knob.velocity);
      mapper_signal_update(out_sig_speed, &speed, 1, MAPPER_NOW);
      mapper_signal_update(out_sig_quant_angle, &knob.angle_discrete, 1, MAPPER_NOW);
      mapper_signal_update(out_sig_acceleration, &knob.acceleration, 1, MAPPER_NOW);

    }
    last_time = now;
  }

  /* ------------------------------*/
  /* -------- GUI update  ---------*/
  /* ------------------------------*/

  if (now - last_time_gui > GUI_RATE) {
    // printf("DATAREADY %i, %i \n", knob.angle_out);
    // printf("Target velocity: %f \n", knob.target_velocity);
    printf("%i\n", knob.angle);
    last_time_gui = now;
  }


  /* ------------------------------*/
  /* -------- Maintenance ---------*/
  /* ------------------------------*/

// Reconnect to wifi if not connected
  if (now - last_time_maintenance > MAINTENANCE_RATE) {
    if (WiFi.status() != WL_CONNECTED) {
      connected = init_wifi(SSID, PASSWORD, 0);
      delay(100);
      if (WiFi.status() == WL_CONNECTED) {
        // init_mapper_signals();
      }
    }
    // Check LiPo battery voltage
    float voltage = tp.GetBatteryVoltage();
    if (voltage < 3.1) {
      is_playing = false;
      printf("Battery voltage is under 3.3 V, shotting down ...:\n" );
    }
    printf("Battery voltage is: %f\n", voltage);
    last_time_maintenance = now;
  }
}
