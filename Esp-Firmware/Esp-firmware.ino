// #include <SPI.h>s
#include <Wire.h>
#include <WiFi.h>
#include <cmath>
#include "mapper.h"
#include "Filter.h"

#include "haptics.h"

// --------  Wifi Credentials   ----------

// Home
const char* SSID     = "Peberfrugt";
const char* PASSWORD = "un3bout3ill3alam3r";

// Mathias K Iphone
// const char* SSID     = "Graveyard";
// const char* PASSWORD = "yoloyolo";

// Mathias B smartphone
// const char* SSID     = "MathiasB";
// const char* PASSWORD = "yoloyolo";

const int FSR_PIN = 34;
const int SEL_PIN = 0;

const uint8_t I2C_BUF_SIZE = 10;
const uint8_t CHECKSUMSIZE = 2;


const uint32_t HAPTICS_UPDATE_RATE = 10000; // 10 KHz
const uint32_t FSR_UPDATE_RATE = 10000; // 100 Hz
// const uint32_t I2CUPDATE_FREQ = 400000; // Fast mode;
// const uint32_t I2CUPDATE_FREQ = 1000000; // Fast mode plus;
const uint32_t I2CUPDATE_FREQ = 3400000; // high speed mode;
const int32_t DEBOUNCE_TIME = 10000; // 10 ms


HapticKnob knob;
int test_val = 0;

// Libmapper variables
mapper_signal in_sig1;
mapper_signal in_sig2;
mapper_signal in_sig3;
mapper_signal in_sig4;
mapper_signal out_sig1;
mapper_signal out_sig2;
mapper_signal out_sig3;
// mapper_signal out_sig4;
// mapper_signal out_sig5;
mapper_signal out_sig6;
mapper_device dev;


// Noise filter
Filter fsr_filt;


int pressure = 0;
int sel = 0;
int sel_min = 0;
int sel_max = 0;
// System variables
int err = 0;
int err_count = 0;
int angle_out = 0;
uint32_t last_time;
uint32_t last_time_errprint;
uint32_t last_time_fsr;
uint32_t now;
uint8_t tx_data[I2C_BUF_SIZE];
uint8_t rx_data[I2C_BUF_SIZE];
uint16_t checksum_rx = 0;
uint16_t checksum_tx = 0;

uint16_t calcsum(uint8_t buf[], uint8_t length) {
  uint16_t val = 0;
  for (int k = 0; k < length; k++) {
    val += buf[k];
  }
  return val;
}

void init_wifi(const char* ssid, const char* password) {
  printf("Connecting to: %s", ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    printf(".");
  }
  IPAddress ip = WiFi.localIP();
  printf("\n Wifi Connected \n IP adress:  %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

int update_fsr(int* pressure) {
  int read = analogRead(FSR_PIN);
  read -= 1350; // Remove static pressure
  read = fsr_filt.update(read);
  *pressure = read < 0 ? 0 : read;
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


int read_param(float * param, uint8_t*  data, uint8_t length) {
  memcpy(param, data, length);
  if ( std::isnan(*param)) {
    return 1;
  } else {
    return 0;
  }
}

int receiveI2C(HapticKnob * knob_) {
  Wire.requestFrom(8, I2C_BUF_SIZE + CHECKSUMSIZE);
  uint8_t k = 0;
  while (Wire.available()) {
    rx_data[k] = Wire.read();
    k++;
  }
  if (k != I2C_BUF_SIZE + CHECKSUMSIZE) { // check if all data is recieved
    Serial.print("Error in recieved data. Bytes missing : ");
    Serial.println(I2C_BUF_SIZE + CHECKSUMSIZE - k);
    return 1;
  }
  else {
    memcpy(&checksum_rx, rx_data + I2C_BUF_SIZE, 2); // read checksum
    if (checksum_rx != calcsum(rx_data, I2C_BUF_SIZE)) { // error in recieved data
      return 2;
    }
    else { // Succesfull recieve
      memcpy(&knob_->angle, rx_data, 2);
      // printf("angle : %d , ", static_cast<int16_t>(knob_->angle));
      // memcpy(&knob_->angle_wrapped, rx_data + 2, 2);
      memcpy(&knob_->velocity, rx_data + 4, 4);
      return 0; //Return 0 if no error has occured
    }
  }
}

void sendI2C(HapticKnob * knob_) {
  Wire.beginTransmission(8); // transmit to device #8
  memcpy(tx_data, &knob_->torque, 2);
  memcpy(tx_data + 2, &knob_->target_velocity, 4);
  memcpy(tx_data + 6, &knob_->active_mode->pid_mode, 1);
  checksum_tx = calcsum(tx_data, I2C_BUF_SIZE);
  memcpy(tx_data + I2C_BUF_SIZE, &checksum_tx, 2);
  int n = Wire.write(tx_data, I2C_BUF_SIZE + CHECKSUMSIZE);
  Wire.endTransmission();    // stop transmitting
}

void in_sig1_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.torque_scale =  *((float*)value);
}

void in_sig2_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.angle_scale =  *((float*)value);
}

void in_sig3_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.set_mode(*((int32_t*)value));
}

void in_sig4_callback(mapper_signal sig, mapper_id instance, const void *value, int count, mapper_timetag_t *tt) {
  knob.target_velocity = (*((float*)value));
}

void init_mapper_signals() {
  dev = mapper_device_new("HapticKnob", 0, 0);

  // Init libmapper inputs
  float torque_scale_min = 0;
  float torque_scale_max = 230;
  in_sig1 = mapper_device_add_input_signal(dev, "Torque", 1, 'f', "Ncm", &torque_scale_min, &torque_scale_max, in_sig1_callback, NULL);

  float angle_scale_min = 0;
  float angle_scale_max = 30;
  in_sig2 = mapper_device_add_input_signal(dev, "DetentDensity", 1, 'f', "Rad", &angle_scale_min, &angle_scale_max, in_sig2_callback, NULL);

  int mode_min = 0;
  int mode_max = knob.num_modes - 1;
  sel_max = mode_max;
  in_sig3 = mapper_device_add_input_signal(dev, "Mode", 1, 'i', "mode", &mode_min, &mode_max, in_sig3_callback, NULL);


  float vel_min = -700;
  float vel_max = 700;
  in_sig4 = mapper_device_add_input_signal(dev, "TargetVelocity", 1, 'f', "Rpm", &vel_min, &vel_max, in_sig4_callback, NULL);

  // Init libmapper outputs
  int angle_min = 0;
  int angle_max = 3600;
  out_sig1 = mapper_device_add_output_signal(dev, "Angle", 1, 'i', 0, &angle_min, &angle_max);

  out_sig2 = mapper_device_add_output_signal(dev, "Velocity", 1, 'f', 0, &vel_min, &vel_max);

  int trig_min = 0;
  int trig_max = 1;
  out_sig3 = mapper_device_add_output_signal(dev, "Trigger", 1, 'i', 0, &trig_min, &trig_max);

  // int pressure_min = 0;
  // int pressure_max = 600;
  // out_sig4 = mapper_device_add_output_signal(dev, "Press", 1, 'i', 0, &pressure_min, &pressure_max);

  // out_sig5 = mapper_device_add_output_signal(dev, "Select", 1, 'i', 0, &sel_min, &sel_max);

  float acc_min = -100;
  float acc_max = 100;
  out_sig6 = mapper_device_add_output_signal(dev, "Acceleration", 1, 'f', 0, &acc_min, &acc_max);


}

void setup() {
  Serial.begin(115200);
  init_wifi(SSID, PASSWORD);
  init_mapper_signals();

  Wire.begin();
  Wire.setClock(I2CUPDATE_FREQ); // Fast mode plus
  knob.set_mode(HapticKnob::CLICK);

  pinMode(SEL_PIN, INPUT);
}

void loop() {


  now = micros();
  if (now - last_time > HAPTICS_UPDATE_RATE) {
    // mapper_device_poll(dev, 0); // Update libmapper connections
    err = receiveI2C(&knob);
    err_count += err;

    if (err) {}
    else {  // Update torque if valid angle measure is recieved.
      knob.update();
      mapper_signal_update(out_sig1, &angle_out, 1, MAPPER_NOW);
      mapper_signal_update(out_sig2, &knob.velocity, 1, MAPPER_NOW);
      mapper_signal_update(out_sig3, &knob.trigger, 1, MAPPER_NOW);
      mapper_signal_update(out_sig6, &knob.acceleration, 1, MAPPER_NOW);
      sendI2C(&knob);
    }
    last_time = now;
  }

  // if (now - last_time_fsr > FSR_UPDATE_RATE) {
  //   update_fsr(&pressure);
  //   mapper_signal_update(out_sig4, &pressure, 1, MAPPER_NOW);
  //   last_time_fsr = now;

  //   if (update_btn(SEL_PIN)) {
  //     sel++;
  //     sel %= sel_max;
  //     mapper_signal_update(out_sig5, &sel, 1, MAPPER_NOW);
  //   };
  // }

  // Error check for tuning of I2c frequency and overall programspeed
  // if (now - last_time_errprint > 5000000) { //10 seconds
  //   Serial.print("Errors in last 5 seconds:  ");  Serial.println(err_count);
  //   err_count = 0;
  //   last_time_errprint = now;
  // }

}

