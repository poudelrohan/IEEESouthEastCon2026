#include "hiwonder_board.h"
#include "hiwonder_i2c.h"

VersionState version;

static void buzzer_callback(TimerHandle_t xTimer) {
  static bool state;
  state = !state;
  if(state == true) {
    ledcWriteTone(LEDC_CHANNEL_0, 3000);
  }
  else {
    ledcWriteTone(LEDC_CHANNEL_0, 0);
  }
}

static void imu_update_callback(TimerHandle_t xTimer) {
  HW_Board *self = static_cast<HW_Board *>(pvTimerGetTimerID(xTimer));
  self->_imu_update();
  
}

static void bat_monitior_update_callback(TimerHandle_t xTimer) {
  HW_Board *self = static_cast<HW_Board *>(pvTimerGetTimerID(xTimer));
  self->bat_voltage_update();
}

void HW_Board::begin() {
  uint8_t count;
  int raw;
  int samples_voltage;
  
  Wire.setPins(SDA, SCL);
  Wire.setBufferSize(512);
  Wire.begin();
  pinMode(led_pin, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);  
  pinMode(button_pin, INPUT);  
  pinMode(32, OUTPUT);  

  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_BIT); // 5kHz 频率, 8位分辨率
  ledcAttachPin(BUZZER_PIN, LEDC_CHANNEL_0);

  buzzer_timer = xTimerCreate("buzzer_timer", 
                    pdMS_TO_TICKS(200), 
                    pdTRUE, 
                    NULL, 
                    buzzer_callback);

  adc1_config_channel_atten(ADC_PIN, ADC_ATTEN);
  adc_chars = (esp_adc_cal_characteristics_t *) malloc(sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, adc_chars);
  bat_monitor_timer = xTimerCreate("bat_monitor_timer", 
                    pdMS_TO_TICKS(bat_monitor_update_interval), 
                    pdTRUE, 
                    this, 
                    bat_monitior_update_callback);  
  for(uint8_t i = 0; i < 50; i++) {
    raw = adc1_get_raw(ADC_PIN);
    samples_voltage = esp_adc_cal_raw_to_voltage(raw, adc_chars);
    samples_voltage = ((R21 + R22) / R22) * samples_voltage;
    if(abs(last_sample_voltage - samples_voltage) < 500 && abs(samples_voltage - 7400) < 1500) {
      count++;
    }
    last_sample_voltage = samples_voltage;
    ESP_LOGI("board", "voltage: %d count:%d\n", samples_voltage, count);
  }

  if(count == 49) {
    bat_voltage = samples_voltage - 200;
    digitalWrite(led_pin, HIGH);
    version = V1_1;
    
  }
  else {
    version = V1_0;
  }

  xTimerStart(bat_monitor_timer, 0);  

  /* QMI8658 INIT */
  if(!qmi.begin(Wire, QMI8658_H_SLAVE_ADDRESS, sda_pin, scl_pin))
  {
    ESP_LOGW("Robot", "Failed to find QMI8658 - check your wiring!\n");
    while (true);
  }

  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_2G, 
                          SensorQMI8658::ACC_ODR_1000Hz, 
                          SensorQMI8658::LPF_MODE_0,
                          true);

  qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS,
                      SensorQMI8658::GYR_ODR_448_4Hz,
                      SensorQMI8658::LPF_MODE_1,
                      true);
  /* In 6DOF mode (accelerometer and gyroscope are both enabled),
  *
  *the output data rate is derived from the nature frequency of gyroscope
  */
  qmi.enableGyroscope();
  qmi.enableAccelerometer();    
  filter.begin(100); 
  imu_calibarate();
  imu_timer = xTimerCreate("imu_update_timer", 
                           pdMS_TO_TICKS(imu_update_interval), 
                           pdTRUE, 
                           this, 
                           imu_update_callback);   

  xTimerStart(imu_timer, 0);    

  ledcWriteTone(LEDC_CHANNEL_0, 3000);
  delay(100);
  ledcWriteTone(LEDC_CHANNEL_0, 0);
  
}

bool HW_Board::get_button_state() {
  return (bool)digitalRead(button_pin);
}

void HW_Board::_imu_update() {
  if(qmi.getDataReady()) {
    qmi.getAccelerometer(acc.x, acc.y, acc.z);
    qmi.getGyroscope(gyro.x, gyro.y, gyro.z);

    acc_cali.x = acc.x - acc_offset.x;
    acc_cali.y = acc.y - acc_offset.y;
    acc_cali.z = acc.z - acc_offset.z;

    gyro_cali.x = gyro.x - gyro_offset.x;
    gyro_cali.y = gyro.y - gyro_offset.y;
    gyro_cali.z = gyro.z - gyro_offset.z;

    filter.updateIMU(gyro_cali.x, gyro_cali.y, gyro_cali.z, acc_cali.x, acc_cali.y, acc_cali.z);

    euler.x = filter.getRoll();
    euler.y = filter.getPitch();
    euler.z = filter.getYaw();
  }
}

void HW_Board::imu_calibarate() {
  uint16_t count = 0;
  uint16_t sample_num = 200;

	for(;;)
	{
		if(qmi.getDataReady())
		{
			qmi.getAccelerometer(acc.x, acc.y, acc.z);
			qmi.getGyroscope(gyro.x, gyro.y, gyro.z);

      acc_offset.x += acc.x;
      acc_offset.y += acc.y;
      acc_offset.z += acc.z;
      
      gyro_offset.x += gyro.x;
      gyro_offset.y += gyro.y;
      gyro_offset.z += gyro.z;
      count++;
			if(count == sample_num) break;
		}	
	}

	acc_offset.x /= sample_num;
	acc_offset.y /= sample_num;
	acc_offset.z = (acc_offset.z / sample_num) - 1.0f;

	gyro_offset.x /= sample_num;
	gyro_offset.y /= sample_num;
	gyro_offset.z /= sample_num;  
}

void HW_Board::get_imu_acc(float *val) {
  val[0] = acc_cali.x;
  val[1] = acc_cali.y;
  val[2] = acc_cali.z;
}

void HW_Board::get_imu_gyro(float *val) {
  val[0] = gyro_cali.x;
  val[1] = gyro_cali.y;
  val[2] = gyro_cali.z;
}

void HW_Board::get_imu_euler(float *val) {
  val[0] = euler.x;
  val[1] = euler.y;
  val[2] = euler.z;
}

void HW_Board::imu_update(bool state) {
  if(state == true && imu_on_state == false) {
    imu_on_state = true;
    xTimerStart(imu_timer, 0);  
    Serial.println("sss");
  }
  else if(state == false && imu_on_state == true){
    imu_on_state = false;
    xTimerStop(imu_timer, portMAX_DELAY);
  }
}

int HW_Board::get_sound_val() {
  return analogRead(sound_pin);
}

void HW_Board::bat_voltage_update() {
  int raw;
  uint16_t bat;
  uint16_t samples_voltage;
  uint32_t sum = 0;

  if(version == V1_1) {
    raw = adc1_get_raw(ADC_PIN);
    samples_voltage = esp_adc_cal_raw_to_voltage(raw, adc_chars);
    samples_voltage = ((R21 + R22) / R22) * samples_voltage;
    // 插入到滑动窗口
    voltage_buffer[voltage_index] = samples_voltage;
    voltage_index = (voltage_index + 1) % WINDOW_SIZE;

    // 计算滑动平均
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += voltage_buffer[i];
    }

    bat = sum / WINDOW_SIZE;  // 平均电压

    if(voltage_buffer[WINDOW_SIZE - 1] != 0) {
      bat_voltage = bat;
    }
    
    if(bat_voltage < 7000 && voltage_buffer[WINDOW_SIZE - 1] != 0 && buzzer_on_state == false) {
      buzzer_on_state = true;
      xTimerStart(buzzer_timer, 0);
    }
    
    if(bat_voltage >= 7400 && buzzer_on_state == true) {
      buzzer_on_state = false;
      xTimerStop(buzzer_timer, portMAX_DELAY);
      ledcWriteTone(LEDC_CHANNEL_0, 0);
    }
  }
  else {
    wire_read_array(BAT_DEC_ADDR, BAT_READ_REG, (uint8_t *)&bat_voltage, 2);

    if(bat_voltage < 7000 && buzzer_on_state == false) {
      buzzer_on_state = true;
      xTimerStart(buzzer_timer, 0);
    }
    
    if(bat_voltage >= 7400 && buzzer_on_state == true) {
      buzzer_on_state = false;
      xTimerStop(buzzer_timer, portMAX_DELAY);
      ledcWriteTone(LEDC_CHANNEL_0, 0);
    }
  }
}
