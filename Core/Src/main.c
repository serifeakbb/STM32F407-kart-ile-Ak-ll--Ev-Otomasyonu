/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "us_delay.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_gpio.h"
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILTER_SAMPLES 5
#define MPU6050_1_ADDR 0xD0
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define WHO_AM_I_REG 0x75

// Duruş eşik değerleri
#define FORWARD_THRESHOLD -0.8
#define BACKWARD_THRESHOLD 0.8
#define SIDE_THRESHOLD 0.5

#define GAS_THRESHOLD 2000 // Gaz eşik değeri

// Sıcaklık eşik değerleri
#define TEMP_HIGH_THRESHOLD 30.0   // Kırmızı LED'in yanacağı sıcaklık
#define TEMP_LOW_THRESHOLD 15.0    // Mavi LED'in yanacağı sıcaklık
#define MAX_RESPONSE_TRIES 5       // Yanıt için deneme sayısı
#define WATER_THRESHOLD 2000       // Su seviyesi eşiği
#define SERVO_MIN 500             // 1ms pulse (0°)
#define SERVO_MAX 2500            // 2ms pulse (180°)
//deprem
#define GYRO_CONFIG_REG 0x1B
#define MPU6050_2_ADDR (0x68 << 1)  // I2C address for MPU6050
// Threshold definitions
#define EARTHQUAKE_THRESHOLD 1.5f  // g value
#define SOIL_THRESHOLD 2000        // ADC value
#define FULL_DISTANCE 5            // cm (for trash bin)
#define LDR_THRESHOLD 2000         // ADC value (for lighting)
#define TIMEOUT_DURATION 30000     // ms (15 seconds for lighting)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
typedef struct {
float Accel_X;
float Accel_Y;
float Accel_Z;
} MPU6050_t;

MPU6050_t MPU6050_1;
MPU6050_t MPU6050_2;
float filter_buffer[FILTER_SAMPLES][3];
uint8_t filter_index = 0;
uint16_t gas_value = 0;
uint8_t gas_leak_detected = 0;
float temperature = 0, humidity = 0;

uint8_t earthquake_detected = 0;
uint8_t motionDetected = 0;
uint32_t motionDetectedTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_1_Write_Byte(uint8_t reg, uint8_t data);
void MPU6050_1_Init(void);
void MPU6050_1_Read_All(void);
uint8_t MPU6050_1_Read_Byte(uint8_t reg);
void apply_filter(float *x, float *y, float *z);
void alert_user(uint8_t state);
void check_posture(float x, float y);
void Servo_SetAngle(uint16_t angle);
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read(void);
void Read_DHT11(float *temperature, float *humidity);
void setServoAngle(uint16_t angle);



void MPU6050_2_Init(void);
void MPU6050_2_Read_Accel(float *ax, float *ay, float *az);

/* Soil Moisture Functions */
uint32_t Read_Soil_Sensor(void);

/* Trash Bin Functions */
void enable_DWT(void);
void delay_us(uint32_t us);
float get_distance_no_timer(void);

/* Lighting Functions */
uint16_t Read_ADC(ADC_HandleTypeDef*hadc2);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU6050_1_Write_Byte(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c3, MPU6050_1_ADDR, reg, 1, &data, 1, 100);
}

void apply_filter(float *x, float *y, float *z) {
    filter_buffer[filter_index][0] = *x;
    filter_buffer[filter_index][1] = *y;
    filter_buffer[filter_index][2] = *z;
    filter_index = (filter_index + 1) % FILTER_SAMPLES;

    float sum_x = 0, sum_y = 0, sum_z = 0;
    for(int i = 0; i < FILTER_SAMPLES; i++) {
        sum_x += filter_buffer[i][0];
        sum_y += filter_buffer[i][1];
        sum_z += filter_buffer[i][2];
    }

    *x = sum_x / FILTER_SAMPLES;
    *y = sum_y / FILTER_SAMPLES;
    *z = sum_z / FILTER_SAMPLES;
}

void MPU6050_1_Init(void) {
    // MPU6050 reset
    MPU6050_1_Write_Byte(PWR_MGMT_1_REG, 0x80);
    HAL_Delay(100);
    // WHO_AM_I kontrolü
    uint8_t who_am_i = MPU6050_1_Read_Byte(WHO_AM_I_REG);
    if(who_am_i != 0x68) {
        Error_Handler(); // Sensör bulunamadı
    }
    // Wake up device
    MPU6050_1_Write_Byte(PWR_MGMT_1_REG, 0x00);
    // Set sample rate
    MPU6050_1_Write_Byte(SMPLRT_DIV_REG, 0x07);
    // Configure accelerometer
    MPU6050_1_Write_Byte(ACCEL_CONFIG_REG, 0x00);
}

void MPU6050_1_Read_All(void) {
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(&hi2c3, MPU6050_1_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 100);

    MPU6050_1.Accel_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]) / 16384.0;
    MPU6050_1.Accel_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]) / 16384.0;
    MPU6050_1.Accel_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]) / 16384.0;
}

uint8_t MPU6050_1_Read_Byte(uint8_t reg) {
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c3, MPU6050_1_ADDR, reg, 1, &data, 1, 1000);
    return data;
}

void alert_user(uint8_t state) {
    // LED ve buzzer kontrolü
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, state ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, state ? GPIO_PIN_SET : GPIO_PIN_RESET); // Buzzer
}

void check_posture(float x, float y) {
    // Tüm yanlış duruşları tek koşulda kontrol et
    uint8_t bad_posture = (y < FORWARD_THRESHOLD) ||   // Öne eğilme
                         (y > BACKWARD_THRESHOLD) ||   // Arkaya yaslanma
                         (x < -SIDE_THRESHOLD) ||      // Sola eğilme
                         (x > SIDE_THRESHOLD);         // Sağa eğilme

    alert_user(bad_posture);
}

void Servo_SetAngle(uint16_t angle) {
    uint16_t pulse = 500 + (angle * 2000) / 180;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}

void DHT11_Start(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DHT11_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_RESET);
    HAL_Delay(18);

    HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = DHT11_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &GPIO_InitStruct);
}

uint8_t DHT11_Check_Response(void) {
    uint32_t timeout = 10000;

    while(HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == GPIO_PIN_SET && timeout--);
    if(timeout == 0) return 0;

    timeout = 10000;
    while(HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == GPIO_PIN_RESET && timeout--);
    if(timeout == 0) return 0;

    return 1;
}

uint8_t DHT11_Read(void) {
    uint8_t value = 0;

    for(int i=0; i<8; i++) {
        while(HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == GPIO_PIN_RESET);

        HAL_Delay_us(40);
        if(HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == GPIO_PIN_SET) {
            value |= (1 << (7-i));
        }

        while(HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == GPIO_PIN_SET);
    }

    return value;
}

void Read_DHT11(float *temperature, float *humidity) {
    uint8_t data[5] = {0};

    DHT11_Start();
    // Sensör yanıt verene kadar bekle (ilk başta uykuda olabilir)
    while (!DHT11_Check_Response()) {
        HAL_Delay(100);  // 100 ms bekle
        DHT11_Start();   // tekrar başlat
    }

    data[0] = DHT11_Read();
    data[1] = DHT11_Read();
    data[2] = DHT11_Read();
    data[3] = DHT11_Read();
    data[4] = DHT11_Read();

    if(data[4] == (data[0] + data[1] + data[2] + data[3])) {
        *humidity = data[0];
        *temperature = data[2];
    }
}

void setServoAngle(uint16_t angle) {
    // Açıyı pulse süresine çevir (500-2500µs aralığı)
    uint16_t pulse = SERVO_MIN + (angle * (SERVO_MAX - SERVO_MIN) / 180);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);  // TIM2_CH4 kullanılıyor
}






/* MPU6050 Implementation */
void MPU6050_2_Init() {
    uint8_t check;
   uint8_t data=0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_2_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_2_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68) {
        data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_2_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
        data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_2_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_2_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Error LED
        while (1);
    }
}

void MPU6050_2_Read_Accel(float *ax, float *ay, float *az) {
    uint8_t recData[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_2_ADDR, ACCEL_XOUT_H_REG, 1, recData, 6, 1000);
    *ax = (int16_t)(recData[0] << 8 | recData[1]) / 16384.0;
    *ay = (int16_t)(recData[2] << 8 | recData[3]) / 16384.0;
    *az = (int16_t)(recData[4] << 8 | recData[5]) / 16384.0;
}

/* Soil Moisture Implementation */
uint32_t Read_Soil_Sensor(void) {
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        sum += HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return sum / 10;
}


/* Trash Bin Implementation */
void enable_DWT() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles);
}

float get_distance_no_timer() {
    uint32_t timeout = 0;
    uint32_t pulse_start = 0, pulse_end = 0;

    // Send TRIG signal
    HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(trig_GPIO_Port,trig_Pin, GPIO_PIN_RESET);

    // Wait for ECHO rising edge
    timeout = 100000;
    while(HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin) == GPIO_PIN_RESET) {
        if(timeout-- == 0) return -1;
        delay_us(1);
    }
    pulse_start = DWT->CYCCNT;

    // Wait for ECHO falling edge
    timeout = 100000;
    while(HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin) == GPIO_PIN_SET) {
        if(timeout-- == 0) return -1;
    }
    pulse_end = DWT->CYCCNT;

    // Calculate distance
    uint32_t pulse_cycles = pulse_end - pulse_start;
    float pulse_us = pulse_cycles / (SystemCoreClock / 1000000.0f);
    return (pulse_us * 0.0343) / 2;
}
uint16_t Read_ADC(ADC_HandleTypeDef* hadc2) {

ADC_ChannelConfTypeDef sConfig = {0};

sConfig.Channel = ADC_CHANNEL_5;

sConfig.Rank = 1;

sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

HAL_ADC_ConfigChannel(hadc2, &sConfig);



HAL_ADC_Start(hadc2);

HAL_ADC_PollForConversion(hadc2, HAL_MAX_DELAY);

return HAL_ADC_GetValue(hadc2);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	  enable_DWT();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // MPU6050 ve filtre başlatma
   memset(filter_buffer, 0, sizeof(filter_buffer));
   MPU6050_1_Init();
   MPU6050_2_Init();
   // PWM başlat
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Servo PWM başlat
   HAL_ADC_Start(&hadc1);                     // ADC başlat
   setServoAngle(90);                         // Başlangıçta musluk açık konum
   Servo_SetAngle(90);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { // MPU6050 işlemleri
      MPU6050_1_Read_All();
      float x = MPU6050_1.Accel_X;
      float y = MPU6050_1.Accel_Y;
      float z = MPU6050_1.Accel_Z;
      apply_filter(&x, &y, &z);
      check_posture(x, y);
      // 2. Gaz Sensörü (ADC_CHANNEL_13)
      HAL_ADC_Stop(&hadc1); // Önce ADC'yi durdur
       ADC_ChannelConfTypeDef sConfig = {0};
       sConfig.Channel = ADC_CHANNEL_13;
       sConfig.Rank = 1;
       sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
       HAL_ADC_ConfigChannel(&hadc1, &sConfig);
       HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
          gas_value = HAL_ADC_GetValue(&hadc1);
          if(gas_value > GAS_THRESHOLD) {
              gas_leak_detected = 1;
              Servo_SetAngle(90); // Pencere açık
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); // BUZZER AÇ
          } else {
              gas_leak_detected = 0;
              Servo_SetAngle(0); // Pencere kapalı
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // BUZZER KAPAT
          }
      }
      HAL_ADC_Stop(&hadc1); // Gaz ölçümü bitti

      // DHT11'den sıcaklık ve nem ölçümü
      Read_DHT11(&temperature, &humidity);

      // LED kontrolü (sıcaklık)
      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

      if(temperature > TEMP_HIGH_THRESHOLD) {
          HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      } else if(temperature < TEMP_LOW_THRESHOLD) {
          HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
      } else {
          HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      }
      // Su seviyesi kontrolü (ADC_CHANNEL_7)
          HAL_ADC_Stop(&hadc1); // Önce ADC'yi durdur
          sConfig.Channel = ADC_CHANNEL_7;
          sConfig.Rank = 1;
          sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
          HAL_ADC_ConfigChannel(&hadc1, &sConfig);
          HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
          uint32_t adcValue = HAL_ADC_GetValue(&hadc1);

          if (adcValue > WATER_THRESHOLD) {
              // Alarm aktif (su var)
              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);  // LED YANIK
              HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET); // Buzzer açık
              setServoAngle(0);
          } else {
              // Normal durum (su yok)
              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);  // LED SÖNÜK
              HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); // Buzzer kapalı
              setServoAngle(90);  // Musluk açık
          }
      }
      HAL_ADC_Stop(&hadc1);

      HAL_Delay(100); // 100ms bekleme



      float ax, ay, az;
      uint32_t soil_moisture;
      MPU6050_2_Read_Accel(&ax, &ay, &az);
      float total_accel = sqrt(ax*ax + ay*ay + az*az);

      if (total_accel > EARTHQUAKE_THRESHOLD) {
          earthquake_detected = 1;
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);  // Earthquake LED
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  // Earthquake Buzzer
      }
      else if (earthquake_detected) {
          earthquake_detected = 0;
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
      }
      HAL_ADC_Stop(&hadc1); // Önce ADC'yi durdur
               sConfig.Channel = ADC_CHANNEL_0;
               sConfig.Rank = 1;
               sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
               HAL_ADC_ConfigChannel(&hadc1, &sConfig);
               HAL_ADC_Start(&hadc1);
      // 2. Soil Moisture Monitoring System
     soil_moisture = Read_Soil_Sensor();
      if (soil_moisture > SOIL_THRESHOLD) {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);   // Moisture LED
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   // Moisture Buzzer
      }
      else {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
      }

      // 3. Trash Bin Level Detection System
      float distance = get_distance_no_timer();
      if(distance > 0 && distance < FULL_DISTANCE) {
          HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);  // Trash LED
          HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET); // Trash Buzzer
          HAL_Delay(500);
          HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
          HAL_Delay(500);
      }
      else {
          HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
      }

      // 4. Lighting Control System
      uint16_t ldrValue = Read_ADC(&hadc2); // LDR değerini oku
      uint8_t currentPIRState = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14); // PIR durumunu oku
      uint32_t currentTime = HAL_GetTick();

      // DEBUG için seri port çıktısı (HAL_UART kullanarak)
      // printf("LDR: %d, PIR: %d\n", ldrValue, currentPIRState);

      // SADECE karanlıkta VE hareket varsa LED'i yak
      if (ldrValue < LDR_THRESHOLD && currentPIRState == GPIO_PIN_SET) {
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET); // LED ON
          motionDetectedTime = currentTime; // Zamanı güncelle
      }
      // Diğer tüm durumlarda LED'i söndür
      else {
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET); // LED OFF
      }
      // Timeout kontrolü (opsiyonel, 15 saniye sonra LED'i söndürür)
      if (currentTime - motionDetectedTime >= TIMEOUT_DURATION) {
          HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
      }
      HAL_Delay(100); // Common delay for all systems

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_BLUE_Pin|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15|DHT11_DATA_Pin|GPIO_PIN_4
                          |GPIO_PIN_5|trig_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|led_Pin|buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_BLUE_Pin PE13 */
  GPIO_InitStruct.Pin = LED_BLUE_Pin|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 PC4 PC5
                           trig_Pin PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5
                          |trig_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_DATA_Pin */
  GPIO_InitStruct.Pin = DHT11_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 led_Pin buzzer_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|led_Pin|buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : echo_Pin */
  GPIO_InitStruct.Pin = echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
