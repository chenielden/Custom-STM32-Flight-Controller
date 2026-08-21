/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ICM-20602 accelerometer + gyroscope SPI test
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* IMU identification / status */
volatile uint8_t imu_who_am_i = 0;
volatile uint8_t imu_init_ok = 0;
volatile HAL_StatusTypeDef imu_spi_status = HAL_ERROR;

/* Raw accelerometer */
volatile int16_t accel_x_raw = 0;
volatile int16_t accel_y_raw = 0;
volatile int16_t accel_z_raw = 0;

/* Raw gyroscope */
volatile int16_t gyro_x_raw = 0;
volatile int16_t gyro_y_raw = 0;
volatile int16_t gyro_z_raw = 0;

/* Temperature */
volatile int16_t temp_raw = 0;

/* Accelerometer in g */
volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;

/* Gyroscope in degrees/second */
volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */

uint8_t ICM20602_ReadRegister(uint8_t reg);
void ICM20602_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length);
void ICM20602_WriteRegister(uint8_t reg, uint8_t value);
uint8_t ICM20602_Init(void);
void ICM20602_ReadSensor(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* ============================================================
   WRITE ONE REGISTER
   ============================================================ */

void ICM20602_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    tx[0] = reg & 0x7F;
    tx[1] = value;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET);

    imu_spi_status = HAL_SPI_Transmit(&hspi1,
                                      tx,
                                      2,
                                      100);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET);
}


/* ============================================================
   READ ONE REGISTER
   ============================================================ */

uint8_t ICM20602_ReadRegister(uint8_t reg)
{
    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};

    tx[0] = reg | 0x80;
    tx[1] = 0x00;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET);

    imu_spi_status = HAL_SPI_TransmitReceive(&hspi1,
                                             tx,
                                             rx,
                                             2,
                                             100);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET);

    return rx[1];
}


/* ============================================================
   READ MULTIPLE REGISTERS
   ============================================================ */

void ICM20602_ReadRegisters(uint8_t reg,
                            uint8_t *data,
                            uint8_t length)
{
    uint8_t address = reg | 0x80;
    uint8_t dummy[14] = {0};

    if (length > 14)
    {
        return;
    }

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET);

    imu_spi_status = HAL_SPI_Transmit(&hspi1,
                                      &address,
                                      1,
                                      100);

    if (imu_spi_status == HAL_OK)
    {

        imu_spi_status = HAL_SPI_TransmitReceive(&hspi1,
                                                 dummy,
                                                 data,
                                                 length,
                                                 100);
    }

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET);
}


/* ============================================================
   INITIALIZE ICM-20602
   ============================================================ */

uint8_t ICM20602_Init(void)
{
    HAL_Delay(100);

    imu_who_am_i = ICM20602_ReadRegister(0x75);

    if (imu_who_am_i != 0x12)
    {
        return 0;
    }

    ICM20602_WriteRegister(0x6B, 0x80);
    HAL_Delay(100);

    ICM20602_WriteRegister(0x6B, 0x01);
    HAL_Delay(10);

    ICM20602_WriteRegister(0x1A, 0x03);

    ICM20602_WriteRegister(0x1B, 0x00);

    ICM20602_WriteRegister(0x1C, 0x00);

    HAL_Delay(10);

    /*
     * Check the ID again after reset/configuration.
     */
    imu_who_am_i = ICM20602_ReadRegister(0x75);

    if (imu_who_am_i != 0x12)
    {
        return 0;
    }

    return 1;
}


/* ============================================================
   READ ACCEL + TEMPERATURE + GYRO
   ============================================================ */

void ICM20602_ReadSensor(void)
{
    uint8_t data[14] = {0};

    ICM20602_ReadRegisters(0x3B, data, 14);

    if (imu_spi_status != HAL_OK)
    {
        return;
    }

    /* Accelerometer */
    accel_x_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    accel_y_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    accel_z_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    /* Temperature */
    temp_raw = (int16_t)(((uint16_t)data[6] << 8) | data[7]);

    /* Gyroscope */
    gyro_x_raw = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
    gyro_y_raw = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
    gyro_z_raw = (int16_t)(((uint16_t)data[12] << 8) | data[13]);

    /*
     * +/-2 g = 16384 LSB/g
     */
    accel_x_g = (float)accel_x_raw / 16384.0f;
    accel_y_g = (float)accel_y_raw / 16384.0f;
    accel_z_g = (float)accel_z_raw / 16384.0f;

    /*
     * +/-250 deg/s = 131 LSB/(deg/s)
     */
    gyro_x_dps = (float)gyro_x_raw / 131.0f;
    gyro_y_dps = (float)gyro_y_raw / 131.0f;
    gyro_z_dps = (float)gyro_z_raw / 131.0f;
}

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI1_Init();

    /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET);

    /*
     * Initialize ICM-20602.
     *
     * Success = 1
     * Failure = 0
     */
    imu_init_ok = ICM20602_Init();

    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */

        if (imu_init_ok)
        {
            ICM20602_ReadSensor();
        }
        else
        {

            imu_who_am_i = ICM20602_ReadRegister(0x75);
        }

        HAL_Delay(10);

        /* USER CODE END 3 */
    }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief SPI1 Initialization Function
  * @retval None
  */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET);

    GPIO_InitStruct.Pin = IMU_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(IMU_CS_GPIO_Port,
                  &GPIO_InitStruct);
}


/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}

#endif
