/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ICM-20602 gyro calibration + roll/pitch
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* IMU status */
volatile uint8_t imu_who_am_i = 0;
volatile uint8_t imu_init_ok = 0;
volatile uint8_t gyro_calibrated = 0;
volatile HAL_StatusTypeDef imu_spi_status = HAL_ERROR;

/* Raw accelerometer */
volatile int16_t accel_x_raw = 0;
volatile int16_t accel_y_raw = 0;
volatile int16_t accel_z_raw = 0;

/* Raw gyro */
volatile int16_t gyro_x_raw = 0;
volatile int16_t gyro_y_raw = 0;
volatile int16_t gyro_z_raw = 0;

/* Temperature */
volatile int16_t temp_raw = 0;

/* Accelerometer values in g */
volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;

/* Raw converted gyro values in degrees/sec */
volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;

/* Gyro calibration offsets */
volatile float gyro_x_bias = 0.0f;
volatile float gyro_y_bias = 0.0f;
volatile float gyro_z_bias = 0.0f;

/* Bias-corrected gyro */
volatile float gyro_x_corrected = 0.0f;
volatile float gyro_y_corrected = 0.0f;
volatile float gyro_z_corrected = 0.0f;

/* Accelerometer-only angles */
volatile float accel_roll_deg = 0.0f;
volatile float accel_pitch_deg = 0.0f;

/* Final complementary-filter angles */
volatile float roll_deg = 0.0f;
volatile float pitch_deg = 0.0f;

/* Timing */
volatile float loop_dt = 0.01f;
uint32_t previous_time_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */

uint8_t ICM20602_ReadRegister(uint8_t reg);

void ICM20602_ReadRegisters(uint8_t reg,
                            uint8_t *data,
                            uint8_t length);

void ICM20602_WriteRegister(uint8_t reg,
                            uint8_t value);

uint8_t ICM20602_Init(void);

void ICM20602_ReadSensor(void);

void ICM20602_CalibrateGyro(void);

void CalculateAccelAngles(void);

void InitializeAttitude(void);

void UpdateAttitude(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*
 * ICM-20602 register addresses
 */
#define ICM20602_WHO_AM_I       0x75
#define ICM20602_PWR_MGMT_1     0x6B
#define ICM20602_SMPLRT_DIV     0x19
#define ICM20602_CONFIG         0x1A
#define ICM20602_GYRO_CONFIG    0x1B
#define ICM20602_ACCEL_CONFIG   0x1C
#define ICM20602_ACCEL_CONFIG2  0x1D
#define ICM20602_ACCEL_XOUT_H   0x3B

#define RAD_TO_DEG 57.2957795f

/*
 * Complementary filter:
 *
 * Higher value = trust gyro more
 * Lower value  = trust accelerometer more
 */
#define COMPLEMENTARY_ALPHA 0.98f


/* ============================================================
   WRITE ONE REGISTER
   ============================================================ */

void ICM20602_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    /*
     * Bit 7 = 0 means write.
     */
    tx[0] = reg & 0x7F;
    tx[1] = value;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET);

    imu_spi_status =
        HAL_SPI_Transmit(&hspi1,
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

    /*
     * Bit 7 = 1 means read.
     */
    tx[0] = reg | 0x80;
    tx[1] = 0x00;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET);

    imu_spi_status =
        HAL_SPI_TransmitReceive(&hspi1,
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
    uint8_t address;
    uint8_t dummy[14] = {0};

    /*
     * This function currently supports
     * up to 14 bytes because that is all
     * the sensor data we need here.
     */
    if (length > 14)
    {
        imu_spi_status = HAL_ERROR;
        return;
    }

    address = reg | 0x80;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET);

    /*
     * Send register address.
     */
    imu_spi_status =
        HAL_SPI_Transmit(&hspi1,
                         &address,
                         1,
                         100);

    /*
     * Only continue if sending the address worked.
     */
    if (imu_spi_status == HAL_OK)
    {
        /*
         * Send dummy bytes while
         * clocking sensor data back.
         */
        imu_spi_status =
            HAL_SPI_TransmitReceive(&hspi1,
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
    uint8_t verify;

    HAL_Delay(100);

    /*
     * Check WHO_AM_I.
     *
     * Expected ICM-20602 ID = 0x12
     */
    imu_who_am_i =
        ICM20602_ReadRegister(ICM20602_WHO_AM_I);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    if (imu_who_am_i != 0x12)
    {
        return 0;
    }


    /*
     * Reset device.
     */
    ICM20602_WriteRegister(ICM20602_PWR_MGMT_1,
                           0x80);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    HAL_Delay(100);


    /*
     * Wake device.
     *
     * Clock source = gyro PLL.
     */
    ICM20602_WriteRegister(ICM20602_PWR_MGMT_1,
                           0x01);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    HAL_Delay(10);


    /*
     * Sample rate.
     *
     * With DLPF enabled:
     *
     * Base rate = 1000 Hz
     *
     * SMPLRT_DIV = 0
     *
     * Sample rate =
     * 1000 / (0 + 1)
     * = 1000 Hz
     */
    ICM20602_WriteRegister(ICM20602_SMPLRT_DIV,
                           0x00);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }


    /*
     * Gyroscope digital low-pass filter.
     *
     * CONFIG = 0x03
     */
    ICM20602_WriteRegister(ICM20602_CONFIG,
                           0x03);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }


    /*
     * Gyroscope range.
     *
     * 0x00 = +/-250 deg/s
     */
    ICM20602_WriteRegister(ICM20602_GYRO_CONFIG,
                           0x00);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }


    /*
     * Accelerometer range.
     *
     * 0x00 = +/-2 g
     */
    ICM20602_WriteRegister(ICM20602_ACCEL_CONFIG,
                           0x00);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }


    /*
     * Accelerometer digital low-pass filter.
     *
     * ACCEL_CONFIG2 = 0x03
     */
    ICM20602_WriteRegister(ICM20602_ACCEL_CONFIG2,
                           0x03);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    HAL_Delay(10);


    /* ========================================================
       VERIFY CONFIGURATION
       ======================================================== */


    /*
     * Verify sample rate.
     */
    verify =
        ICM20602_ReadRegister(ICM20602_SMPLRT_DIV);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    if (verify != 0x00)
    {
        return 0;
    }


    /*
     * Verify gyro DLPF.
     */
    verify =
        ICM20602_ReadRegister(ICM20602_CONFIG);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    if (verify != 0x03)
    {
        return 0;
    }


    /*
     * Verify gyro range.
     */
    verify =
        ICM20602_ReadRegister(ICM20602_GYRO_CONFIG);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    if (verify != 0x00)
    {
        return 0;
    }


    /*
     * Verify accelerometer range.
     */
    verify =
        ICM20602_ReadRegister(ICM20602_ACCEL_CONFIG);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    if (verify != 0x00)
    {
        return 0;
    }


    /*
     * Verify accelerometer DLPF.
     */
    verify =
        ICM20602_ReadRegister(ICM20602_ACCEL_CONFIG2);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    if (verify != 0x03)
    {
        return 0;
    }


    /*
     * Check WHO_AM_I one final time.
     */
    imu_who_am_i =
        ICM20602_ReadRegister(ICM20602_WHO_AM_I);

    if (imu_spi_status != HAL_OK)
    {
        return 0;
    }

    if (imu_who_am_i != 0x12)
    {
        return 0;
    }

    return 1;
}


/* ============================================================
   READ SENSOR
   ============================================================ */

void ICM20602_ReadSensor(void)
{
    uint8_t data[14] = {0};

    /*
     * Start at ACCEL_XOUT_H = 0x3B.
     *
     * Read:
     *
     * accel X/Y/Z
     * temperature
     * gyro X/Y/Z
     */
    ICM20602_ReadRegisters(ICM20602_ACCEL_XOUT_H,
                           data,
                           14);

    /*
     * Do not update the sensor values
     * if the SPI transaction failed.
     */
    if (imu_spi_status != HAL_OK)
    {
        return;
    }


    /* Accelerometer raw values */

    accel_x_raw =
        (int16_t)(((uint16_t)data[0] << 8) |
                  data[1]);

    accel_y_raw =
        (int16_t)(((uint16_t)data[2] << 8) |
                  data[3]);

    accel_z_raw =
        (int16_t)(((uint16_t)data[4] << 8) |
                  data[5]);


    /* Temperature */

    temp_raw =
        (int16_t)(((uint16_t)data[6] << 8) |
                  data[7]);


    /* Gyroscope raw values */

    gyro_x_raw =
        (int16_t)(((uint16_t)data[8] << 8) |
                  data[9]);

    gyro_y_raw =
        (int16_t)(((uint16_t)data[10] << 8) |
                  data[11]);

    gyro_z_raw =
        (int16_t)(((uint16_t)data[12] << 8) |
                  data[13]);


    /*
     * Accelerometer:
     *
     * Range = +/-2 g
     * Sensitivity = 16384 counts/g
     */
    accel_x_g =
        (float)accel_x_raw / 16384.0f;

    accel_y_g =
        (float)accel_y_raw / 16384.0f;

    accel_z_g =
        (float)accel_z_raw / 16384.0f;


    /*
     * Gyroscope:
     *
     * Range = +/-250 deg/s
     * Sensitivity = 131 counts/(deg/s)
     */
    gyro_x_dps =
        (float)gyro_x_raw / 131.0f;

    gyro_y_dps =
        (float)gyro_y_raw / 131.0f;

    gyro_z_dps =
        (float)gyro_z_raw / 131.0f;
}


/* ============================================================
   GYRO CALIBRATION
   ============================================================ */

void ICM20602_CalibrateGyro(void)
{
    float x_sum = 0.0f;
    float y_sum = 0.0f;
    float z_sum = 0.0f;

    const uint16_t samples = 1000;

    gyro_calibrated = 0;

    /*
     * IMPORTANT:
     *
     * Keep the IMU completely still
     * while this runs.
     *
     * 1000 samples x 2 ms ≈ 2 seconds.
     */
    for (uint16_t i = 0; i < samples; i++)
    {
        ICM20602_ReadSensor();

        /*
         * If communication fails,
         * stop calibration.
         */
        if (imu_spi_status != HAL_OK)
        {
            return;
        }

        x_sum += gyro_x_dps;
        y_sum += gyro_y_dps;
        z_sum += gyro_z_dps;

        HAL_Delay(2);
    }


    /*
     * Average the stationary readings.
     *
     * These become the gyro biases.
     */
    gyro_x_bias =
        x_sum / (float)samples;

    gyro_y_bias =
        y_sum / (float)samples;

    gyro_z_bias =
        z_sum / (float)samples;

    gyro_calibrated = 1;
}


/* ============================================================
   ACCELEROMETER ANGLES
   ============================================================ */

void CalculateAccelAngles(void)
{
    /*
     * Roll:
     *
     * Rotation around X axis.
     */
    accel_roll_deg =
        atan2f(accel_y_g,
               accel_z_g)
        * RAD_TO_DEG;


    /*
     * Pitch:
     *
     * Rotation around Y axis.
     */
    accel_pitch_deg =
        atan2f(-accel_x_g,
               sqrtf((accel_y_g * accel_y_g) +
                     (accel_z_g * accel_z_g)))
        * RAD_TO_DEG;
}


/* ============================================================
   INITIAL ANGLE
   ============================================================ */

void InitializeAttitude(void)
{
    ICM20602_ReadSensor();

    if (imu_spi_status != HAL_OK)
    {
        return;
    }

    CalculateAccelAngles();

    /*
     * Start complementary filter using the
     * accelerometer's initial orientation.
     */
    roll_deg =
        accel_roll_deg;

    pitch_deg =
        accel_pitch_deg;

    previous_time_ms =
        HAL_GetTick();
}


/* ============================================================
   COMPLEMENTARY FILTER
   ============================================================ */

void UpdateAttitude(void)
{
    uint32_t current_time_ms;

    float gyro_roll;
    float gyro_pitch;


    /*
     * Read latest sensor values.
     */
    ICM20602_ReadSensor();

    if (imu_spi_status != HAL_OK)
    {
        return;
    }


    /*
     * Remove stationary gyro offsets.
     */
    gyro_x_corrected =
        gyro_x_dps -
        gyro_x_bias;

    gyro_y_corrected =
        gyro_y_dps -
        gyro_y_bias;

    gyro_z_corrected =
        gyro_z_dps -
        gyro_z_bias;


    /*
     * Get loop time.
     */
    current_time_ms =
        HAL_GetTick();

    loop_dt =
        (float)(current_time_ms -
                previous_time_ms)
        / 1000.0f;

    previous_time_ms =
        current_time_ms;


    /*
     * Protect against a weird dt while debugging.
     */
    if ((loop_dt <= 0.0f) ||
        (loop_dt > 0.1f))
    {
        loop_dt = 0.01f;
    }


    /*
     * Calculate angle based only on gravity.
     */
    CalculateAccelAngles();


    /*
     * Integrate gyro angular velocity.
     */
    gyro_roll =
        roll_deg +
        (gyro_x_corrected * loop_dt);

    gyro_pitch =
        pitch_deg +
        (gyro_y_corrected * loop_dt);


    /*
     * Complementary filter.
     *
     * 98% gyro
     * 2% accelerometer
     */
    roll_deg =
        (COMPLEMENTARY_ALPHA *
         gyro_roll)
        +
        ((1.0f - COMPLEMENTARY_ALPHA) *
         accel_roll_deg);

    pitch_deg =
        (COMPLEMENTARY_ALPHA *
         gyro_pitch)
        +
        ((1.0f - COMPLEMENTARY_ALPHA) *
         accel_pitch_deg);
}


/* USER CODE END 0 */


/* ============================================================
   MAIN
   ============================================================ */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_SPI1_Init();


    /* USER CODE BEGIN 2 */

    /*
     * Make sure IMU is deselected initially.
     */
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET);


    /*
     * Initialize sensor.
     *
     * This now also verifies:
     *
     * sample rate
     * gyro DLPF
     * gyro range
     * accel range
     * accel DLPF
     * WHO_AM_I
     */
    imu_init_ok =
        ICM20602_Init();


    if (imu_init_ok)
    {
        /*
         * IMPORTANT:
         *
         * Don't move the sensor during
         * gyro calibration.
         *
         * Takes about 2 seconds.
         */
        ICM20602_CalibrateGyro();


        /*
         * Establish initial roll/pitch.
         */
        if (gyro_calibrated)
        {
            InitializeAttitude();
        }
    }


    /* USER CODE END 2 */


    while (1)
    {
        /* USER CODE BEGIN 3 */

        if (imu_init_ok &&
            gyro_calibrated)
        {
            /*
             * Update roll and pitch.
             */
            UpdateAttitude();
        }
        else
        {
            /*
             * Retry WHO_AM_I if initialization
             * or calibration failed.
             */
            imu_who_am_i =
                ICM20602_ReadRegister(
                    ICM20602_WHO_AM_I
                );
        }


        /*
         * Approximately 100 Hz loop
         * for development.
         *
         * Later this should use a timer
         * instead of HAL_Delay().
         */
        HAL_Delay(10);


        /* USER CODE END 3 */
    }
}


/* ============================================================
   CLOCK CONFIG
   ============================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );


    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_NONE;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }


    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_0)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
   SPI1 CONFIG
   ============================================================ */

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;

    hspi1.Init.Mode =
        SPI_MODE_MASTER;

    hspi1.Init.Direction =
        SPI_DIRECTION_2LINES;

    hspi1.Init.DataSize =
        SPI_DATASIZE_8BIT;

    /*
     * ICM-20602 SPI mode:
     *
     * CPOL = 0
     * CPHA = 0
     */
    hspi1.Init.CLKPolarity =
        SPI_POLARITY_LOW;

    hspi1.Init.CLKPhase =
        SPI_PHASE_1EDGE;

    hspi1.Init.NSS =
        SPI_NSS_SOFT;


    /*
     * 16 MHz / 16 = 1 MHz
     */
    hspi1.Init.BaudRatePrescaler =
        SPI_BAUDRATEPRESCALER_16;

    hspi1.Init.FirstBit =
        SPI_FIRSTBIT_MSB;

    hspi1.Init.TIMode =
        SPI_TIMODE_DISABLE;

    hspi1.Init.CRCCalculation =
        SPI_CRCCALCULATION_DISABLE;

    hspi1.Init.CRCPolynomial = 10;


    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
   GPIO CONFIG
   ============================================================ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();


    /*
     * CS inactive = HIGH.
     */
    HAL_GPIO_WritePin(
        IMU_CS_GPIO_Port,
        IMU_CS_Pin,
        GPIO_PIN_SET
    );


    GPIO_InitStruct.Pin =
        IMU_CS_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        IMU_CS_GPIO_Port,
        &GPIO_InitStruct
    );
}


/* ============================================================
   ERROR HANDLER
   ============================================================ */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file,
                   uint32_t line)
{
}

#endif
