#define MICROPY_HW_BOARD_NAME               "01Studio PYDRONE-ESP32-S3P"
#define MICROPY_HW_MCU_NAME                 "ESP32S3"

#define MICROPY_PY_MACHINE_DAC			(0)

#define MICROPY_ENABLE_ESP_DRONE		(1)

// 主传感器I2C总线(MPU6050, HMC5883L, SPL06)
#define MICROPY_HW_SENSOR_I2C_PIN_SCL	(15)
#define MICROPY_HW_SENSOR_I2C_PIN_SDA	(16)

// TOF传感器I2C总线(VL53L0X/VL53L1X) - 匹配根目录ESP32配置
#define MICROPY_HW_TOF_I2C_PIN_SCL		(39)
#define MICROPY_HW_TOF_I2C_PIN_SDA		(38)
#define MICROPY_MPU_PIN_IRQ				(7)

// 扩展I2C总线
#define MICROPY_HW_DECK_I2C_PIN_SCL		(1)
#define MICROPY_HW_DECK_I2C_PIN_SDA		(6)

// 光流传感器SPI接口(PMW3901) - 匹配根目录ESP32配置
#define MICROPY_HW_OPTICAL_FLOW_SPI_SCK	(18)
#define MICROPY_HW_OPTICAL_FLOW_SPI_MOSI	(21)
#define MICROPY_HW_OPTICAL_FLOW_SPI_MISO	(17)
#define MICROPY_HW_OPTICAL_FLOW_SPI_CS	(8)
#define MICROPY_HW_OPTICAL_FLOW_LED_PIN	(47)

#define MICROPY_MOTOR01_PIN				(4)
#define MICROPY_MOTOR02_PIN				(5)
#define MICROPY_MOTOR03_PIN				(40)
#define MICROPY_MOTOR04_PIN				(41)

#define MICROPY_PY_PICLIB				(1)

//sensor
#define MICROPY_ENABLE_SENSOR			(1)
#define MICROPY_HW_OV2640				(1)

//WEB stream
#define MICROPY_ENABLE_STREAM			(1) 
//CAM
#define CAM_PIN_XCLK					(11)
#define CAM_PIN_SIOD					(17)
#define CAM_PIN_SIOC					(18)

#define CAM_PIN_D7						(10)
#define CAM_PIN_D6						(12)
#define CAM_PIN_D5						(13)
#define CAM_PIN_D4						(21)
#define CAM_PIN_D3						(48)
#define CAM_PIN_D2						(39)
#define CAM_PIN_D1						(38)
#define CAM_PIN_D0						(47)
#define CAM_PIN_VSYNC					(8)
#define CAM_PIN_HREF					(9)
#define CAM_PIN_PCLK					(14)

#ifndef CONFIG_SCCB_HARDWARE_I2C_PORT1
#define CONFIG_SCCB_HARDWARE_I2C_PORT1	(1)
#endif
#ifndef CONFIG_OV2640_SUPPORT
#define CONFIG_OV2640_SUPPORT			(1)
#endif
