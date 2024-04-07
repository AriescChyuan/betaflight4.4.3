/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// #define TARGET_BOARD_IDENTIFIER "MB41"
// #define USBD_PRODUCT_STRING     "MAMBA411"

#define SYSTEM_HSE_MHZ 25

#define TARGET_BOARD_IDENTIFIER "TT411"
#define USBD_PRODUCT_STRING     "ThunderTigerF411"

// ******* LEDs and BEEPER ********

#define LED0_PIN                PC13
#define LED1_PIN                PC14

#define USE_BEEPER
#define BEEPER_PIN              PB2
#define BEEPER_INVERTED


// ******* GYRO and ACC ********

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA1
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define GYRO_1_ALIGN            CW180_DEG
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500

// *************** Baro **************************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8        // SCL pad
#define I2C1_SDA                PB9        // SDA pad
// #define USE_I2C_PULLUP
#define BARO_I2C_INSTANCE       (I2CDEV_1)

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_BMP085
#define USE_BARO_MS5611
#define USE_BARO_QMP6988
// #define DEFAULT_BARO_BMP280

// ******* MAG ********

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL
#define MAG_I2C_INSTANCE        (I2CDEV_1)

// ******* OSD ********

// #define USE_MAX7456
// #define MAX7456_SPI_INSTANCE    SPI2
// #define MAX7456_SPI_CS_PIN      PB12

// ******* FLASH  ************************** 

#define USE_FLASHFS
#define USE_FLASH_TOOLS
// #define USE_FLASH_M25P16
// #define USE_FLASH_W25N01G          // 1Gb NAND flash support
// #define USE_FLASH_W25M             // Stacked die support
// #define USE_FLASH_W25M512          // 512Kb (256Kb x 2 stacked) NOR flash support
// #define USE_FLASH_W25M02G          // 2Gb (1Gb x 2 stacked) NAND flash support
#define USE_FLASH_W25Q128FV        // 16MB Winbond 25Q128 iFlight_Beast_H7_55A_V1 version uses 16Mbit Winbond W25Q128FV Flash
#define FLASH_CS_PIN            PB12
#define FLASH_SPI_INSTANCE      SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// ******* SERIAL(UART)********
#define USE_VCP
#define USB_DETECT_PIN          PC15
#define USE_USB_DETECT

#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART2, SOFTSERIAL x 2

#define USE_ESCSERIAL
// #define ESCSERIAL_TIMER_TX_PIN  PB7  // (HARDARE=0,PPM)

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1

// #define INVERTER_PIN_UART1      PB10

// ******* SPI ********

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            GYRO_1_CS_PIN
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            FLASH_CS_PIN
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

// ******* ADC ********

#define USE_ADC

#define ADC_INSTANCE            ADC1  // Default added
#define ADC1_DMA_OPT            0     // DMA 2 Stream 0 Channel 0
#define VBAT_ADC_PIN            PB0
// #define RSSI_ADC_PIN            PA0
#define CURRENT_METER_ADC_PIN   PB1
// #define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
// #define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

//  ******* ESC ********

#define USE_DSHOT
#define USE_DSHOT_DMAR
#define USE_DSHOT_TELEMETRY
#define USE_DSHOT_BITBAND
#define USE_DSHOT_TELEMETRY_STATS
#define USE_BRUSHED_ESC_AUTODETECT 
#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON
#define DSHOT_BITBANG_DEFAULT   DSHOT_BITBANG_AUTO

// ******* FEATURES ********

// #define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
// #define SERIALRX_UART           SERIAL_PORT_USART1
// #define SERIALRX_PROVIDER       SERIALRX_SBUS

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL)
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS            ( TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(5)|TIM_N(9) )






// ================== T E S T ===========================
/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// #pragma once
// #define TARGET_BOARD_IDENTIFIER "S411"
// #define USBD_PRODUCT_STRING     "ThunderTigerF411"

// #define FC_TARGET_MCU     STM32F411

// #define BOARD_NAME        ThunderTigerF411
// #define MANUFACTURER_ID   ThunderTige

// #define USE_GYRO
// #define USE_GYRO_SPI_MPU6500
// #define USE_GYRO_SPI_MPU6000
// #define USE_ACC
// #define USE_ACC_SPI_MPU6500
// #define USE_ACC_SPI_MPU6000
// #define USE_BARO
// #define USE_BARO_BMP280
// #define USE_BARO_BMP085
// #define USE_BARO_MS5611
// #define USE_BARO_QMP6988

// #define USE_MAX7456
// #define USE_FLASH_CHIP
// #define USE_FLASH_SPI
// #define USE_FLASHFS
// #define USE_FLASH_TOOLS
// #define USE_FLASH_M25P16
// #define USE_FLASH_W25N01G          // 1Gb NAND flash support
// #define USE_FLASH_W25M             // Stacked die support
// #define USE_FLASH_W25M512          // 512Kb (256Kb x 2 stacked) NOR flash support
// #define USE_FLASH_W25M02G          // 2Gb (1Gb x 2 stacked) NAND flash support
// #define USE_FLASH_W25Q128FV
// #define USE_DSHOT_BITBAND

// #define BEEPER_PIN           PB2
// #define MOTOR1_PIN           PB4
// #define MOTOR2_PIN           PB5
// #define MOTOR3_PIN           PB6
// #define MOTOR4_PIN           PB7
// #define MOTOR5_PIN           PB3
// #define MOTOR6_PIN           PB10
// #define RX_PPM_PIN           PA3
// #define LED_STRIP_PIN        PA8
// #define UART1_TX_PIN         PA9
// #define UART2_TX_PIN         PA2
// #define UART1_RX_PIN         PA10
// #define UART2_RX_PIN         PA3
// #define I2C1_SCL_PIN         PB8
// #define I2C1_SDA_PIN         PB9
// #define LED0_PIN             PC13
// #define LED1_PIN             PC14
// #define SPI1_SCK_PIN         PA5
// #define SPI2_SCK_PIN         PB13
// #define SPI1_SDI_PIN         PA6
// #define SPI2_SDI_PIN         PB14
// #define SPI1_SDO_PIN         PA7
// #define SPI2_SDO_PIN         PB15
// #define ADC_VBAT_PIN         PB0
// #define ADC_CURR_PIN         PB1
// // #define MAX7456_SPI_CS_PIN   PB12
// #define FLASH_CS_PIN         PB12
// #define GYRO_1_EXTI_PIN      PA1
// #define GYRO_2_EXTI_PIN      NONE
// #define GYRO_1_CS_PIN        PA4
// #define USB_DETECT_PIN       PC15

// #define TIMER_PIN_MAPPING \
//     TIMER_PIN_MAP( 0, PA3 , 3, -1) \
//     TIMER_PIN_MAP( 1, PB4 , 1,  0) \
//     TIMER_PIN_MAP( 2, PB5 , 1,  0) \
//     TIMER_PIN_MAP( 3, PB6 , 1,  0) \
//     TIMER_PIN_MAP( 4, PB7 , 1,  0) \
//     TIMER_PIN_MAP( 5, PB3 , 1,  0) \
//     TIMER_PIN_MAP( 6, PB10, 1,  0) \
//     TIMER_PIN_MAP( 7, PA0 , 2,  0) \
//     TIMER_PIN_MAP( 8, PA2 , 2,  0) \
//     TIMER_PIN_MAP( 9, PA8 , 1,  0)


// #define ADC1_DMA_OPT        1

// #define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
// #define DEFAULT_DSHOT_BURST DSHOT_DMAR_AUTO
// #define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_OFF
// #define MAG_I2C_INSTANCE (I2CDEV_1)
// #define USE_BARO
// #define BARO_I2C_INSTANCE (I2CDEV_1)
// #define DEFAULT_BARO_DEVICE BARO_NONE
// #define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
// #define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
// //TODO #define VBAT_DETECT_CELL_VOLTAGE 300
// #define BEEPER_INVERTED
// #define SYSTEM_HSE_MHZ 25
// // #define MAX7456_SPI_INSTANCE SPI2
// #define FLASH_SPI_INSTANCE      SPI2
// #define GYRO_1_SPI_INSTANCE SPI1
// #define GYRO_1_ALIGN CW180_DEG

// #define TARGET_IO_PORTA         0xffff
// #define TARGET_IO_PORTB         0xffff
// #define TARGET_IO_PORTC         0xffff
// #define TARGET_IO_PORTD         (BIT(2))

// #define USE_VCP
// #define USE_SOFTSERIAL1
// #define USE_SOFTSERIAL2

// #define SERIAL_PORT_COUNT       5 //VCP, USART1, USART2, SOFTSERIAL x 2

// #define USABLE_TIMER_CHANNEL_COUNT 10
// #define USED_TIMERS            ( TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(5)|TIM_N(9) )