/**
 * @file bflb_gpio.h
 * @author WangChong
 * @brief  中文翻译版的BL616的GPIO使用说明，基于chart-GPT和人工核对, 请以官方文档为主
 * @version 0.1
 * @date 2023-10-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _BFLB_GPIO_H
#define _BFLB_GPIO_H

#include "bflb_core.h"

/** @addtogroup LHAL
  * @{
  */

/** @addtogroup GPIO
  * @{
  */

/**
 * @brief GPIO 定义，根据不同的芯片选择不同的GPIO区间
 *
 * BL602  : GPIO0/1/2/7/8/14/15/20/21/22
 * BL604  : GPIO0 ~ GPIO5, GPIO7/8/11/12/14/16/17/20/21/22
 * BL606  : GPIO0 ~ GPIO22
 * BL702  : GPIO0/1/2/7/8/9/14/15/17/23/24/25/26/27/28
 * BL704  : GPIO0 ~ GPIO3, GPIO7 ~ GPIO11, GPIO14/15, GPIO17 ~ GPIO28,
 * BL706  : GPIO0 ~ GPIO31
 * BL606P : GPIO0 ~ GPIO5, GPIO11 ~ GPIO12, GPIO16 ~ GPIO21, GPIO24 ~ GPIO28, GPIO34 ~ GPIO41
 * BL616  : GPIO0 ~ GPIO3, GPIO10 ~ GPIO17, GPIO20 ~ GPIO22, GPIO27 ~ GPIO30
 * BL618  : GPIO0 ~ GPIO34
 * BL808C : GPIO0 ~ GPIO23, GPIO34 ~ GPIO45
 * BL808D : GPIO0 ~ GPIO8, GPIO11 ~ GPIO41
 *
 */

/** @defgroup GPIO_PIN GPIO引脚定义
  * @{
  * @brief 所有芯片系列的GPIO定义，不一定全部可以，请根据上方芯片选择
  */
#define GPIO_PIN_0      0
#define GPIO_PIN_1      1
#define GPIO_PIN_2      2
#define GPIO_PIN_3      3
#define GPIO_PIN_4      4
#define GPIO_PIN_5      5
#define GPIO_PIN_6      6
#define GPIO_PIN_7      7
#define GPIO_PIN_8      8
#define GPIO_PIN_9      9
#define GPIO_PIN_10     10
#define GPIO_PIN_11     11
#define GPIO_PIN_12     12
#define GPIO_PIN_13     13
#define GPIO_PIN_14     14
#define GPIO_PIN_15     15
#define GPIO_PIN_16     16
#define GPIO_PIN_17     17
#define GPIO_PIN_18     18
#define GPIO_PIN_19     19
#define GPIO_PIN_20     20
#define GPIO_PIN_21     21
#define GPIO_PIN_22     22
#define GPIO_PIN_23     23
#define GPIO_PIN_24     24
#define GPIO_PIN_25     25
#define GPIO_PIN_26     26
#define GPIO_PIN_27     27
#define GPIO_PIN_28     28
#define GPIO_PIN_29     29
#define GPIO_PIN_30     30
#define GPIO_PIN_31     31
#define GPIO_PIN_32     32
#define GPIO_PIN_33     33
#define GPIO_PIN_34     34
#define GPIO_PIN_35     35
#define GPIO_PIN_36     36
#define GPIO_PIN_37     37
#define GPIO_PIN_38     38
#define GPIO_PIN_39     39
#define GPIO_PIN_40     40
#define GPIO_PIN_41     41
#define GPIO_PIN_42     42
#define GPIO_PIN_43     43
#define GPIO_PIN_44     44
#define GPIO_PIN_45     45
/**
  * @}
  */

/**
@brief 配置GPIO模式
cfgset列表
*/
#define GPIO_FUNC_SHIFT (0) /* Bits 0-4: GPIO功能 */
#define GPIO_FUNC_MASK  (0x1f << GPIO_FUNC_SHIFT)

/**
 * @brief 宏定义检查芯片是否是BL602，
 * 如果是则定义下述GPIO功能
 */
#if defined(BL602)
#define GPIO_FUNC_SDU  (1 << GPIO_FUNC_SHIFT)   // SDU功能
#define GPIO_FUNC_SPI0 (4 << GPIO_FUNC_SHIFT)   // SPI0功能
#define GPIO_FUNC_I2C0 (6 << GPIO_FUNC_SHIFT)   // I2C0功能
#define GPIO_FUNC_PWM0 (8 << GPIO_FUNC_SHIFT)   // PWM0功能
#define GPIO_FUNC_JTAG (14 << GPIO_FUNC_SHIFT)  // JTAG功能

/**
 * @brief 宏定义检查芯片是否是BL702，
 * 如果是则定义下述GPIO功能
 */
#elif defined(BL702)
#define GPIO_FUNC_CLK_OUT  (0 << GPIO_FUNC_SHIFT)    // 时钟输出功能
#define GPIO_FUNC_I2S      (3 << GPIO_FUNC_SHIFT)    // I2S功能
#define GPIO_FUNC_SPI0     (4 << GPIO_FUNC_SHIFT)    // SPI0功能
#define GPIO_FUNC_I2C0     (6 << GPIO_FUNC_SHIFT)    // I2C0功能
#define GPIO_FUNC_PWM0     (8 << GPIO_FUNC_SHIFT)    // PWM0功能
#define GPIO_FUNC_CAM      (9 << GPIO_FUNC_SHIFT)    // 摄像头功能
#define GPIO_FUNC_JTAG     (14 << GPIO_FUNC_SHIFT)   // JTAG功能
#define GPIO_FUNC_EMAC     (19 << GPIO_FUNC_SHIFT)   // 以太网MAC功能
#define GPIO_FUNC_CAM_MCLK (23 << GPIO_FUNC_SHIFT)   // 摄像头主时钟功能

/**
 * @brief 宏定义检查芯片是否是BL702L，
 * 如果是则定义下述GPIO功能
 */
#elif defined(BL702L)
#define GPIO_FUNC_SPI0    (4 << GPIO_FUNC_SHIFT)    // SPI0功能
#define GPIO_FUNC_I2C0    (6 << GPIO_FUNC_SHIFT)    // I2C0功能
#define GPIO_FUNC_PWM0    (8 << GPIO_FUNC_SHIFT)    // PWM0功能
#define GPIO_FUNC_KEYSCAN (13 << GPIO_FUNC_SHIFT)   // 按键扫描功能
#define GPIO_FUNC_JTAG    (14 << GPIO_FUNC_SHIFT)   // JTAG功能

/**
 * @brief 宏定义检查芯片是否是BL616，
 * 如果是则定义下述GPIO功能
 */
#elif defined(BL616)
#define GPIO_FUNC_SDH       (0 << GPIO_FUNC_SHIFT)   // SDH功能
#define GPIO_FUNC_SPI0      (1 << GPIO_FUNC_SHIFT)   // SPI0功能
#define GPIO_FUNC_I2S       (3 << GPIO_FUNC_SHIFT)   // I2S功能
#define GPIO_FUNC_PDM       (4 << GPIO_FUNC_SHIFT)   // PDM功能
#define GPIO_FUNC_I2C0      (5 << GPIO_FUNC_SHIFT)   // I2C0功能
#define GPIO_FUNC_I2C1      (6 << GPIO_FUNC_SHIFT)   // I2C1功能
#define GPIO_FUNC_EMAC      (8 << GPIO_FUNC_SHIFT)   // 以太网MAC功能
#define GPIO_FUNC_CAM       (9 << GPIO_FUNC_SHIFT)   // 摄像头功能
#define GPIO_FUNC_SDU       (12 << GPIO_FUNC_SHIFT)  // SDU功能
#define GPIO_FUNC_PWM0      (16 << GPIO_FUNC_SHIFT)  // PWM0功能
#define GPIO_FUNC_DBI_B     (22 << GPIO_FUNC_SHIFT)  // DBI_B功能
#define GPIO_FUNC_DBI_C     (23 << GPIO_FUNC_SHIFT)  // DBI_C功能
#define GPIO_FUNC_DBI_QSPI  (24 << GPIO_FUNC_SHIFT)  // DBI_QSPI功能
#define GPIO_FUNC_AUDAC_PWM (25 << GPIO_FUNC_SHIFT)  // AUDAC_PWM功能
#define GPIO_FUNC_JTAG      (26 << GPIO_FUNC_SHIFT)  // JTAG功能
#define GPIO_FUNC_CLKOUT    (31 << GPIO_FUNC_SHIFT)  // 时钟输出功能

/**
 * @brief 宏定义检查芯片是否是BL616 或者 BL808
 * 如果是则定义下述GPIO功能
 */
#elif defined(BL606P) || defined(BL808)
#define GPIO_FUNC_SDH     (0 << GPIO_FUNC_SHIFT)    // SDH功能
#define GPIO_FUNC_SPI0    (1 << GPIO_FUNC_SHIFT)    // SPI0功能
#define GPIO_FUNC_I2S     (3 << GPIO_FUNC_SHIFT)    // I2S功能
#define GPIO_FUNC_PDM     (4 << GPIO_FUNC_SHIFT)    // PDM功能
#define GPIO_FUNC_I2C0    (5 << GPIO_FUNC_SHIFT)    // I2C0功能
#define GPIO_FUNC_I2C1    (6 << GPIO_FUNC_SHIFT)    // I2C1功能
#define GPIO_FUNC_EMAC    (8 << GPIO_FUNC_SHIFT)    // 以太网MAC功能
#define GPIO_FUNC_CAM     (9 << GPIO_FUNC_SHIFT)    // 摄像头功能
#define GPIO_FUNC_SDU     (12 << GPIO_FUNC_SHIFT)   // SDU功能
#define GPIO_FUNC_PWM0    (16 << GPIO_FUNC_SHIFT)   // PWM0功能
#define GPIO_FUNC_PWM1    (17 << GPIO_FUNC_SHIFT)   // PWM1功能
#define GPIO_FUNC_SPI1    (18 << GPIO_FUNC_SHIFT)   // SPI1功能
#define GPIO_FUNC_I2C2    (19 << GPIO_FUNC_SHIFT)   // I2C2功能
#define GPIO_FUNC_I2C3    (20 << GPIO_FUNC_SHIFT)   // I2C3功能
#define GPIO_FUNC_DBI_B   (22 << GPIO_FUNC_SHIFT)   // DBI_B功能
#define GPIO_FUNC_DBI_C   (23 << GPIO_FUNC_SHIFT)   // DBI_C功能
#define GPIO_FUNC_JTAG_LP (25 << GPIO_FUNC_SHIFT)   // JTAG_LP功能
#define GPIO_FUNC_JTAG_M0 (26 << GPIO_FUNC_SHIFT)   // JTAG_M0功能
#define GPIO_FUNC_JTAG_D0 (27 << GPIO_FUNC_SHIFT)   // JTAG_D0功能
#define GPIO_FUNC_CLKOUT  (31 << GPIO_FUNC_SHIFT)   // 时钟输出功能

/**
 * @brief 宏定义检查芯片是否是BL628 或者 BL628
 * 如果是则定义下述GPIO功能
 */
#elif defined(BL628)
#define GPIO_FUNC_SDH    (0 << GPIO_FUNC_SHIFT)    // SDH功能
#define GPIO_FUNC_SPI0   (1 << GPIO_FUNC_SHIFT)    // SPI0功能
#define GPIO_FUNC_I2S    (3 << GPIO_FUNC_SHIFT)    // I2S功能
#define GPIO_FUNC_PDM    (4 << GPIO_FUNC_SHIFT)    // PDM功能
#define GPIO_FUNC_I2C0   (5 << GPIO_FUNC_SHIFT)    // I2C0功能
#define GPIO_FUNC_I2C1   (6 << GPIO_FUNC_SHIFT)    // I2C1功能
#define GPIO_FUNC_UART   (7 << GPIO_FUNC_SHIFT)    // UART功能
#define GPIO_FUNC_EMAC   (8 << GPIO_FUNC_SHIFT)    // 以太网MAC功能
#define GPIO_FUNC_CAM    (9 << GPIO_FUNC_SHIFT)    // 摄像头功能
#define GPIO_FUNC_SDU    (12 << GPIO_FUNC_SHIFT)   // SDU功能
#define GPIO_FUNC_PWM0   (16 << GPIO_FUNC_SHIFT)   // PWM0功能
#define GPIO_FUNC_DBI_B  (22 << GPIO_FUNC_SHIFT)   // DBI_B功能
#define GPIO_FUNC_DBI_C  (23 << GPIO_FUNC_SHIFT)   // DBI_C功能
#define GPIO_FUNC_CLKOUT (31 << GPIO_FUNC_SHIFT)   // 时钟输出功能
#endif

#define GPIO_MODE_SHIFT                      (5) /* Bits 5-6: 端口模式 */
#define GPIO_MODE_MASK                       (3 << GPIO_MODE_SHIFT)
#define GPIO_INPUT                           (0 << GPIO_MODE_SHIFT) /* 输入使能 */
#define GPIO_OUTPUT                          (1 << GPIO_MODE_SHIFT) /* 输出使能 */
#define GPIO_ANALOG                          (2 << GPIO_MODE_SHIFT) /* 模拟使能 */
#define GPIO_ALTERNATE                       (3 << GPIO_MODE_SHIFT) /* 备用使能 */

#define GPIO_PUPD_SHIFT                      (7) /* Bits 7-8: 上拉/下拉 */
#define GPIO_PUPD_MASK                       (3 << GPIO_PUPD_SHIFT)
#define GPIO_FLOAT                           (0 << GPIO_PUPD_SHIFT) /* 无上拉或下拉 */
#define GPIO_PULLUP                          (1 << GPIO_PUPD_SHIFT) /* 上拉 */
#define GPIO_PULLDOWN                        (2 << GPIO_PUPD_SHIFT) /* 下拉 */

#define GPIO_SMT_SHIFT                       (9) /* Bits 9: SMT使能 */
#define GPIO_SMT_MASK                        (1 << GPIO_SMT_SHIFT)
#define GPIO_SMT_DIS                         (0 << GPIO_SMT_SHIFT)
#define GPIO_SMT_EN                          (1 << GPIO_SMT_SHIFT)

#define GPIO_DRV_SHIFT                       (10) /* Bits 10-11: 驱动 */
#define GPIO_DRV_MASK                        (3 << GPIO_DRV_SHIFT)
#define GPIO_DRV_0                           (0 << GPIO_DRV_SHIFT)
#define GPIO_DRV_1                           (1 << GPIO_DRV_SHIFT)
#define GPIO_DRV_2                           (2 << GPIO_DRV_SHIFT)
#define GPIO_DRV_3                           (3 << GPIO_DRV_SHIFT)





/** @defgroup GPIO_INT_TRIG_MODE GPIO 中断触发模式定义
  * @{
  */
#define GPIO_INT_TRIG_MODE_SYNC_FALLING_EDGE 0     //下降沿
#define GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE  1     //上升沿
#define GPIO_INT_TRIG_MODE_SYNC_LOW_LEVEL    2     //低电平
#define GPIO_INT_TRIG_MODE_SYNC_HIGH_LEVEL   3     //高电平

/**
 * @brief 宏定义检查芯片是否是BL602 或者 BL702
 * 如果是则定义下述GPIO功能
 */
#if defined(BL602) || defined(BL702)
#define GPIO_INT_TRIG_MODE_ASYNC_FALLING_EDGE 4    // 异步下降沿
#define GPIO_INT_TRIG_MODE_ASYNC_RISING_EDGE  5    // 异步上升沿
#define GPIO_INT_TRIG_MODE_ASYNC_LOW_LEVEL    6    // 异步低电平
#define GPIO_INT_TRIG_MODE_ASYNC_HIGH_LEVEL   7    //异步高电平
#else
#define GPIO_INT_TRIG_MODE_SYNC_FALLING_RISING_EDGE 4     //同步下降沿或者上升沿
#define GPIO_INT_TRIG_MODE_ASYNC_FALLING_EDGE       8     //异步下降沿
#define GPIO_INT_TRIG_MODE_ASYNC_RISING_EDGE        9     //异步下降沿
#define GPIO_INT_TRIG_MODE_ASYNC_LOW_LEVEL          10    //异步低电平
#define GPIO_INT_TRIG_MODE_ASYNC_HIGH_LEVEL         11    //异步高电平
#endif
/**
  * @}
  */



/** @defgroup GPIO_UART_FUNC GPIO UART功能定义
  * @{
  */
#define GPIO_UART_FUNC_UART0_RTS 0
#define GPIO_UART_FUNC_UART0_CTS 1
#define GPIO_UART_FUNC_UART0_TX  2
#define GPIO_UART_FUNC_UART0_RX  3
#define GPIO_UART_FUNC_UART1_RTS 4
#define GPIO_UART_FUNC_UART1_CTS 5
#define GPIO_UART_FUNC_UART1_TX  6
#define GPIO_UART_FUNC_UART1_RX  7
#if defined(BL808) || defined(BL606P)
#define GPIO_UART_FUNC_UART2_RTS 8
#define GPIO_UART_FUNC_UART2_CTS 9
#define GPIO_UART_FUNC_UART2_TX  10
#define GPIO_UART_FUNC_UART2_RX  11
#endif
/**
  * @}
  */

/** @defgroup GPIO_CMD GPIO功能控制命令定义
  * @{
  */
#define GPIO_CMD_GET_GPIO_FUN (0x01)
/**
  * @}
  */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化GPIO引脚。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 * @param [in] cfgset GPIO配置掩码
 */
void bflb_gpio_init(struct bflb_device_s *dev, uint8_t pin, uint32_t cfgset);

/**
 * @brief 使能GPIO引脚并设置为输入浮空状态。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 */
void bflb_gpio_deinit(struct bflb_device_s *dev, uint8_t pin);

/**
 * @brief 设置GPIO引脚为高电平。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 */
void bflb_gpio_set(struct bflb_device_s *dev, uint8_t pin);

/**
 * @brief 设置GPIO引脚为低电平。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 */
void bflb_gpio_reset(struct bflb_device_s *dev, uint8_t pin);

/**
 * @brief 从GPIO引脚读取电平状态。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 * @return true表示高电平，否则为低电平
 */
bool bflb_gpio_read(struct bflb_device_s *dev, uint8_t pin);

/**
 * @brief 写入0~31号GPIO引脚。
 *
 * @param [in] dev 设备句柄
 * @param [in] val 0~31号GPIO引脚的值
 */
void bflb_gpio_pin0_31_write(struct bflb_device_s *dev, uint32_t val);

/**
 * @brief 写入32~63号GPIO引脚。
 *
 * @param [in] dev 设备句柄
 * @param [in] val 32~63号GPIO引脚的值
 */
void bflb_gpio_pin32_63_write(struct bflb_device_s *dev, uint32_t val);

/**
 * @brief 从0~31号GPIO引脚读取电平。
 *
 * @param [in] dev 设备句柄
 * @return 0~31号GPIO引脚的电平
 */
uint32_t bflb_gpio_pin0_31_read(struct bflb_device_s *dev);

/**
 * @brief 从32~63号GPIO引脚读取电平。
 *
 * @param [in] dev 设备句柄
 * @return 32~63号GPIO引脚的电平
 */
uint32_t bflb_gpio_pin32_63_read(struct bflb_device_s *dev);

/**
 * @brief 配置GPIO引脚中断。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 * @param [in] trig_mode 触发中断的模式
 */
void bflb_gpio_int_init(struct bflb_device_s *dev, uint8_t pin, uint8_t trig_mode);

/**
 * @brief 使能或禁用GPIO引脚中断。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 * @param [in] mask true表示禁用，false表示使能
 */
void bflb_gpio_int_mask(struct bflb_device_s *dev, uint8_t pin, bool mask);

/**
 * @brief 获取GPIO引脚中断状态。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 * @return true表示有中断，false表示无中断
 */
bool bflb_gpio_get_intstatus(struct bflb_device_s *dev, uint8_t pin);

/**
 * @brief 清除GPIO引脚中断状态。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 */
void bflb_gpio_int_clear(struct bflb_device_s *dev, uint8_t pin);

/**
 * @brief 配置GPIO引脚为UART功能。
 *
 * @param [in] dev 设备句柄
 * @param [in] pin GPIO引脚，使用 @ref GPIO_PIN
 * @param [in] uart_func UART功能，使用 @ref GPIO_UART_FUNC
 */
void bflb_gpio_uart_init(struct bflb_device_s *dev, uint8_t pin, uint8_t uart_func);

/**
 * @brief 控制GPIO功能。
 *
 * @param [in] dev 设备句柄
 * @param [in] cmd 功能命令，使用 @ref GPIO_CMD
 * @param [in] arg 用户数据
 * @return 失败时返回负数的错误值。
 */

int bflb_gpio_feature_control(struct bflb_device_s *dev, int cmd, size_t arg);

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
  */

#endif
