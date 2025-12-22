#ifndef DRIVERS_PCA9685_REGISTERS_H
#define DRIVERS_PCA9685_REGISTERS_H

typedef enum {
  PCA9685_MODE1 = 0x00,        /**< Mode Register 1 */
  PCA9685_MODE2 = 0x01,        /**< Mode Register 2 */
  PCA9685_SUBADR1 = 0x02,      /**< I2C-bus subaddress 1 */
  PCA9685_SUBADR2 = 0x03,      /**< I2C-bus subaddress 2 */
  PCA9685_SUBADR3 = 0x04,      /**< I2C-bus subaddress 3 */
  PCA9685_ALLCALLADR = 0x05,   /**< LED All Call I2C-bus address */
  PCA9685_LED0_ON_L = 0x06,    /**< LED0 on tick, low byte*/
  PCA9685_LED0_ON_H = 0x07,    /**< LED0 on tick, high byte*/
  PCA9685_LED0_OFF_L = 0x08,   /**< LED0 off tick, low byte */
  PCA9685_LED0_OFF_H = 0x09,   /**< LED0 off tick, high byte */
  PCA9685_ALLLED_ON_L = 0xFA,  /**< load all the LEDn_ON registers, low */
  PCA9685_ALLLED_ON_H = 0xFB,  /**< load all the LEDn_ON registers, high */
  PCA9685_ALLLED_OFF_L = 0xFC, /**< load all the LEDn_OFF registers, low */
  PCA9685_ALLLED_OFF_H = 0xFD, /**< load all the LEDn_OFF registers,high */
  PCA9685_PRESCALE = 0xFE,     /**< Prescaler for PWM output frequency */
  PCA9685_TESTMODE = 0xFF,     /**< defines the test mode to be entered */
} PCA9685Registers_e;

typedef enum {
  MODE1_ALLCAL = 0x01,  /**< respond to LED All Call I2C-bus address */
  MODE1_SUB3 = 0x02,    /**< respond to I2C-bus subaddress 3 */
  MODE1_SUB2 = 0x04,    /**< respond to I2C-bus subaddress 2 */
  MODE1_SUB1 = 0x08,    /**< respond to I2C-bus subaddress 1 */
  MODE1_SLEEP = 0x10,   /**< Low power mode. Oscillator off */
  MODE1_AI = 0x20,      /**< Auto-Increment enabled */
  MODE1_EXTCLK = 0x40,  /**< Use EXTCLK pin clock */
  MODE1_RESTART = 0x80, /**< Restart enabled */
} PCA9685Mode1Register_e;

typedef enum {
  MODE2_OUTNE_0 = 0x01, /**< Active LOW output enable input */
  MODE2_OUTNE_1 = 0x02, /**< Active LOW output enable input - high impedience */
  MODE2_OUTDRV = 0x04,  /**< totem pole structure vs open-drain */
  MODE2_OCH = 0x08,     /**< Outputs change on ACK vs STOP */
  MODE2_INVRT = 0x10,   /**< Output logic state inverted */
} PCA9685Mode2Register_e;

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

#define DEFAULT_MODE1 0x11

#endif // !DRIVERS_PCA9685_REGISTERS_H
