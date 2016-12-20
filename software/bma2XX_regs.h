#include <inttypes.h>
#include <stdbool.h>

/*
 * Register transcription by Helge Wurst
 */

#ifndef _BMA2XX_REGS_H_
#define _BMA2XX_REGS_H_

  #define BMAREG_FIFO_DATA_OUT 0x3F // (BMA255/280 only)
  #define BMAREG_FIFO_CONFIG   0x3E // (BMA255/280 only)
  #define BMA_FIFO_MODE_BYPASS  (0b00 << 6)// (BMA255/280 only)
  #define BMA_FIFO_MODE_FIFO    (0b01 << 6)// (BMA255/280 only)
  #define BMA_FIFO_MODE_STREAM  (0b10 << 6)// (BMA255/280 only)
  #define BMA_FIFO_DATA_SEL_XYZ (0b00 << 0)// (BMA255/280 only)
  #define BMA_FIFO_DATA_SEL_X_ONLY (0b01 << 0)// (BMA255/280 only)
  #define BMA_FIFO_DATA_SEL_Y_ONLY (0b10 << 0)// (BMA255/280 only)
  #define BMA_FIFO_DATA_SEL_Z_ONLY (0b11 << 0)// (BMA255/280 only)
  
  #define BMAREG_OFFSET_FILT_Z 0x3D // _BMA250_ only!
  #define BMAREG_GP1       0x3C //  (BMA255/280 only)
  #define BMAREG_OFFSET_FILT_Y 0x3C // _BMA250_ only!
  #define BMAREG_GP0       0x3B //  (BMA255/280 only)
  #define BMAREG_OFFSET_FILT_X 0x3B // _BMA250_ only!

  #define BMAREG_OFFSET_Z 0x3A
  #define BMAREG_OFFSET_Y 0x39
  #define BMAREG_OFFSET_X 0x38
  
  #define BMAREG_UNDEF1 0x37
  #define BMAREG_UNDEF2 0x36
  
  #define BMAREG_INTERFACE_CFG 0x34
  #define IFCON_I2C_WDT_EN  2
  #define IFCON_I2C_WDT_SEL 1
  #define IFCON_SPI_3WIRE   0

  #define BMAREG_NVM 0x33
  #define NVM_REMAIN    4 // (BMA255/280 only) bits 7:4
  #define NVM_LOAD    3
  #define NVM_READY   2
  #define NVM_PROG_TRIG 1
  #define NVM_PROG_MODE 0
  
  // BMA2XX register map, bits and values
  #define BMAREG_SELF_TEST 0x32
  #define SELF_TEST_Z 0b11
  #define SELF_TEST_Y 0b10
  #define SELF_TEST_X 0b01
  #define SELF_TEST_OFF 0b00
  #define SELF_TEST_SIGN_POSITIVE 0b000
  #define SELF_TEST_SIGN_NEGATIVE 0b100
  
  #define BMAREG_FIFO_WATERMARK 0x30 // BMA255/280 only
  #define FIFOCFG_WATERMARKLVL_TRIG 0 // bits 5:0
  #define BMAREG_FLAT1 0x2F
  #define FLAT_HOLD_TIME  4 // bits 5:4 // incomplete def
  #define FLAT_HYST   0 // bits 2:0 // incomplete def
  #define BMAREG_FLAT0 0x2E
  #define FLAT_THETA 0 // bits 5:0
  #define BMAREG_ORIENT1 0x2D
  #define ORIENT_UD_EN 6 // (BMA255/280 only)
  #define ORIENT_THETA 0 // bits 5:0
  #define BMAREG_ORIENT0 0x2C
  #define ORIENT_HYST (0b000 << 4) // incomplete def
  #define ORIENT_BLOCKING (0b00 << 2) // incomplete def
  #define ORIENT_MODE (0b00 << 0) // incomplete def
  #define BMAREG_TAP_OPT_THR 0x2B
  #define TAP_SAMPLES_16 (0b11<<6)
  #define TAP_SAMPLES_8  (0b10<<6)
  #define TAP_SAMPLES_4  (0b01<<6)
  #define TAP_SAMPLES_2  (0b00<<6)
  #define TAP_THR 0 // bits 4:0 occupy tap_threshold, see ds
  #define BMAREG_TAP_OPT_DUR 0x2A
  #define TAP_QUIET 7
  #define TAP_SHOCK 6
  // bits 3:0 occupy tap_dur, see ds
  #define BMAREG_SLO_NO_MOT 0x29 // (BMA255/280 only)
  #define BMAREG_SLOPE_THRESHOLD 0x28
  #define BMAREG_SLOW_THRESHOLD 0x29
  #define BMAREG_SLOPE_DURATION 0x27
  #define BMAREG_HIGH_THRESHOLD 0x26
  #define BMAREG_HIGH_DURATION 0x25
  #define BMAREG_MODE_HYST  0x24
  #define HM_HI1      7 // swap for options in ds on demand
  #define HM_HI0      6
  #define HM_LOW_MODE 2
  #define HM_LO1      1 // swap for options in ds on demand
  #define HM_LO0      0
  #define BMAREG_LOW_THRESHOLD 0x23
  #define BMAREG_LOW_DURATION 0x22

  #define BMAREG_INT_CTRL_LATCH    0x21 // interrupt monostable time or latching settings (bit 3:0 settings: ds table 14)
  #define BMA_RESET_INT    7 // setting this clears any latched interrupts
  // bits 3:0 : interrupt signal mode (see below)
  #define INTMODE_NONLATCHED 0b0000
  #define INTMODE_TEMP250ms  0b0001
  #define INTMODE_TEMP500ms  0b0010
  #define INTMODE_TEMP1s     0b0011
  #define INTMODE_TEMP2s     0b0100
  #define INTMODE_TEMP4s     0b0101
  #define INTMODE_TEMP8s     0b0110
  #define INTMODE_LATCHED    0b0111
  #define INTMODE_TEMP500us  0b1001
  #define INTMODE_TEMP1ms    0b1011
  #define INTMODE_TEMP12ms   0b1100 // 12.5 millisecs
  #define INTMODE_TEMP25ms   0b1101
  #define INTMODE_TEMP50ms   0b1110
  #define BMAREG_INTPIN_OPTS 0x20 // interrupt output drive modes. Use open-drive + active lo for TWI compliance
  #define INTPIN_INT2_DRV    3 // 0: open-drive, 1: push-pull
  #define INTPIN_INT2_ACTLVL 2 // set to '0' for active high mode
  #define INTPIN_INT1_DRV    1 // 0: open-drive, 1: push-pull
  #define INTPIN_INT1_ACTLVL 0 // set to '0' for active high mode
  #define INTPIN_ACTIVE_HI 0x01
  #define INTPIN_ACTIVE_LO 0x00
  #define INTPIN_PUSHPULL  0x00
  #define INTPIN_OPENDRIVE 0x01
  #define BMAREG_INT_MAP3 0x1B
  #define MAP_INT2_FLAT       7
  #define MAP_INT2_ORIENT     6
  #define MAP_INT2_SINGLE_TAP 5
  #define MAP_INT2_DOUBLE_TAP 4
  #define MAP_INT2_SLOPE      2
  #define MAP_INT2_HI         1
  #define MAP_INT2_LO         0
  #define BMAREG_INT_MAP2 0x1A
  #define MAP_INT2_DATA 7
  #define MAP_INT1_DATA 0
  #define BMAREG_INT_MAP1 0x19
  #define MAP_INT1_FLAT       7
  #define MAP_INT1_ORIENT     6
  #define MAP_INT1_SINGLE_TAP 5
  #define MAP_INT1_DOUBLE_TAP 4
  #define MAP_INT1_NO_MOTION 3
  #define MAP_INT1_SLOPE      2
  #define MAP_INT1_HI         1
  #define MAP_INT1_LO         0
  #define BMAREG_DETECT_OPTS3 0x18  // (BMA255/280 only)
  #define DO_SLO_NO_MOT_SEL 3 // (BMA255/280 only)
  #define DO_SLO_NO_MOT_Z_EN  2 // (BMA255/280 only)
  #define DO_SLO_NO_MOT_Y_EN  1 // (BMA255/280 only)
  #define DO_SLO_NO_MOT_X_EN  0 // (BMA255/280 only)
  
  #define BMAREG_DETECT_OPTS2 0x17
  #define DO_FIFO_WM_EN 6   // (BMA255/280 only)
  #define DO_FIFO_FULL_EN 5 // (BMA255/280 only)
  #define DO_NEW_DATA_EN 4
  #define DO_LOW_EN      3
  #define DO_HIGH_Z_EN   2
  #define DO_HIGH_Y_EN   1
  #define DO_HIGH_X_EN   0
  #define BMAREG_DETECT_OPTS1 0x16
  #define DO_FLAT_EN       7
  #define DO_ORIENT_EN     6
  #define DO_SINGLE_TAP_EN 5 // only one tap mode is allowed at a time
  #define DO_DOUBLE_TAP_EN 4
  #define DO_SLOPE_Z_EN    2 // motion detection can be enabled for each axis individually
  #define DO_SLOPE_Y_EN    1
  #define DO_SLOPE_X_EN  0
  // - 0x15 reserved -
  #define BMAREG_SOFTRESET 0x14
  #define BMA_SOFTRESET_MAGICNUMBER 0xB6 // writing this number to 0x14 triggers a soft reset (30-40ms wait time?)
  
  #define BMAREG_DATA_ACQUISITION 0x13
  #define DISABLE_LOW_PASS_FILTERING 7 // data_high_bandwidth option for unfiltered AD values to output the unfiltered datastream (default: filtering enabled (0))
  #define DISABLE_SHADOWING 6 // with shadowing enabled, >8bit data is double buffered to get consisting low/high byte values.
  
  #define BMAREG_LOWPOWER 0x12      // (BMA255/280 only)
  #define LPWR_LOWPOWER_MODE    6 // (BMA255/280 only), set to 1 for BMA250 legacy mode (default is 0 :( )
  #define LPWR_LOWPOWER_MODE1   0b0 // (0 << LPWR_LOWPOWER_MODE) // (BMA255/280 only)
  #define LPWR_LOWPOWER_MODE2   0b1 // (1 << LPWR_LOWPOWER_MODE) // (BMA255/280 only)
  #define LPWR_SLEEPTIMER_MODE   5  // (BMA255/280 only), set to 0 for BMA250 legacy mode (default is 0 :( )
  #define LPWR_SLEEPMODE_EVENT_DRIVEN 0b0 // (BMA255/280 only)
  #define LPWR_SLEEPMODE_EQUIDISTANT  0b1 // (BMA255/280 only)
  #define LPWR_LOWPOWER_RESERVED 0  // (BMA255/280 only), bits 4:0
  
  #define BMAREG_SLEEP_DURATION 0x11
  #define BMA_SUSPEND      7
  #define BMA_LOWPOWER_ENA 6 // also depends on external CS pin in some modes
  #define BMA_DEEP_SUSPEND 5 // BMA280 only
  #define BMA_SLEEP        1 // bits 1 through 4
  #define BMA_SLEEP_500us 0b0000 // 100µA average power consumption (low power mode, normal mode: 137µA)
  #define BMA_SLEEP_1ms 0b0110 // 78 µA
  #define BMA_SLEEP_2ms 0b0111 // 55 µA
  #define BMA_SLEEP_4ms 0b1000 // 34µA
  #define BMA_SLEEP_6ms 0b1001 // 25µA
  #define BMA_SLEEP_10ms  0b1010 // 16µA
  #define BMA_SLEEP_25ms  0b1011 // 7µA
  #define BMA_SLEEP_50ms  0b1100 // 4µA
  #define BMA_SLEEP_100ms 0b1101 // 2.3µA
  #define BMA_SLEEP_500ms 0b1110 // 0.9µA
  #define BMA_SLEEP_1s  0b1111 // 0.7µA
  
  #define BMAREG_BANDWIDTH 0x10 // sampling rate is twice the bandwidth value
  #define BW_8Hz    0b01000 //    7.81 Hz
  #define BW_16Hz   0b01001 //   15.63 Hz
  #define BW_32Hz   0b01010 //   31.25 Hz
  #define BW_63Hz   0b01011 //   62.5 Hz
  #define BW_125Hz  0b01100 //  125 Hz
  #define BW_250Hz  0b01101 //  250 Hz
  #define BW_500Hz  0b01110 //  500 Hz
  #define BW_1000Hz 0b01111 // 1000 Hz
  
  #define BMAREG_ACC_RANGE 0x0F // bits 3:0
  #define ACC_2g  0b0011
  #define ACC_4g  0b0101
  #define ACC_8g  0b1000
  #define ACC_16g 0b1100
  
  #define BMAREG_FIFOSTAT 0x0E    // (BMA255/280 only)
  #define FIFOSTAT_FFOV   7 // (BMA255/280 only) FIFO overflow
  #define FIFOSTAT_FRAMECNT 0 // (BMA255/280 only) bits 6:0: FIFO_FRAME_COUNTER
  
  #define BMAREG_INTSTAT3 0x0C
  #define INTSTAT3_FLAT   7
  #define INTSTAT3_ORIENT   4 // bits 6:4 // incomplete def
  #define INTSTAT3_HIGH_SIGN  3
  #define INTSTAT3_HIGH_Z   2
  #define INTSTAT3_HIGH_Y   1
  #define INTSTAT3_HIGH_X   0
  #define BMAREG_INTSTAT2 0x0B
  #define INTSTAT2_TAPSIGN    7
  #define INTSTAT2_TAP_FIRST_Z  6
  #define INTSTAT2_TAP_FIRST_Y  5
  #define INTSTAT2_TAP_FIRST_X  4
  #define INTSTAT2_SLOPE_SIGN   3
  #define INTSTAT2_SLOPE_FIRST_Z  2
  #define INTSTAT2_SLOPE_FIRST_Y  1
  #define INTSTAT2_SLOPE_FIRST_X  0
  #define BMAREG_INTSTAT1 0x0A
  #define INTSTAT1_DATAINT      7
  #define INTSTAT1_FIFO_WATERMARK_INT 6 // (BMA255/280 only) FIFO almost full
  #define INTSTAT1_FIFO_FULL_INT    5 // (BMA255/280 only) FIFO full
  #define BMAREG_INTSTAT0 0x09
  #define INTSTAT0_FLATINT    7
  #define INTSTAT0_ORIENTINT    6
  #define INTSTAT0_STAPINT    5
  #define INTSTAT0_DTAPINT    4
  #define INTSTAT0_SLO_NO_MOT_INT 3
  #define INTSTAT0_SLOPEINT   2
  #define INTSTAT0_HIGHINT    1
  #define INTSTAT0_LOWINT     0
  
  #define BMAREG_TEMPERATURE 0x08
  #define BMAREG_ACC_Z_HI 0x07
  #define BMAREG_ACC_Z_LO 0x06
  #define BMAREG_ACC_Y_HI 0x05
  #define BMAREG_ACC_Y_LO 0x04
  #define BMAREG_ACC_X_HI 0x03
  #define BMAREG_ACC_X_LO 0x02
  #define BMAREG_CHIPID 0x00
  

  
#endif // _BMA2XX_REGS_H_
