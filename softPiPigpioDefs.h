#ifndef _SOFT_PI_PIGPIO_DEFS_H
#define _SOFT_PI_PIGPIO_DEFS_H

#include <stdint.h>
#include <pthread.h>

#define THOUSAND 1000
#define MILLION  1000000
#define BILLION  1000000000

typedef void (*callbk_t) ();

typedef struct
{
   uint16_t func;
   uint16_t size;
} gpioHeader_t;

typedef struct
{
   size_t size;
   void *ptr;
   uint32_t data;
} gpioExtent_t;

typedef struct
{
   uint32_t tick;
   uint32_t level;
} gpioSample_t;

typedef struct
{
   uint16_t seqno;
   uint16_t flags;
   uint32_t tick;
   uint32_t level;
} gpioReport_t;

typedef struct
{
   uint32_t gpioOn;
   uint32_t gpioOff;
   uint32_t usDelay;
} gpioPulse_t;

#define WAVE_FLAG_READ  1
#define WAVE_FLAG_TICK  2

typedef struct
{
   uint32_t gpioOn;
   uint32_t gpioOff;
   uint32_t usDelay;
   uint32_t flags;
} rawWave_t;

/*
CBs are used in order from the lowest numbered CB up to
the maximum NUM_WAVE_CBS.

OOLS are used from the bottom climbing up and from
the top climbing down.

The GPIO on and off settings climb up from the bottom (botOOL/numBOOL).

The level and tick read values are stored in descending locations
from the top (topOOL/numTOOL).
*/

typedef struct
{
   uint16_t botCB;  /* first CB used by wave  */
   uint16_t topCB;  /* last CB used by wave   */
   uint16_t botOOL; /* first bottom OOL used by wave  */
                    /* botOOL to botOOL + numBOOL - 1 are in use */
   uint16_t topOOL; /* last top OOL used by wave */
                    /* topOOL - numTOOL to topOOL are in use.*/
   uint16_t deleted;
   uint16_t numCB;
   uint16_t numBOOL;
   uint16_t numTOOL;
} rawWaveInfo_t;

typedef struct
{
   int clk;     /* GPIO for clock           */
   int mosi;    /* GPIO for MOSI            */
   int miso;    /* GPIO for MISO            */
   int ss_pol;  /* slave select off state   */
   int ss_us;   /* delay after slave select */
   int clk_pol; /* clock off state          */
   int clk_pha; /* clock phase              */
   int clk_us;  /* clock micros             */
} rawSPI_t;

typedef struct { /* linux/arch/arm/mach-bcm2708/include/mach/dma.h */
   uint32_t info;
   uint32_t src;
   uint32_t dst;
   uint32_t length;
   uint32_t stride;
   uint32_t next;
   uint32_t pad[2];
} rawCbs_t;


typedef struct
{
   uint16_t addr;  /* slave address       */
   uint16_t flags;
   uint16_t len;   /* msg length          */
   uint8_t  *buf;  /* pointer to msg data */
} pi_i2c_msg_t;

/* BSC FIFO size */

#define BSC_FIFO_SIZE 512

typedef struct
{
   uint32_t control;          /* Write */
   int rxCnt;                 /* Read only */
   char rxBuf[BSC_FIFO_SIZE]; /* Read only */
   int txCnt;                 /* Write */
   char txBuf[BSC_FIFO_SIZE]; /* Write */
} bsc_xfer_t;

typedef struct
{
   callbk_t func;
   unsigned ex;
   void *userdata;

   int      wdSteadyUs;
   uint32_t wdTick;
   uint32_t wdLBitV;

   int      nfSteadyUs;
   int      nfActiveUs;
   int      nfActive;
   uint32_t nfTick1;
   uint32_t nfTick2;
   uint32_t nfLBitV;
   uint32_t nfRBitV;

   uint32_t gfSteadyUs;
   uint32_t gfTick;
   uint32_t gfLBitV;
   uint32_t gfRBitV;

} gpioAlert_t;

typedef void (*gpioAlertFunc_t)    (int      gpio,
                                    int      level,
                                    uint32_t tick);

typedef void (*gpioAlertFuncEx_t)  (int      gpio,
                                    int      level,
                                    uint32_t tick,
                                    void    *userdata);

typedef void (*eventFunc_t)        (int      event,
                                    uint32_t tick);

typedef void (*eventFuncEx_t)      (int      event,
                                    uint32_t tick,
                                    void    *userdata);

typedef void (*gpioISRFunc_t)      (int      gpio,
                                    int      level,
                                    uint32_t tick);

typedef void (*gpioISRFuncEx_t)    (int      gpio,
                                    int      level,
                                    uint32_t tick,
                                    void    *userdata);

typedef void (*gpioTimerFunc_t)    (void);

typedef void (*gpioTimerFuncEx_t)  (void *userdata);

typedef void (*gpioTimerFuncEx_t)  (void *userdata);

typedef void (*gpioSignalFunc_t)   (int signum);

typedef void (*gpioSignalFuncEx_t) (int    signum,
                                    void  *userdata);

typedef void (*gpioGetSamplesFunc_t)   (const gpioSample_t *samples,
                                        int                 numSamples);

typedef void (*gpioGetSamplesFuncEx_t) (const gpioSample_t *samples,
                                        int                 numSamples,
                                        void               *userdata);

typedef void *(gpioThreadFunc_t) (void *);


/* gpio: 0-53 */

#define PI_MIN_GPIO       0
#define PI_MAX_GPIO      53

/* user_gpio: 0-31 */

#define PI_MAX_USER_GPIO 31

/* level: 0-1 */

#define PI_OFF   0
#define PI_ON    1

#define PI_CLEAR 0
#define PI_SET   1

#define PI_LOW   0
#define PI_HIGH  1

/* level: only reported for GPIO time-out, see gpioSetWatchdog */

#define PI_TIMEOUT 2

/* mode: 0-7 */

#define PI_INPUT  0
#define PI_OUTPUT 1
#define PI_ALT0   4
#define PI_ALT1   5
#define PI_ALT2   6
#define PI_ALT3   7
#define PI_ALT4   3
#define PI_ALT5   2

/* pud: 0-2 */

#define PI_PUD_OFF  0
#define PI_PUD_DOWN 1
#define PI_PUD_UP   2

/* dutycycle: 0-range */

#define PI_DEFAULT_DUTYCYCLE_RANGE   255

/* range: 25-40000 */

#define PI_MIN_DUTYCYCLE_RANGE        25
#define PI_MAX_DUTYCYCLE_RANGE     40000

/* pulsewidth: 0, 500-2500 */

#define PI_SERVO_OFF 0
#define PI_MIN_SERVO_PULSEWIDTH 500
#define PI_MAX_SERVO_PULSEWIDTH 2500

/* hardware PWM */

#define PI_HW_PWM_MIN_FREQ 1
#define PI_HW_PWM_MAX_FREQ      125000000
#define PI_HW_PWM_MAX_FREQ_2711 187500000
#define PI_HW_PWM_RANGE 1000000

/* hardware clock */

#define PI_HW_CLK_MIN_FREQ       4689
#define PI_HW_CLK_MIN_FREQ_2711 13184
#define PI_HW_CLK_MAX_FREQ      250000000
#define PI_HW_CLK_MAX_FREQ_2711 375000000

#define PI_NOTIFY_SLOTS  32

#define PI_NTFY_FLAGS_EVENT    (1 <<7)
#define PI_NTFY_FLAGS_ALIVE    (1 <<6)
#define PI_NTFY_FLAGS_WDOG     (1 <<5)
#define PI_NTFY_FLAGS_BIT(x) (((x)<<0)&31)

#define PI_WAVE_BLOCKS     4
#define PI_WAVE_MAX_PULSES (PI_WAVE_BLOCKS * 3000)
#define PI_WAVE_MAX_CHARS  (PI_WAVE_BLOCKS *  300)

#define PI_BB_I2C_MIN_BAUD     50
#define PI_BB_I2C_MAX_BAUD 500000

#define PI_BB_SPI_MIN_BAUD     50
#define PI_BB_SPI_MAX_BAUD 250000

#define PI_BB_SER_MIN_BAUD     50
#define PI_BB_SER_MAX_BAUD 250000

#define PI_BB_SER_NORMAL 0
#define PI_BB_SER_INVERT 1

#define PI_WAVE_MIN_BAUD      50
#define PI_WAVE_MAX_BAUD 1000000

#define PI_SPI_MIN_BAUD     32000
#define PI_SPI_MAX_BAUD 125000000

#define PI_MIN_WAVE_DATABITS 1
#define PI_MAX_WAVE_DATABITS 32

#define PI_MIN_WAVE_HALFSTOPBITS 2
#define PI_MAX_WAVE_HALFSTOPBITS 8

#define PI_WAVE_MAX_MICROS (30 * 60 * 1000000) /* half an hour */

#define PI_MAX_WAVES 250

#define PI_MAX_WAVE_CYCLES 65535
#define PI_MAX_WAVE_DELAY  65535

#define PI_WAVE_COUNT_PAGES 10

/* wave tx mode */

#define PI_WAVE_MODE_ONE_SHOT      0
#define PI_WAVE_MODE_REPEAT        1
#define PI_WAVE_MODE_ONE_SHOT_SYNC 2
#define PI_WAVE_MODE_REPEAT_SYNC   3

/* special wave at return values */

#define PI_WAVE_NOT_FOUND  9998 /* Transmitted wave not found. */
#define PI_NO_TX_WAVE      9999 /* No wave being transmitted. */

/* Files, I2C, SPI, SER */

#define PI_FILE_SLOTS 16
#define PI_I2C_SLOTS  64
#define PI_SPI_SLOTS  32
#define PI_SER_SLOTS  16

#define PI_MAX_I2C_ADDR 0x7F

#define PI_NUM_AUX_SPI_CHANNEL 3
#define PI_NUM_STD_SPI_CHANNEL 2

#define PI_MAX_I2C_DEVICE_COUNT (1<<16)
#define PI_MAX_SPI_DEVICE_COUNT (1<<16)

/* max pi_i2c_msg_t per transaction */

#define  PI_I2C_RDRW_IOCTL_MAX_MSGS 42

/* flags for i2cTransaction, pi_i2c_msg_t */

#define PI_I2C_M_WR           0x0000 /* write data */
#define PI_I2C_M_RD           0x0001 /* read data */
#define PI_I2C_M_TEN          0x0010 /* ten bit chip address */
#define PI_I2C_M_RECV_LEN     0x0400 /* length will be first received byte */
#define PI_I2C_M_NO_RD_ACK    0x0800 /* if I2C_FUNC_PROTOCOL_MANGLING */
#define PI_I2C_M_IGNORE_NAK   0x1000 /* if I2C_FUNC_PROTOCOL_MANGLING */
#define PI_I2C_M_REV_DIR_ADDR 0x2000 /* if I2C_FUNC_PROTOCOL_MANGLING */
#define PI_I2C_M_NOSTART      0x4000 /* if I2C_FUNC_PROTOCOL_MANGLING */

/* bbI2CZip and i2cZip commands */

#define PI_I2C_END          0
#define PI_I2C_ESC          1
#define PI_I2C_START        2
#define PI_I2C_COMBINED_ON  2
#define PI_I2C_STOP         3
#define PI_I2C_COMBINED_OFF 3
#define PI_I2C_ADDR         4
#define PI_I2C_FLAGS        5
#define PI_I2C_READ         6
#define PI_I2C_WRITE        7

/* SPI */

#define PI_SPI_FLAGS_BITLEN(x) ((x&63)<<16)
#define PI_SPI_FLAGS_RX_LSB(x)  ((x&1)<<15)
#define PI_SPI_FLAGS_TX_LSB(x)  ((x&1)<<14)
#define PI_SPI_FLAGS_3WREN(x)  ((x&15)<<10)
#define PI_SPI_FLAGS_3WIRE(x)   ((x&1)<<9)
#define PI_SPI_FLAGS_AUX_SPI(x) ((x&1)<<8)
#define PI_SPI_FLAGS_RESVD(x)   ((x&7)<<5)
#define PI_SPI_FLAGS_CSPOLS(x)  ((x&7)<<2)
#define PI_SPI_FLAGS_MODE(x)    ((x&3))

/* BSC registers */
#define BSC_DR         0
#define BSC_RSR        1
#define BSC_SLV        2
#define BSC_CR         3
#define BSC_FR         4
#define BSC_IFLS       5
#define BSC_IMSC       6
#define BSC_RIS        7
#define BSC_MIS        8
#define BSC_ICR        9
#define BSC_DMACR     10
#define BSC_TDR       11
#define BSC_GPUSTAT   12
#define BSC_HCTRL     13
#define BSC_DEBUG_I2C 14
#define BSC_DEBUG_SPI 15

#define BSC_CR_TESTFIFO 2048
#define BSC_CR_RXE  512
#define BSC_CR_TXE  256
#define BSC_CR_BRK  128
#define BSC_CR_CPOL  16
#define BSC_CR_CPHA   8
#define BSC_CR_I2C    4
#define BSC_CR_SPI    2
#define BSC_CR_EN     1

#define BSC_FR_RXBUSY 32
#define BSC_FR_TXFE   16
#define BSC_FR_RXFF    8
#define BSC_FR_TXFF    4
#define BSC_FR_RXFE    2
#define BSC_FR_TXBUSY  1

/* BSC GPIO */

#define BSC_SDA_MOSI 18
#define BSC_SCL_SCLK 19
#define BSC_MISO     20
#define BSC_CE_N     21

/* Longest busy delay */

#define PI_MAX_BUSY_DELAY 0
//100

/* timeout: 0-60000 */

#define PI_MIN_WDOG_TIMEOUT 0
#define PI_MAX_WDOG_TIMEOUT 60000
/* timer: 0-9 */

#define PI_MIN_TIMER 0
#define PI_MAX_TIMER 9

/* millis: 10-60000 */

#define PI_MIN_MS 10
#define PI_MAX_MS 60000

#define PI_MAX_SCRIPTS       32

#define PI_MAX_SCRIPT_TAGS   50
#define PI_MAX_SCRIPT_VARS  150
#define PI_MAX_SCRIPT_PARAMS 10

/* script status */

#define PI_SCRIPT_INITING 0
#define PI_SCRIPT_HALTED  1
#define PI_SCRIPT_RUNNING 2
#define PI_SCRIPT_WAITING 3
#define PI_SCRIPT_FAILED  4

/* signum: 0-63 */

#define PI_MIN_SIGNUM 0
#define PI_MAX_SIGNUM 63

/* timetype: 0-1 */

#define PI_TIME_RELATIVE 0
#define PI_TIME_ABSOLUTE 1

#define PI_MAX_MICS_DELAY 1000000 /* 1 second */
#define PI_MAX_MILS_DELAY 60000   /* 60 seconds */

/* cfgMillis */

#define PI_BUF_MILLIS_MIN 100
#define PI_BUF_MILLIS_MAX 10000

/* cfgMicros: 1, 2, 4, 5, 8, or 10 */

/* cfgPeripheral: 0-1 */

#define PI_CLOCK_PWM 0
#define PI_CLOCK_PCM 1

/* DMA channel: 0-15, 15 is unset */

#define PI_MIN_DMA_CHANNEL 0
#define PI_MAX_DMA_CHANNEL 15

/* port */

#define PI_MIN_SOCKET_PORT 1024
#define PI_MAX_SOCKET_PORT 32000


/* ifFlags: */

#define PI_DISABLE_FIFO_IF   1
#define PI_DISABLE_SOCK_IF   2
#define PI_LOCALHOST_SOCK_IF 4
#define PI_DISABLE_ALERT     8

/* memAllocMode */

#define PI_MEM_ALLOC_AUTO    0
#define PI_MEM_ALLOC_PAGEMAP 1
#define PI_MEM_ALLOC_MAILBOX 2

/* filters */

#define PI_MAX_STEADY  300000
#define PI_MAX_ACTIVE 1000000

/* gpioCfgInternals */

#define PI_CFG_DBG_LEVEL         0 /* bits 0-3 */
#define PI_CFG_ALERT_FREQ        4 /* bits 4-7 */
#define PI_CFG_RT_PRIORITY       (1<<8)
#define PI_CFG_STATS             (1<<9)
#define PI_CFG_NOSIGHANDLER      (1<<10)

#define PI_CFG_ILLEGAL_VAL       (1<<11)


/* gpioISR */

#define RISING_EDGE  0
#define FALLING_EDGE 1
#define EITHER_EDGE  2


/* pads */

#define PI_MAX_PAD 2

#define PI_MIN_PAD_STRENGTH 1
#define PI_MAX_PAD_STRENGTH 16

/* files */

#define PI_FILE_NONE   0
#define PI_FILE_MIN    1
#define PI_FILE_READ   1
#define PI_FILE_WRITE  2
#define PI_FILE_RW     3
#define PI_FILE_APPEND 4
#define PI_FILE_CREATE 8
#define PI_FILE_TRUNC  16
#define PI_FILE_MAX    31

#define PI_FROM_START   0
#define PI_FROM_CURRENT 1
#define PI_FROM_END     2

/* Allowed socket connect addresses */

#define MAX_CONNECT_ADDRESSES 256

/* events */

#define PI_MAX_EVENT 31

/* Event auto generated on BSC slave activity */

#define PI_EVENT_BSC 31

#define PI_INIT_FAILED       -1 // gpioInitialise failed
#define PI_BAD_USER_GPIO     -2 // GPIO not 0-31
#define PI_BAD_GPIO          -3 // GPIO not 0-53
#define PI_BAD_MODE          -4 // mode not 0-7
#define PI_BAD_LEVEL         -5 // level not 0-1
#define PI_BAD_PUD           -6 // pud not 0-2
#define PI_BAD_PULSEWIDTH    -7 // pulsewidth not 0 or 500-2500
#define PI_BAD_DUTYCYCLE     -8 // dutycycle outside set range
#define PI_BAD_TIMER         -9 // timer not 0-9
#define PI_BAD_MS           -10 // ms not 10-60000
#define PI_BAD_TIMETYPE     -11 // timetype not 0-1
#define PI_BAD_SECONDS      -12 // seconds < 0
#define PI_BAD_MICROS       -13 // micros not 0-999999
#define PI_TIMER_FAILED     -14 // gpioSetTimerFunc failed
#define PI_BAD_WDOG_TIMEOUT -15 // timeout not 0-60000
#define PI_NO_ALERT_FUNC    -16 // DEPRECATED
#define PI_BAD_CLK_PERIPH   -17 // clock peripheral not 0-1
#define PI_BAD_CLK_SOURCE   -18 // DEPRECATED
#define PI_BAD_CLK_MICROS   -19 // clock micros not 1, 2, 4, 5, 8, or 10
#define PI_BAD_BUF_MILLIS   -20 // buf millis not 100-10000
#define PI_BAD_DUTYRANGE    -21 // dutycycle range not 25-40000
#define PI_BAD_DUTY_RANGE   -21 // DEPRECATED (use PI_BAD_DUTYRANGE)
#define PI_BAD_SIGNUM       -22 // signum not 0-63
#define PI_BAD_PATHNAME     -23 // can't open pathname
#define PI_NO_HANDLE        -24 // no handle available
#define PI_BAD_HANDLE       -25 // unknown handle
#define PI_BAD_IF_FLAGS     -26 // ifFlags > 4
#define PI_BAD_CHANNEL      -27 // DMA channel not 0-15
#define PI_BAD_PRIM_CHANNEL -27 // DMA primary channel not 0-15
#define PI_BAD_SOCKET_PORT  -28 // socket port not 1024-32000
#define PI_BAD_FIFO_COMMAND -29 // unrecognized fifo command
#define PI_BAD_SECO_CHANNEL -30 // DMA secondary channel not 0-15
#define PI_NOT_INITIALISED  -31 // function called before gpioInitialise
#define PI_INITIALISED      -32 // function called after gpioInitialise
#define PI_BAD_WAVE_MODE    -33 // waveform mode not 0-3
#define PI_BAD_CFG_INTERNAL -34 // bad parameter in gpioCfgInternals call
#define PI_BAD_WAVE_BAUD    -35 // baud rate not 50-250K(RX)/50-1M(TX)
#define PI_TOO_MANY_PULSES  -36 // waveform has too many pulses
#define PI_TOO_MANY_CHARS   -37 // waveform has too many chars
#define PI_NOT_SERIAL_GPIO  -38 // no bit bang serial read on GPIO
#define PI_BAD_SERIAL_STRUC -39 // bad (null) serial structure parameter
#define PI_BAD_SERIAL_BUF   -40 // bad (null) serial buf parameter
#define PI_NOT_PERMITTED    -41 // GPIO operation not permitted
#define PI_SOME_PERMITTED   -42 // one or more GPIO not permitted
#define PI_BAD_WVSC_COMMND  -43 // bad WVSC subcommand
#define PI_BAD_WVSM_COMMND  -44 // bad WVSM subcommand
#define PI_BAD_WVSP_COMMND  -45 // bad WVSP subcommand
#define PI_BAD_PULSELEN     -46 // trigger pulse length not 1-100
#define PI_BAD_SCRIPT       -47 // invalid script
#define PI_BAD_SCRIPT_ID    -48 // unknown script id
#define PI_BAD_SER_OFFSET   -49 // add serial data offset > 30 minutes
#define PI_GPIO_IN_USE      -50 // GPIO already in use
#define PI_BAD_SERIAL_COUNT -51 // must read at least a byte at a time
#define PI_BAD_PARAM_NUM    -52 // script parameter id not 0-9
#define PI_DUP_TAG          -53 // script has duplicate tag
#define PI_TOO_MANY_TAGS    -54 // script has too many tags
#define PI_BAD_SCRIPT_CMD   -55 // illegal script command
#define PI_BAD_VAR_NUM      -56 // script variable id not 0-149
#define PI_NO_SCRIPT_ROOM   -57 // no more room for scripts
#define PI_NO_MEMORY        -58 // can't allocate temporary memory
#define PI_SOCK_READ_FAILED -59 // socket read failed
#define PI_SOCK_WRIT_FAILED -60 // socket write failed
#define PI_TOO_MANY_PARAM   -61 // too many script parameters (> 10)
#define PI_NOT_HALTED       -62 // DEPRECATED
#define PI_SCRIPT_NOT_READY -62 // script initialising
#define PI_BAD_TAG          -63 // script has unresolved tag
#define PI_BAD_MICS_DELAY   -64 // bad MICS delay (too large)
#define PI_BAD_MILS_DELAY   -65 // bad MILS delay (too large)
#define PI_BAD_WAVE_ID      -66 // non existent wave id
#define PI_TOO_MANY_CBS     -67 // No more CBs for waveform
#define PI_TOO_MANY_OOL     -68 // No more OOL for waveform
#define PI_EMPTY_WAVEFORM   -69 // attempt to create an empty waveform
#define PI_NO_WAVEFORM_ID   -70 // no more waveforms
#define PI_I2C_OPEN_FAILED  -71 // can't open I2C device
#define PI_SER_OPEN_FAILED  -72 // can't open serial device
#define PI_SPI_OPEN_FAILED  -73 // can't open SPI device
#define PI_BAD_I2C_BUS      -74 // bad I2C bus
#define PI_BAD_I2C_ADDR     -75 // bad I2C address
#define PI_BAD_SPI_CHANNEL  -76 // bad SPI channel
#define PI_BAD_FLAGS        -77 // bad i2c/spi/ser open flags
#define PI_BAD_SPI_SPEED    -78 // bad SPI speed
#define PI_BAD_SER_DEVICE   -79 // bad serial device name
#define PI_BAD_SER_SPEED    -80 // bad serial baud rate
#define PI_BAD_PARAM        -81 // bad i2c/spi/ser parameter
#define PI_I2C_WRITE_FAILED -82 // i2c write failed
#define PI_I2C_READ_FAILED  -83 // i2c read failed
#define PI_BAD_SPI_COUNT    -84 // bad SPI count
#define PI_SER_WRITE_FAILED -85 // ser write failed
#define PI_SER_READ_FAILED  -86 // ser read failed
#define PI_SER_READ_NO_DATA -87 // ser read no data available
#define PI_UNKNOWN_COMMAND  -88 // unknown command
#define PI_SPI_XFER_FAILED  -89 // spi xfer/read/write failed
#define PI_BAD_POINTER      -90 // bad (NULL) pointer
#define PI_NO_AUX_SPI       -91 // no auxiliary SPI on Pi A or B
#define PI_NOT_PWM_GPIO     -92 // GPIO is not in use for PWM
#define PI_NOT_SERVO_GPIO   -93 // GPIO is not in use for servo pulses
#define PI_NOT_HCLK_GPIO    -94 // GPIO has no hardware clock
#define PI_NOT_HPWM_GPIO    -95 // GPIO has no hardware PWM
#define PI_BAD_HPWM_FREQ    -96 // invalid hardware PWM frequency
#define PI_BAD_HPWM_DUTY    -97 // hardware PWM dutycycle not 0-1M
#define PI_BAD_HCLK_FREQ    -98 // invalid hardware clock frequency
#define PI_BAD_HCLK_PASS    -99 // need password to use hardware clock 1
#define PI_HPWM_ILLEGAL    -100 // illegal, PWM in use for main clock
#define PI_BAD_DATABITS    -101 // serial data bits not 1-32
#define PI_BAD_STOPBITS    -102 // serial (half) stop bits not 2-8
#define PI_MSG_TOOBIG      -103 // socket/pipe message too big
#define PI_BAD_MALLOC_MODE -104 // bad memory allocation mode
#define PI_TOO_MANY_SEGS   -105 // too many I2C transaction segments
#define PI_BAD_I2C_SEG     -106 // an I2C transaction segment failed
#define PI_BAD_SMBUS_CMD   -107 // SMBus command not supported by driver
#define PI_NOT_I2C_GPIO    -108 // no bit bang I2C in progress on GPIO
#define PI_BAD_I2C_WLEN    -109 // bad I2C write length
#define PI_BAD_I2C_RLEN    -110 // bad I2C read length
#define PI_BAD_I2C_CMD     -111 // bad I2C command
#define PI_BAD_I2C_BAUD    -112 // bad I2C baud rate, not 50-500k
#define PI_CHAIN_LOOP_CNT  -113 // bad chain loop count
#define PI_BAD_CHAIN_LOOP  -114 // empty chain loop
#define PI_CHAIN_COUNTER   -115 // too many chain counters
#define PI_BAD_CHAIN_CMD   -116 // bad chain command
#define PI_BAD_CHAIN_DELAY -117 // bad chain delay micros
#define PI_CHAIN_NESTING   -118 // chain counters nested too deeply
#define PI_CHAIN_TOO_BIG   -119 // chain is too long
#define PI_DEPRECATED      -120 // deprecated function removed
#define PI_BAD_SER_INVERT  -121 // bit bang serial invert not 0 or 1
#define PI_BAD_EDGE        -122 // bad ISR edge value, not 0-2
#define PI_BAD_ISR_INIT    -123 // bad ISR initialisation
#define PI_BAD_FOREVER     -124 // loop forever must be last command
#define PI_BAD_FILTER      -125 // bad filter parameter
#define PI_BAD_PAD         -126 // bad pad number
#define PI_BAD_STRENGTH    -127 // bad pad drive strength
#define PI_FIL_OPEN_FAILED -128 // file open failed
#define PI_BAD_FILE_MODE   -129 // bad file mode
#define PI_BAD_FILE_FLAG   -130 // bad file flag
#define PI_BAD_FILE_READ   -131 // bad file read
#define PI_BAD_FILE_WRITE  -132 // bad file write
#define PI_FILE_NOT_ROPEN  -133 // file not open for read
#define PI_FILE_NOT_WOPEN  -134 // file not open for write
#define PI_BAD_FILE_SEEK   -135 // bad file seek
#define PI_NO_FILE_MATCH   -136 // no files match pattern
#define PI_NO_FILE_ACCESS  -137 // no permission to access file
#define PI_FILE_IS_A_DIR   -138 // file is a directory
#define PI_BAD_SHELL_STATUS -139 // bad shell return status
#define PI_BAD_SCRIPT_NAME -140 // bad script name
#define PI_BAD_SPI_BAUD    -141 // bad SPI baud rate, not 50-500k
#define PI_NOT_SPI_GPIO    -142 // no bit bang SPI in progress on GPIO
#define PI_BAD_EVENT_ID    -143 // bad event id
#define PI_CMD_INTERRUPTED -144 // Used by Python
#define PI_NOT_ON_BCM2711  -145 // not available on BCM2711
#define PI_ONLY_ON_BCM2711 -146 // only available on BCM2711

#define PI_PIGIF_ERR_0    -2000
#define PI_PIGIF_ERR_99   -2099

#define PI_CUSTOM_ERR_0   -3000
#define PI_CUSTOM_ERR_999 -3999

#define PI_DEFAULT_BUFFER_MILLIS           120
#define PI_DEFAULT_CLK_MICROS              5
#define PI_DEFAULT_CLK_PERIPHERAL          PI_CLOCK_PCM
#define PI_DEFAULT_IF_FLAGS                3
#define PI_DEFAULT_FOREGROUND              0
#define PI_DEFAULT_DMA_CHANNEL             14
#define PI_DEFAULT_DMA_PRIMARY_CHANNEL     14
#define PI_DEFAULT_DMA_SECONDARY_CHANNEL   6
#define PI_DEFAULT_DMA_PRIMARY_CH_2711     7
#define PI_DEFAULT_DMA_SECONDARY_CH_2711   6
#define PI_DEFAULT_DMA_NOT_SET             15
#define PI_DEFAULT_SOCKET_PORT             8888
#define PI_DEFAULT_SOCKET_PORT_STR         "8888"
#define PI_DEFAULT_SOCKET_ADDR_STR         "127.0.0.1"
#define PI_DEFAULT_UPDATE_MASK_UNKNOWN     0x0000000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_B1          0x03E7CF93
#define PI_DEFAULT_UPDATE_MASK_A_B2        0xFBC7CF9C
#define PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS 0x0080480FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_ZERO        0x0080000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_PI2B        0x0080480FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_PI3B        0x0000000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_PI4B        0x0000000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_COMPUTE     0x00FFFFFFFFFFFFLL
#define PI_DEFAULT_MEM_ALLOC_MODE          PI_MEM_ALLOC_AUTO

#define PI_DEFAULT_CFG_INTERNALS           0

#ifndef EMBEDDED_IN_VM
#define DBG(level, format, arg...) DO_DBG(level, format, ## arg)
#else
#define DBG(level, format, arg...)
#endif

#define DO_DBG(level, format, arg...)                              \
   {                                                               \
         fprintf(stderr, "%s: " format "\n" ,                   \
            __FUNCTION__ , ## arg);                 \
   }


#define SOFT_ERROR(x, format, arg...)                              \
   do                                                              \
   {                                                               \
	  DBG(DBG_ALWAYS, format, ## arg);                             \
      return x;                                                    \
   }                                                               \
   while (0)

#define CHECK_INITED                                               \
   do                                                              \
   {                                                               \
      if (!libInitialised)                                         \
      {                                                            \
         return PI_NOT_INITIALISED;                                \
      }                                                            \
   }                                                               \
   while (0)

#define CHECK_NOT_INITED                                           \
   do                                                              \
   {                                                               \
      if (libInitialised)                                          \
      {                                                            \
         return PI_INITIALISED;                                    \
      }                                                            \
   }                                                               \
   while (0)

#define CHECK_INITED_RET_NULL_PTR                                  \
   do                                                              \
   {                                                               \
      if (!libInitialised)                                         \
      {                                                            \
         return (NULL);                                            \
      }                                                            \
   }                                                               \
   while (0)

#define CHECK_INITED_RET_NIL                                       \
   do                                                              \
   {                                                               \
      if (!libInitialised)                                         \
      {                                                            \
      		return;												   \
	  }                                                            \
   }                                                               \
   while (0)


#define TIMER_ADD(a, b, result)                                    \
   do                                                              \
   {                                                               \
      (result)->tv_sec =  (a)->tv_sec  + (b)->tv_sec;              \
      (result)->tv_nsec = (a)->tv_nsec + (b)->tv_nsec;             \
      if ((result)->tv_nsec >= BILLION)                            \
      {                                                            \
        ++(result)->tv_sec;                                        \
        (result)->tv_nsec -= BILLION;                              \
      }                                                            \
   }                                                               \
   while (0)

#define TIMER_SUB(a, b, result)                                    \
   do                                                              \
   {                                                               \
      (result)->tv_sec =  (a)->tv_sec  - (b)->tv_sec;              \
      (result)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec;             \
      if ((result)->tv_nsec < 0)                                   \
      {                                                            \
         --(result)->tv_sec;                                       \
         (result)->tv_nsec += BILLION;                             \
      }                                                            \
   }                                                               \
   while (0)
#endif
