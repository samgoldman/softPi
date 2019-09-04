#include "softPi.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>

#define NUMBER_GPIO_PINS 32

#define GPIO_REC_PORT 42420
#define GPIO_SND_PORT 42421
#define BACKLOG 10

#define STACK_SIZE (256*1024)

static int libInitialised = PI_OFF;

uint32_t gpioLevel[NUMBER_GPIO_PINS];
uint32_t gpioMode[NUMBER_GPIO_PINS];
uint32_t gpioPwmDutycycle[NUMBER_GPIO_PINS];
uint32_t gpioRisingEdgeEnabled[NUMBER_GPIO_PINS];
uint32_t gpioFallingEdgeEnabled[NUMBER_GPIO_PINS];
uint32_t gpioPullUpDown[NUMBER_GPIO_PINS];
uint32_t gpioAlertPins[NUMBER_GPIO_PINS];
pthread_t gpioInterruptThread[NUMBER_GPIO_PINS];
pthread_cond_t gpioInterruptCond[NUMBER_GPIO_PINS];
pthread_mutex_t gpioMutex[NUMBER_GPIO_PINS];
gpioAlert_t gpioAlertData [NUMBER_GPIO_PINS];

pthread_t *gpioServerThread;
pthread_t *gpioAlertThread;

static struct timespec libStarted;

uint32_t tick = 0;

typedef struct
{
    uint32_t gpioLevel[NUMBER_GPIO_PINS];
	uint32_t gpioMode[NUMBER_GPIO_PINS];
	uint32_t gpioPwmDutycycle[NUMBER_GPIO_PINS];
} gpioMessage_t, *gpioMessage_p;

void *gpioAlertMonitor(void *x)
{
    uint32_t last_gpioLevel[NUMBER_GPIO_PINS];
    uint32_t beforeLast_gpioLevel[NUMBER_GPIO_PINS];

    memcpy(last_gpioLevel, gpioLevel, sizeof(uint32_t) * NUMBER_GPIO_PINS);
    memcpy(beforeLast_gpioLevel, gpioLevel, sizeof(uint32_t) * NUMBER_GPIO_PINS);
    
    while (1)
    {
        for (int gpio = 0; gpio < NUMBER_GPIO_PINS; gpio++)
        {
            
	        if (PI_ON == gpioAlertPins[gpio] && (gpioLevel[gpio] != last_gpioLevel[gpio] || gpioLevel[gpio] != beforeLast_gpioLevel[gpio]))
	        {
		        if (gpioAlertData[gpio].func)
		        {
			        if (gpioAlertData[gpio].ex)
			        {
				        (gpioAlertData[gpio].func)(gpio, gpioLevel[gpio], gpioTick(), gpioAlertData[gpio].userdata);
			        }
			        else
			        {
				        (gpioAlertData[gpio].func)(gpio, gpioLevel[gpio], gpioTick());
			        }
		        }
	        }		
        }
        memcpy(beforeLast_gpioLevel, last_gpioLevel, sizeof(uint32_t) * NUMBER_GPIO_PINS);
        memcpy(last_gpioLevel, gpioLevel, sizeof(uint32_t) * NUMBER_GPIO_PINS);
    }
    
}

void *gpioServer(void *x)
{
    int sockfd, new_fd;  /* listen on sock_fd, new connection on new_fd */
    struct sockaddr_in my_addr;    /* my address information */
    struct sockaddr_in their_addr; /* connector's address information */
    int numbytes;
    socklen_t sin_size;

    gpioMessage_t buf;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(1);
    }

    my_addr.sin_family = AF_INET;         /* host byte order */
    my_addr.sin_port = htons(GPIO_SND_PORT);     /* short, network byte order */
    my_addr.sin_addr.s_addr = INADDR_ANY; /* auto-fill with my IP */
    bzero(&(my_addr.sin_zero), 8);        /* zero the rest of the struct */

    if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) {
        perror("bind");
        exit(1);
    }

    if (listen(sockfd, BACKLOG) == -1) {
        perror("listen");
        exit(1);
    }
   
	sin_size = sizeof(struct sockaddr_in);
    if ((new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size)) == -1) {
        perror("accept");
    }

    while(1) {
		memcpy(&buf.gpioLevel, gpioLevel, sizeof(uint32_t) * NUMBER_GPIO_PINS);
		memcpy(&buf.gpioMode, gpioMode, sizeof(uint32_t) * NUMBER_GPIO_PINS);
		memcpy(&buf.gpioPwmDutycycle, gpioPwmDutycycle, sizeof(uint32_t) * NUMBER_GPIO_PINS);

    	if ((numbytes=send(new_fd, &buf, sizeof(gpioMessage_t), 0)) == -1) {
            perror("send");
            exit(1);
        }

            if ((numbytes=recv(new_fd, &buf, sizeof(gpioMessage_t), 0)) == -1) {
                perror("recv");
                exit(1);
            }
            for (int i = 0; i < NUMBER_GPIO_PINS; i++)
            {
                if (PI_INPUT == gpioMode[i])
                {
                    gpioWrite(i, buf.gpioLevel[i]);
                }
            } 
        
    }

    close(sockfd);
}

int sim_gpioInitialise(void)
{
	int i;
	for (i = 0; i < NUMBER_GPIO_PINS; i++)
	{
		gpioLevel[i] = PI_OFF;
		gpioMode[i] = PI_INPUT;
		gpioRisingEdgeEnabled[i] = PI_OFF;
		gpioFallingEdgeEnabled[i] = PI_OFF;
		gpioPullUpDown[i] = PI_PUD_OFF;
		gpioAlertPins[i] = PI_OFF;
		gpioInterruptThread[i] = -1;
		pthread_cond_init(&(gpioInterruptCond[i]), NULL);
		pthread_mutex_init(&(gpioMutex[i]), NULL);
	}
	clock_gettime(CLOCK_MONOTONIC, &libStarted);
    libInitialised = PI_ON;

	gpioServerThread = sim_gpioStartThread(gpioServer, NULL);
    gpioAlertThread = sim_gpioStartThread(gpioAlertMonitor, NULL);

	return 0;
}


void sim_gpioTerminate(void)
{
	gpioStopThread(gpioAlertThread);

	int i;
	for (i = 0; i < NUMBER_GPIO_PINS; i++)
	{
		gpioLevel[i] = 0;
        gpioMode[i] = PI_INPUT;
        gpioRisingEdgeEnabled[i] = PI_OFF;
        gpioFallingEdgeEnabled[i] = PI_OFF;
        gpioPullUpDown[i] = PI_PUD_OFF;
		gpioAlertPins[i] = PI_OFF;
		if (-1 != gpioInterruptThread[i])
		{
			pthread_cancel(gpioInterruptThread[i]);
			gpioInterruptThread[i] = -1;
		}
		pthread_cond_destroy(&(gpioInterruptCond[i]));
		pthread_mutex_destroy(&(gpioMutex[i]));
	}

	libInitialised = PI_OFF;
}


int sim_gpioSetMode(unsigned gpio, unsigned mode)
{
	CHECK_INITED;

	if (gpio >= NUMBER_GPIO_PINS)
		SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);
	
	if (mode > PI_ALT3)
		SOFT_ERROR(PI_BAD_MODE, "gpio %d, bad mode (%d)", gpio, mode);

	gpioMode[gpio] = mode;

	return 0;
}


int sim_gpioGetMode(unsigned gpio)
{
	CHECK_INITED;

	if (gpio >= NUMBER_GPIO_PINS)
		SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);
	
	return gpioMode[gpio];
}


int sim_gpioSetPullUpDown(unsigned gpio, unsigned pud)
{
	CHECK_INITED;

    if (gpio > NUMBER_GPIO_PINS)
        SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

    if (pud > PI_PUD_UP)
        SOFT_ERROR(PI_BAD_PUD, "gpio %d, bad pud (%d)", gpio, pud);

	gpioPullUpDown[gpio] = pud;

	return 0;	
}


int sim_gpioRead(unsigned gpio)
{
	CHECK_INITED;

    if (gpio > NUMBER_GPIO_PINS)
        SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

	return gpioLevel[gpio];	
}


int sim_gpioWrite(unsigned gpio, unsigned level)
{
	CHECK_INITED;

    if (gpio > NUMBER_GPIO_PINS)
        SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

	if (level > PI_ON)
		SOFT_ERROR(PI_BAD_LEVEL, "gpio %d, bad level (%d)", gpio, level);

	//sim_gpioSetMode(gpio, PI_OUTPUT);

	gpioLevel[gpio] = level;

	return 0;
}


int sim_gpioPWM(unsigned gpio, unsigned dutycycle)
{
	CHECK_INITED;

	if (gpio > NUMBER_GPIO_PINS)
		SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

	//if (val > gpioInfo[gpio].range)
    //  SOFT_ERROR(PI_BAD_DUTYCYCLE, "gpio %d, bad dutycycle (%d)", gpio, val);

	sim_gpioSetMode(gpio, PI_OUTPUT);

	if (!dutycycle)
		sim_gpioWrite(gpio, PI_OFF);

	gpioPwmDutycycle[gpio] = dutycycle;

    return 0;
}


int sim_gpioGetPWMdutycycle(unsigned gpio)
{
	CHECK_INITED;

	if (gpio > NUMBER_GPIO_PINS)
        SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

	return gpioPwmDutycycle[gpio];
}


int sim_gpioSetPWMrange(unsigned gpio, unsigned range)
{
	printf("sim_gpioSetPWMrange is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioGetPWMrange(unsigned gpio)
{
	printf("sim_gpioGetPWMrange is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioGetPWMrealRange(unsigned gpio)
{
	printf("sim_gpioGetPWMrealRange is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSetPWMfrequency(unsigned gpio, unsigned frequency)
{
	printf("sim_gpioSetPWMfrequency is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioGetPWMfrequency(unsigned gpio)
{
	printf("sim_gpioGetPWMfrequency is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioServo(unsigned gpio, unsigned pulsewidth)
{
	printf("sim_gpioServo is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioGetServoPulsewidth(unsigned gpio)
{
	printf("sim_gpioGetServoPulsewidth is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSetAlertFunc(unsigned gpio, gpioAlertFunc_t f)
{
	CHECK_INITED;

	if (gpio > NUMBER_GPIO_PINS)
		SOFT_ERROR(PI_BAD_USER_GPIO, "bad gpio (%d)", gpio);

	gpioAlertData[gpio].ex = 0;
	gpioAlertData[gpio].userdata = NULL;
	gpioAlertData[gpio].func = f;

	if (f)
	{
		gpioAlertPins[gpio] = PI_ON;
	}
	else
	{
		gpioAlertPins[gpio] = PI_OFF;
	}

	return 0;
}


int sim_gpioNotifyOpen(void)
{
	printf("sim_gpioNotifyOpen is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioNotifyOpenWithSize(int bufSize)
{
	printf("sim_gpioNotifyOpenWithSize is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioNotifyBegin(unsigned handle, uint32_t bits)
{
	printf("sim_gpioNotifyBegin is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioNotifyPause(unsigned handle)
{
	printf("sim_gpioNotifyPause is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioNotifyClose(unsigned handle)
{
	printf("sim_gpioNotifyClose is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveClear(void)
{
	printf("sim_gpioWaveClear is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveAddNew(void)
{
	printf("sim_gpioWaveAddNew is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveAddGeneric(unsigned numPulses, gpioPulse_t *pulses)
{
	printf("sim_gpioWaveAddGeneric is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveCreate(void)
{
	printf("sim_gpioWaveCreate is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveDelete(unsigned wave_id)
{
	printf("sim_gpioWaveDelete is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveTxSend(unsigned wave_id, unsigned wave_mode)
{
	printf("sim_gpioWaveTxSend is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveChain(char *buf, unsigned bufSize)
{
	printf("sim_gpioWaveChain is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveTxAt(void)
{
	printf("sim_gpioWaveTxAt is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveTxBusy(void)
{
	printf("sim_gpioWaveTxBusy is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveTxStop(void)
{
	printf("sim_gpioWaveTxStop is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveGetMicros(void)
{
	printf("sim_gpioWaveGetMicros is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveGetHighMicros(void)
{
	printf("sim_gpioWaveGetHighMicros is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveGetMaxMicros(void)
{
	printf("sim_gpioWaveGetMaxMicros is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveGetPulses(void)
{
	printf("sim_gpioWaveGetPulses is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveGetHighPulses(void)
{
	printf("sim_gpioWaveGetHighPulses is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveGetMaxPulses(void)
{
	printf("sim_gpioWaveGetMaxPulses is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveGetCbs(void)
{
	printf("sim_gpioWaveGetCbs is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveGetHighCbs(void)
{
	printf("sim_gpioWaveGetHighCbs is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWaveGetMaxCbs(void)
{
	printf("sim_gpioWaveGetMaxCbs is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSerialReadOpen(unsigned gpio, unsigned baud, unsigned data_bits)
{
	printf("sim_gpioSerialReadOpen is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSerialReadInvert(unsigned gpio, unsigned invert)
{
	printf("sim_gpioSerialReadInvert is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSerialRead(unsigned gpio, void *buf, size_t bufSize)
{
	printf("sim_gpioSerialRead is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSerialReadClose(unsigned gpio)
{
	printf("sim_gpioSerialReadClose is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cOpen(unsigned i2cBus, unsigned i2cAddr, unsigned i2cFlags)
{
	printf("sim_i2cOpen is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cClose(unsigned handle)
{
	printf("sim_i2cClose is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cWriteQuick(unsigned handle, unsigned bit)
{
	printf("sim_i2cWriteQuick is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cWriteByte(unsigned handle, unsigned bVal)
{
	printf("sim_i2cWriteByte is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cReadByte(unsigned handle)
{
	printf("sim_i2cReadByte is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cWriteByteData(unsigned handle, unsigned i2cReg, unsigned bVal)
{
	printf("sim_i2cWriteByteData is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cWriteWordData(unsigned handle, unsigned i2cReg, unsigned wVal)
{
	printf("sim_i2cWriteWordData is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cReadByteData(unsigned handle, unsigned i2cReg)
{
	printf("sim_i2cReadByteData is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cReadWordData(unsigned handle, unsigned i2cReg)
{
	printf("sim_i2cReadWordData is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cProcessCall(unsigned handle, unsigned i2cReg, unsigned wVal)
{
	printf("sim_i2cProcessCall is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cReadBlockData(unsigned handle, unsigned i2cReg, char *buf)
{
	printf("sim_i2cReadBlockData is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cReadDevice(unsigned handle, char *buf, unsigned count)
{
	printf("sim_i2cReadDevice is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cWriteDevice(unsigned handle, char *buf, unsigned count)
{
	printf("sim_i2cWriteDevice is not yet supported. Exiting...\n");
	exit(1);
}


void sim_i2cSwitchCombined(int setting)
{
	printf("sim_i2cSwitchCombined is not yet supported. Exiting...\n");
	exit(1);
}


int sim_i2cSegments(unsigned handle, pi_i2c_msg_t *segs, unsigned numSegs)
{
	printf("sim_i2cSegments is not yet supported. Exiting...\n");
	exit(1);
}


int sim_bbI2COpen(unsigned SDA, unsigned SCL, unsigned baud)
{
	printf("sim_bbI2COpen is not yet supported. Exiting...\n");
	exit(1);
}


int sim_bbI2CClose(unsigned SDA)
{
	printf("sim_bbI2CClose is not yet supported. Exiting...\n");
	exit(1);
}


int sim_bscXfer(bsc_xfer_t *bsc_xfer)
{
	printf("sim_bscXfer is not yet supported. Exiting...\n");
	exit(1);
}


int sim_bbSPIClose(unsigned CS)
{
	printf("sim_bbSPIClose is not yet supported. Exiting...\n");
	exit(1);
}


int sim_spiOpen(unsigned spiChan, unsigned baud, unsigned spiFlags)
{
	printf("sim_spiOpen is not yet supported. Exiting...\n");
	exit(1);
}


int sim_spiClose(unsigned handle)
{
	printf("sim_spiClose is not yet supported. Exiting...\n");
	exit(1);
}


int sim_spiRead(unsigned handle, char *buf, unsigned count)
{
	printf("sim_spiRead is not yet supported. Exiting...\n");
	exit(1);
}


int sim_spiWrite(unsigned handle, char *buf, unsigned count)
{
	printf("sim_spiWrite is not yet supported. Exiting...\n");
	exit(1);
}


int sim_spiXfer(unsigned handle, char *txBuf, char *rxBuf, unsigned count)
{
	printf("sim_spiXfer is not yet supported. Exiting...\n");
	exit(1);
}


int sim_serOpen(char *sertty, unsigned baud, unsigned serFlags)
{
	printf("sim_serOpen is not yet supported. Exiting...\n");
	exit(1);
}


int sim_serClose(unsigned handle)
{
	printf("sim_serClose is not yet supported. Exiting...\n");
	exit(1);
}


int sim_serWriteByte(unsigned handle, unsigned bVal)
{
	printf("sim_serWriteByte is not yet supported. Exiting...\n");
	exit(1);
}


int sim_serReadByte(unsigned handle)
{
	printf("sim_serReadByte is not yet supported. Exiting...\n");
	exit(1);
}


int sim_serWrite(unsigned handle, char *buf, unsigned count)
{
	printf("sim_serWrite is not yet supported. Exiting...\n");
	exit(1);
}


int sim_serRead(unsigned handle, char *buf, unsigned count)
{
	printf("sim_serRead is not yet supported. Exiting...\n");
	exit(1);
}


int sim_serDataAvailable(unsigned handle)
{
	printf("sim_serDataAvailable is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioTrigger(unsigned gpio, unsigned pulseLen, unsigned level)
{
	printf("sim_gpioTrigger is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSetWatchdog(unsigned gpio, unsigned timeout)
{
	printf("sim_gpioSetWatchdog is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioNoiseFilter(unsigned gpio, unsigned steady, unsigned active)
{
	printf("sim_gpioNoiseFilter is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioGlitchFilter(unsigned gpio, unsigned steady)
{
	printf("sim_gpioGlitchFilter is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSetGetSamplesFunc(gpioGetSamplesFunc_t f, uint32_t bits)
{
	printf("sim_gpioSetGetSamplesFunc is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSetTimerFunc(unsigned timer, unsigned millis, gpioTimerFunc_t f)
{
	printf("sim_gpioSetTimerFunc is not yet supported. Exiting...\n");
	exit(1);
}


pthread_t *sim_gpioStartThread(gpioThreadFunc_t f, void *userdata)
{
	printf("%s\n", __FUNCTION__);
	pthread_t *pth;
    pthread_attr_t pthAttr;

    CHECK_INITED_RET_NULL_PTR;

    pth = malloc(sizeof(pthread_t));

    if (pth)
    {
        if (pthread_attr_init(&pthAttr))
		{
			free(pth);
			printf("pthread_attr_init failed\n");
        }

        if (pthread_attr_setstacksize(&pthAttr, STACK_SIZE))
        {
            free(pth);
			printf("pthread_attr_setstacksize failed\n");
        }

        if (pthread_create(pth, &pthAttr, f, userdata))
        {
            free(pth);
			printf("create failed\n");
        }
    }

	printf("Thread started...\n");
    return pth;
}


void sim_gpioStopThread(pthread_t *pth)
{
    CHECK_INITED_RET_NIL;

    if (pth)
    {
        if (pthread_self() == *pth)
        {
            free(pth);
            pthread_exit(NULL);
        }
        else
        {
            pthread_cancel(*pth);
            pthread_join(*pth, NULL);
            free(pth);
   		}
	}
}


int sim_gpioStoreScript(char *script)
{
	printf("sim_gpioStoreScript is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioRunScript(unsigned script_id, unsigned numPar, uint32_t *param)
{
	printf("sim_gpioRunScript is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioUpdateScript(unsigned script_id, unsigned numPar, uint32_t *param)
{
	printf("sim_gpioUpdateScript is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioScriptStatus(unsigned script_id, uint32_t *param)
{
	printf("sim_gpioScriptStatus is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioStopScript(unsigned script_id)
{
	printf("sim_gpioStopScript is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioDeleteScript(unsigned script_id)
{
	printf("sim_gpioDeleteScript is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSetSignalFunc(unsigned signum, gpioSignalFunc_t f)
{
	printf("sim_gpioSetSignalFunc is not yet supported. Exiting...\n");
	exit(1);
}


uint32_t sim_gpioRead_Bits_0_31(void)
{
	printf("sim_gpioRead_Bits_0_31 is not yet supported. Exiting...\n");
	exit(1);
}


uint32_t sim_gpioRead_Bits_32_53(void)
{
	printf("sim_gpioRead_Bits_32_53 is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWrite_Bits_0_31_Clear(uint32_t bits)
{
	printf("sim_gpioWrite_Bits_0_31_Clear is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWrite_Bits_32_53_Clear(uint32_t bits)
{
	printf("sim_gpioWrite_Bits_32_53_Clear is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWrite_Bits_0_31_Set(uint32_t bits)
{
	printf("sim_gpioWrite_Bits_0_31_Set is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioWrite_Bits_32_53_Set(uint32_t bits)
{
	printf("sim_gpioWrite_Bits_32_53_Set is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioHardwareClock(unsigned gpio, unsigned clkfreq)
{
	printf("sim_gpioHardwareClock is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioHardwarePWM(unsigned gpio, unsigned PWMfreq, unsigned PWMduty)
{
	printf("sim_gpioHardwarePWM is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioTime(unsigned timetype, int *seconds, int *micros)
{
   struct timespec ts;

   CHECK_INITED;

   if (timetype > PI_TIME_ABSOLUTE)
      SOFT_ERROR(PI_BAD_TIMETYPE, "bad timetype (%d)", timetype);

   if (timetype == PI_TIME_ABSOLUTE)
   {
      clock_gettime(CLOCK_MONOTONIC, &ts);
      *seconds = ts.tv_sec;
      *micros  = ts.tv_nsec/1000;
   }
   else
   {
      clock_gettime(CLOCK_MONOTONIC, &ts);

      TIMER_SUB(&ts, &libStarted, &ts);

      *seconds = ts.tv_sec;
      *micros  = ts.tv_nsec/1000;
   }

   return 0;
}


int sim_gpioSleep(unsigned timetype, int seconds, int micros)
{
   struct timespec ts, rem;

   CHECK_INITED;

   if (timetype > PI_TIME_ABSOLUTE)
      SOFT_ERROR(PI_BAD_TIMETYPE, "bad timetype (%d)", timetype);

   if (seconds < 0)
      SOFT_ERROR(PI_BAD_SECONDS, "bad seconds (%d)", seconds);

   if ((micros < 0) || (micros > 999999))
      SOFT_ERROR(PI_BAD_MICROS, "bad micros (%d)", micros);

   ts.tv_sec  = seconds;
   ts.tv_nsec = micros * 1000;

   if (timetype == PI_TIME_ABSOLUTE)
   {
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &rem));
   }
   else
   {
      while (clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, &rem))
      {
         /* copy remaining time to ts */
         ts.tv_sec  = rem.tv_sec;
         ts.tv_nsec = rem.tv_nsec;
      }
   }

   return 0;
}


uint32_t sim_gpioDelay(uint32_t micros)
{
	printf("sim_gpioDelay is not yet supported. Exiting...\n");
	exit(1);
}


uint32_t sim_gpioTick(void)
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return 1000000 * tv.tv_sec + tv.tv_usec;
}


unsigned sim_gpioHardwareRevision(void)
{
	printf("sim_gpioHardwareRevision is not yet supported. Exiting...\n");
	exit(1);
}


unsigned sim_gpioVersion(void)
{
	return 0;	
}


int sim_gpioGetPad(unsigned pad)
{
	printf("sim_gpioGetPad is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioSetPad(unsigned pad, unsigned padStrength)
{
	printf("sim_gpioSetPad is not yet supported. Exiting...\n");
	exit(1);
}


int sim_eventMonitor(unsigned handle, uint32_t bits)
{
	printf("sim_eventMonitor is not yet supported. Exiting...\n");
	exit(1);
}


int sim_eventSetFunc(unsigned event, eventFunc_t f)
{
	printf("sim_eventSetFunc is not yet supported. Exiting...\n");
	exit(1);
}


int sim_eventSetFuncEx(unsigned event, eventFuncEx_t f, void *userdata)
{
	printf("sim_eventSetFuncEx is not yet supported. Exiting...\n");
	exit(1);
}


int sim_eventTrigger(unsigned event)
{
	printf("sim_eventTrigger is not yet supported. Exiting...\n");
	exit(1);
}


int sim_shell(char *scriptName, char *scriptString)
{
	printf("sim_shell is not yet supported. Exiting...\n");
	exit(1);
}


int sim_fileOpen(char *file, unsigned mode)
{
	printf("sim_fileOpen is not yet supported. Exiting...\n");
	exit(1);
}


int sim_fileClose(unsigned handle)
{
	printf("sim_fileClose is not yet supported. Exiting...\n");
	exit(1);
}


int sim_fileWrite(unsigned handle, char *buf, unsigned count)
{
	printf("sim_fileWrite is not yet supported. Exiting...\n");
	exit(1);
}


int sim_fileRead(unsigned handle, char *buf, unsigned count)
{
	printf("sim_fileRead is not yet supported. Exiting...\n");
	exit(1);
}


int sim_fileSeek(unsigned handle, int32_t seekOffset, int seekFrom)
{
	printf("sim_fileSeek is not yet supported. Exiting...\n");
	exit(1);
}


int sim_fileList(char *fpat,  char *buf, unsigned count)
{
	printf("sim_fileList is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgBufferSize(unsigned cfgMillis)
{
	printf("sim_gpioCfgBufferSize is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgDMAchannel(unsigned DMAchannel) /* DEPRECATED */
{
	printf("sim_gpioCfgDMAchannel is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgDMAchannels(unsigned primaryChannel, unsigned secondaryChannel)
{
	printf("sim_gpioCfgDMAchannels is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgPermissions(uint64_t updateMask)
{
	printf("sim_gpioCfgPermissions is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgSocketPort(unsigned port)
{
	printf("sim_gpioCfgSocketPort is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgInterfaces(unsigned ifFlags)
{
	printf("sim_gpioCfgInterfaces is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgMemAlloc(unsigned memAllocMode)
{
	printf("sim_gpioCfgMemAlloc is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgNetAddr(int numSockAddr, uint32_t *sockAddr)
{
	printf("sim_gpioCfgNetAddr is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgInternals(unsigned cfgWhat, unsigned cfgVal)
{
	printf("sim_gpioCfgInternals is not yet supported. Exiting...\n");
	exit(1);
}


uint32_t sim_gpioCfgGetInternals(void)
{
	printf("sim_gpioCfgGetInternals is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCfgSetInternals(uint32_t cfgVal)
{
	printf("sim_gpioCfgSetInternals is not yet supported. Exiting...\n");
	exit(1);
}


int sim_gpioCustom1(unsigned arg1, unsigned arg2, char *argx, unsigned argc)
{
	printf("sim_gpioCustom1 is not yet supported. Exiting...\n");
	exit(1);
}


int sim_rawWaveAddGeneric(unsigned numPulses, rawWave_t *pulses)
{
	printf("sim_rawWaveAddGeneric is not yet supported. Exiting...\n");
	exit(1);
}


unsigned sim_rawWaveCB(void)
{
	printf("sim_rawWaveCB is not yet supported. Exiting...\n");
	exit(1);
}


rawCbs_t *sim_rawWaveCBAdr(int cbNum)
{
	printf("sim_*rawWaveCBAdr is not yet supported. Exiting...\n");
	exit(1);
}


uint32_t sim_rawWaveGetOOL(int pos)
{
	printf("sim_rawWaveGetOOL is not yet supported. Exiting...\n");
	exit(1);
}


void sim_rawWaveSetOOL(int pos, uint32_t lVal)
{
	printf("sim_rawWaveSetOOL is not yet supported. Exiting...\n");
	exit(1);
}


uint32_t sim_rawWaveGetOut(int pos)
{
	printf("sim_rawWaveGetOut is not yet supported. Exiting...\n");
	exit(1);
}


void sim_rawWaveSetOut(int pos, uint32_t lVal)
{
	printf("sim_rawWaveSetOut is not yet supported. Exiting...\n");
	exit(1);
}


uint32_t sim_rawWaveGetIn(int pos)
{
	printf("sim_rawWaveGetIn is not yet supported. Exiting...\n");
	exit(1);
}


void sim_rawWaveSetIn(int pos, uint32_t lVal)
{
	printf("sim_rawWaveSetIn is not yet supported. Exiting...\n");
	exit(1);
}


rawWaveInfo_t sim_rawWaveInfo(int wave_id)
{
	printf("sim_rawWaveInfo is not yet supported. Exiting...\n");
	exit(1);
}


int sim_getBitInBytes(int bitPos, char *buf, int numBits)
{
	printf("sim_getBitInBytes is not yet supported. Exiting...\n");
	exit(1);
}


void sim_putBitInBytes(int bitPos, char *buf, int bit)
{
	printf("sim_putBitInBytes is not yet supported. Exiting...\n");
	exit(1);
}


double sim_time_time(void)
{
	printf("sim_time_time is not yet supported. Exiting...\n");
	exit(1);
}


void sim_time_sleep(double seconds)
{
	printf("sim_time_sleep is not yet supported. Exiting...\n");
	exit(1);
}


void sim_rawDumpWave(void)
{
	printf("sim_rawDumpWave is not yet supported. Exiting...\n");
	exit(1);
}


void sim_rawDumpScript(unsigned script_id)
{
	printf("sim_rawDumpScript is not yet supported. Exiting...\n");
	exit(1);
}

