#ifndef INC_ADS1118_H_
#define INC_ADS1118_H_

#include "stdint.h"
#include "stdbool.h"
#include "spi.h"

#define CONFIG_BIT_RESV 1
#define ADS_EN_PORT GPIOA
#define ADS_EN_PIN GPIO_PIN_4

// The Config Bitfield
typedef union {
	struct {
		volatile unsigned int RESV :1; // Low Bit (0)
		volatile unsigned int NOP :2;
		volatile unsigned int PULL_UP_EN :1;
		volatile unsigned int TS_MODE :1;
		volatile unsigned int DR :3;
		volatile unsigned int MODE :1;
		volatile unsigned int PGA :3;
		volatile unsigned int MUX :3;
		volatile unsigned int SS :1; // High Bit (15)
	} bits;
	volatile uint16_t word;
	volatile uint8_t bytes[2];
} ADS_Config_Bitfield;

// Config Register
typedef enum SS {
	STOPPED = 0x0, START = 0x1
} SS;
typedef enum MUX {
	AINPN_0_1 = 0b000,
	AINPN_0_3 = 0b001,
	AINPN_1_3 = 0b010,
	AINPN_2_3 = 0b011,
	AINPN_0_G = 0b100,
	AINPN_1_G = 0b101,
	AINPN_2_G = 0b110,
	AINPN_3_G = 0b111
} MUX;
typedef enum PGA {
	FSR_6144 = 0b000,
	FSR_4096 = 0b001,
	FSR_2048 = 0b010,
	FSR_1024 = 0b011,
	FSR_0512 = 0b100,
	FSR_0256 = 0b101
} PGA;
typedef enum MODE { // Continuous Conversion (CC) & Single Shot (SS)
	CC_EN = 0x0,
	SS_EN = 0x1
} MODE;
typedef enum DR {
	SPS_8 = 0b000,
	SPS_16 = 0b001,
	SPS_32 = 0b010,
	SPS_64 = 0b011,
	SPS_128 = 0b100,
	SPS_250 = 0b101,
	SPS_475 = 0b110,
	SPS_860 = 0b111
} DR;
typedef enum TS_MODE {
	ADC_MODE = 0x0, TEMP_MODE = 0x1
} TS_MODE;
typedef enum PULL_UP_EN {
	DISABLED = 0x0, ENABLED = 0x1
} PULL_UP_EN;
typedef enum NOP {
	DATA_VALID = 0b01, DATA_INVALID = 0b10
} NOP;

// Simple ADS Struct
typedef struct ADS {
	SPI_HandleTypeDef *hspi;
	ADS_Config_Bitfield configReg;
	float FSR;
	unsigned int SPS;
} ADS;

void initADS_SW(ADS* adsInstance, SPI_HandleTypeDef* spiInstance);
bool initADS_HW(ADS* adsInstance, uint8_t* rxData);

bool resetConfig(ADS* ads, uint8_t* rxData);
bool editConfig(ADS* ads, uint8_t* rxData);
bool enableSingleshot(ADS* adsInstance, uint8_t* rxData);
bool enable_AIN0_SE(ADS* adsInstance, uint8_t* rxData);

bool singleshotRead(ADS* adsInstance, uint8_t* rxData, float* voltage);
float parseVoltage(ADS* adsInstance, uint16_t adsReading);

#endif /* SRC_ADS1118_H_ */
