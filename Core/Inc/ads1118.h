#ifndef INC_ADS1118_H_
#define INC_ADS1118_H_

#include "stdint.h"
#include "stdbool.h"
#include "spi.h"

#define CONFIG_BIT_RESV 1

// ADS Struct
typedef struct ADS {
	SPI_HandleTypeDef *hspi;
	float FSR;
	unsigned int SPS;

	GPIO_TypeDef* GPIO_PORT;
	uint8_t GPIO_PIN;
	void (*cs_low)(struct ADS*);
	void (*cs_high)(struct ADS*);

	uint8_t rxADS[2];
	uint8_t rxConfig[4];

	float voltage;
	float temp;

	union {
		uint16_t word;
		uint8_t bytes[2];
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
	} config;
} ADS;

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

void cs_low_stm(struct ADS*);
void cs_high_stm(struct ADS*);
void cs_low_pcf(struct ADS*);
void cs_high_pcf(struct ADS*);

void initADS(ADS* adsInstance, SPI_HandleTypeDef* spiInstance, GPIO_TypeDef* PORT, uint16_t GPIO_PIN);
bool resetConfig(ADS* ads);
bool editConfig(ADS* ads, uint16_t prevConfig);
bool enableSingleshot(ADS* ads);
bool enableContinuousConversion(ADS* ads);

bool enableADCSensor(ADS* ads);
bool enableTempSensor(ADS* ads);

bool continuousRead(ADS* ads);
bool singleshotRead(ADS* ads);

bool enableAINPN_0_1(ADS* ads);
bool enableAINPN_0_3(ADS* ads);
bool enableAINPN_1_3(ADS* ads);
bool enableAINPN_2_3(ADS* ads);
bool enableAINPN_0_G(ADS* ads);
bool enableAINPN_1_G(ADS* ads);
bool enableAINPN_2_G(ADS* ads);
bool enableAINPN_3_G(ADS* ads);

bool enableFSR_6144(ADS* ads);
bool enableFSR_4096(ADS* ads);
bool enableFSR_2048(ADS* ads);
bool enableFSR_1024(ADS* ads);
bool enableFSR_0512(ADS* ads);
bool enableFSR_0256(ADS* ads);

bool enableSPS_8(ADS* ads);
bool enableSPS_16(ADS* ads);
bool enableSPS_32(ADS* ads);
bool enableSPS_64(ADS* ads);
bool enableSPS_128(ADS* ads);
bool enableSPS_250(ADS* ads);
bool enableSPS_475(ADS* ads);
bool enableSPS_860(ADS* ads);

float parseVoltage(ADS* adsInstance, int16_t adsReading);
float parseTemp(int16_t adsReading);

#endif /* SRC_ADS1118_H_ */
