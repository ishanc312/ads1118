/*
 * ads1118.c
 *
 *  Created on: Apr 16, 2025
 *      Author: ishanchitale
 */


#include "ads1118.h"

void cs_low_stm(struct ADS* ads) {
	HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_RESET);
}

void cs_high_stm(struct ADS* ads) {
	HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_SET);
}

void initADS(ADS* ads, SPI_HandleTypeDef* spiInstance, GPIO_TypeDef* PORT, uint16_t GPIO_PIN) {
	// Important Parameters
	ads->hspi = spiInstance;
	ads->FSR = 2.048; // Full Scale Range
	ads->SPS = 128; // Data Rate, i.e. Samples Per Second
	ads->GPIO_PORT = PORT; // The GPIO Port on our STM32 for our chip select
	ads->GPIO_PIN = GPIO_PIN; // The GPIO Pin for our chip select; could be either STM32 or on the PCF8574A

	ads->cs_low = &cs_low_stm;
	ads->cs_high = &cs_high_stm;

	// Default Config according to Datasheet
	ads->config.bits.RESV = CONFIG_BIT_RESV;
	ads->config.bits.NOP = DATA_VALID;
	ads->config.bits.PULL_UP_EN = ENABLED;
	ads->config.bits.TS_MODE = ADC_MODE;
	ads->config.bits.DR = SPS_128;
	ads->config.bits.MODE = SS_EN;
	ads->config.bits.PGA = FSR_2048;
	ads->config.bits.MUX = AINPN_0_1;
	ads->config.bits.SS = STOPPED;
	// This is equivalent to 0x058B if you write it out
}

bool editConfig(ADS* ads, uint16_t prevConfig) {
	uint8_t txData[4] = {ads->config.bytes[1], ads->config.bytes[0],
			ads->config.bytes[1], ads->config.bytes[0]};
	ads->cs_low(ads);
	HAL_SPI_TransmitReceive(ads->hspi, txData, ads->rxConfig, 4, HAL_MAX_DELAY);
	ads->cs_high(ads);
	if (txData[0] == ads->rxConfig[2] && txData[1] == ads->rxConfig[3]) {
		return 1;
	}
	// Transmission failed; revert back to previous known configuration state on SW Side
	ads->config.word = prevConfig;
	return 0;
}

bool resetConfig(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.word = 0x058B;
	// 0x058B is the "RESET" Command
	// 0000010110001011 (Bit 15 to Bit 0)
	// Which matches with our ADS.config.bits in initSW() command
	return editConfig(ads, prevConfig);
}

bool enableSingleshot(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MODE = SS_EN;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool enableContinuousConversion(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MODE = CC_EN;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool enableADCSensor(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.TS_MODE = ADC_MODE;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool enableTempSensor(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.TS_MODE = TEMP_MODE;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool singleshotRead(ADS* ads) {
	// Check if we are in Singleshot Mode first
	if (ads->config.bits.MODE == 1) {
		// When we write SS Bit to 1, the ADS responds by powering on and setting SS back to 0
		// It then performs the Singleshot Conversion and stores it in its buffer
		uint16_t prevConfig = ads->config.word;
		ads->config.bits.SS = START;
		ads->config.bits.NOP = DATA_VALID;
		uint8_t txData[2] = {ads->config.bytes[1], ads->config.bytes[0]};
		uint8_t rxData[2];
		ads->cs_low(ads);
		HAL_SPI_Transmit(ads->hspi, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(ads->hspi, rxData, 2, HAL_MAX_DELAY);
		ads->cs_high(ads);
		ads->config.bits.SS = STOPPED;

		if (rxData[0] == ads->config.bytes[1] && rxData[1] == ads->config.bytes[0]) {
			// If the ADS wrote SS back to 0 successfully, we now read its conversion out of the buffer
			uint8_t txData[4] = {ads->config.bytes[1], ads->config.bytes[0],
					ads->config.bytes[1], ads->config.bytes[0]};
			uint8_t rxData[4];
			ads->cs_low(ads);
			HAL_SPI_TransmitReceive(ads->hspi, txData, rxData, 4, HAL_MAX_DELAY);
			ads->cs_high(ads);

			ads->rxADS[0] = rxData[0];
			ads->rxADS[1] = rxData[1];
			int16_t ads_reading = (ads->rxADS[0] << 8) | ads->rxADS[1];
			if (ads->config.bits.TS_MODE == TEMP_MODE) {
				ads->temp = parseTemp(ads_reading);
			} else {
				ads->voltage = parseVoltage(ads, ads_reading);
			}
			return 1;
		} else {
			// Writing SS Bit to 1 failed; go back to prevConfig
			ads->config.word = prevConfig;
			return 0;
		}
	}
	return 0;
}

bool continuousRead(ADS* ads) {
	// Check if we are in Continuous Conversion Mode first
	if (ads->config.bits.MODE == 0) {
		uint8_t txData[4] = {ads->config.bytes[1], ads->config.bytes[0],
				ads->config.bytes[1], ads->config.bytes[0]};
		uint8_t rxData[4];
		ads->cs_low(ads);
		HAL_SPI_TransmitReceive(ads->hspi, txData, rxData, 4, HAL_MAX_DELAY);
		ads->cs_high(ads);
		 ads->rxADS[0] = rxData[0];
		 ads->rxADS[1] = rxData[1];
		int16_t ads_reading = (ads->rxADS[0] << 8) | ads->rxADS[1];
		if (ads->config.bits.TS_MODE == TEMP_MODE) {
			ads->temp = parseTemp(ads_reading);
		} else {
			ads->voltage = parseVoltage(ads, ads_reading);
		}
		return 1;
	}
	return 0;
}

// Select input channel to read voltage from
bool enableAINPN_0_1(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MUX = AINPN_0_1;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool enableAINPN_0_3(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MUX = AINPN_0_3;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool enableAINPN_1_3(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MUX = AINPN_1_3;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool enableAINPN_2_3(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MUX = AINPN_2_3;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool enableAINPN_0_G(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MUX = AINPN_0_G;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}
bool enableAINPN_1_G(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MUX = AINPN_1_G;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool enableAINPN_2_G(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MUX = AINPN_2_G;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

bool enableAINPN_3_G(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.MUX = AINPN_3_G;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

// Adjust full scale ranges
bool enableFSR_6144(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.PGA = FSR_6144;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->FSR = 6.144;
		return 1;
	}
	return 0;
}

bool enableFSR_4096(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.PGA = FSR_4096;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->FSR = 4.096;
		return 1;
	}
	return 0;
}

bool enableFSR_2048(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.PGA = FSR_2048;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->FSR = 2.048;
		return 1;
	}
	return 0;
}

bool enableFSR_1024(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.PGA = FSR_1024;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->FSR = 1.024;
		return 1;
	}
	return 0;
}

bool enableFSR_0512(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.PGA = FSR_0512;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->FSR = 0.512;
		return 1;
	}
	return 0;
}

bool enableFSR_0256(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.PGA = FSR_0256;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->FSR = 0.256;
		return 1;
	}
	return 0;
}

// Change samples per second
bool enableSPS_8(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.DR = SPS_8;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->SPS = 8;
		return 1;
	}
	return 0;
}

bool enableSPS_16(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.DR = SPS_16;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->SPS = 16;
		return 1;
	}
	return 0;
}


bool enableSPS_32(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.DR = SPS_32;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->SPS = 32;
		return 1;
	}
	return 0;
}

bool enableSPS_64(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.DR = SPS_64;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->SPS = 64;
		return 1;
	}
	return 0;
}

bool enableSPS_128(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.DR = SPS_128;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->SPS = 128;
		return 1;
	}
	return 0;
}

bool enableSPS_250(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.DR = SPS_250;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->SPS = 250;
		return 1;
	}
	return 0;
}

bool enableSPS_475(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.DR = SPS_475;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->SPS = 475;
		return 1;
	}
	return 0;
}

bool enableSPS_860(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.DR = SPS_860;
	ads->config.bits.NOP = DATA_VALID;
	if (editConfig(ads, prevConfig)) {
		ads->SPS = 860;
		return 1;
	}
	return 0;
}

// Parsing helper functions
float parseVoltage(ADS* ads, int16_t ads_reading) {
	float lsb = (ads->FSR)/32768.0;
	return ads_reading*lsb;
}

float parseTemp(int16_t ads_reading) {
	int16_t signed_temp = ads_reading >> 2;
	if (signed_temp & 0x2000) { // If sign bit (bit 13) is set
		signed_temp |= 0xC000;  // Set upper bits to preserve sign in 16-bit
	}
	return (signed_temp * 0.03125);
}
