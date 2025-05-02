/*
 * ads1118.c
 *
 *  Created on: Apr 16, 2025
 *      Author: ishanchitale
 */


#include "ads1118.h"

// Initialize ADS Struct
void initADS_SW(ADS* ads, SPI_HandleTypeDef* spiInstance, GPIO_TypeDef* GPIO_PORT, uint16_t GPIO_PIN) {
	// Important Parameters
	ads->hspi = spiInstance;
	ads->FSR = 2.048; // Full Scale Range
	ads->SPS = 128; // Data Rate, i.e. Samples Per Second
	ads->GPIO_PORT = GPIO_PORT;
	ads->GPIO_PIN = GPIO_PIN;

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
	HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(ads->hspi, txData, ads->rxConfig, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_SET);
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

bool enableTempSensor(ADS* ads) {
	uint16_t prevConfig = ads->config.word;
	ads->config.bits.TS_MODE = TEMP_MODE;
	ads->config.bits.NOP = DATA_VALID;
	return editConfig(ads, prevConfig);
}

// We read data by transmitting the config & receiving at the same time!
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
		HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(ads->hspi, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(ads->hspi, rxData, 2, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_SET);
		ads->config.bits.SS = STOPPED;
		if (rxData[0] == ads->config.bytes[1] && rxData[1] == ads->config.bytes[0]) {
			// If the ADS wrote SS back to 0 successfully, we now read its conversion
			uint8_t txADS[2] = {ads->config.bytes[1], ads->config.bytes[0]};
			HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(ads->hspi, txADS, ads->rxADS, 2, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_SET);

			uint16_t ads_reading = ((uint16_t) ads->rxADS[0] << 8) | ads->rxADS[1];
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
		uint8_t txData[2] = {ads->config.bytes[1], ads->config.bytes[0]};
		HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(ads->hspi, txData, ads->rxADS, 2, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(ads->GPIO_PORT, ads->GPIO_PIN, GPIO_PIN_SET);
		uint16_t ads_reading = ((uint16_t) ads->rxADS[0] << 8) | ads->rxADS[1];
		if (ads->config.bits.TS_MODE == TEMP_MODE) {
			ads->temp = parseTemp(ads_reading);
		} else {
			ads->voltage = parseVoltage(ads, ads_reading);
		}
		return 1;
	}
	return 0;
}

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

float parseVoltage(ADS* ads, uint16_t ads_reading) {
	return ((ads_reading/32768.0)*(ads->FSR));
}

float parseTemp(uint16_t ads_reading) {
	int16_t signed_temp = ads_reading >> 2;
	if (signed_temp & 0x2000) { // If sign bit (bit 13) is set
		signed_temp |= 0xC000;  // Set upper bits to preserve sign in 16-bit
	}
	return (signed_temp * 0.03125);
}

