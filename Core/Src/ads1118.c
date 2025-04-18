/*
 * ads1118.c
 *
 *  Created on: Apr 16, 2025
 *      Author: ishanchitale
 */


#include "ads1118.h"

// Purely for initialization of the ADS struct, does not interface with Hardware
void initADS(ADS* ads, SPI_HandleTypeDef* spiInstance) {
	ads->hspi = spiInstance;
	ads->FSR = 2.048;
	ads->SPS = 128;

	// Default Config According to Datasheet
	ads->configReg.bits.RESV = CONFIG_BIT_RESV; // Low bit (0)
	ads->configReg.bits.NOP = DATA_VALID;
	ads->configReg.bits.PULL_UP_EN = ENABLED;
	ads->configReg.bits.TS_MODE = ADC_MODE;
	ads->configReg.bits.DR = SPS_128;
	ads->configReg.bits.MODE = SS_EN;
	ads->configReg.bits.PGA = FSR_2048;
	ads->configReg.bits.MUX = AINPN_0_1;
	ads->configReg.bits.SS = STOPPED; // High bit (15)

}

// ------------ ADS1118 HARDWARE INTERACTION ------------

// Reading/Writing the CONFIG Register

bool resetConfig(ADS* ads, uint8_t* rxData) {
	uint8_t txData[4] = {0x05, 0x8B, 0x05, 0x8B};
	HAL_GPIO_WritePin(ADS_EN_PORT, ADS_EN_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(ads->hspi, txData, rxData, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(ADS_EN_PORT, ADS_EN_PIN, GPIO_PIN_SET);
	if (txData[0] == rxData[2] && txData[1] == rxData[3]) return 1;
	return 0;

	// 0x058B is the "RESET" Byte
	// 0000010110001011 (bit 15 to bit 0)
	// Based on our initialization, we get 0000010110001011, as desired!
	// uint8_t bytes[2] correctly shows bytes[0], i.e. bits 0-7 being 10001011
	// and bytes[1], i.e. bits 8-15 being 00000101
	// Hence why we sent bytes[1] first and then bytes[0] (we send MSB first!)
}

bool editConfig(ADS* ads, uint8_t* rxData) {
	ads->configReg.bits.NOP = DATA_VALID;
	uint8_t txData[4] = {ads->configReg.bytes[1], ads->configReg.bytes[0],
				ads->configReg.bytes[1], ads->configReg.bytes[0]
	};
	HAL_GPIO_WritePin(ADS_EN_PORT, ADS_EN_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(ads->hspi, txData, rxData, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(ADS_EN_PORT, ADS_EN_PIN, GPIO_PIN_SET);
	if (rxData[2] == ads->configReg.bytes[1] && rxData[3] == ads->configReg.bytes[0]) {
		ads->configReg.bits.NOP = DATA_INVALID;
		return 1;
	}
	ads->configReg.bits.NOP = DATA_INVALID;
	return 0;
}

bool enableSingleshot(ADS* ads, uint8_t* rxData) {
	ads->configReg.bits.MODE = SS_EN;
	return editConfig(ads, rxData);
}

bool enable_AIN0_SE(ADS* ads, uint8_t* rxData) {
	ads->configReg.bits.MUX = AINPN_0_G;
	return editConfig(ads, rxData);
}

// Reading the CONVERSION Register

float singleshotRead(ADS* ads, uint8_t* rxData) {
	ads->configReg.bits.SS = START;
	int status = editConfig(ads, rxData);
	ads->configReg.bits.SS = STOPPED;
	if (!status) {
		/* The ADS should say the txData and rxData are unequal, which makes sense
		 * If you write to CONFIG with SS bit set to 1 (i.e. start SS Conversion)
		 * The ADS should parse this and change SS bit back to 0; change is reflected in rxData
		 * So, status is 0 because txData does NOT match rxData
		 */
		editConfig(ads, rxData);
		// Temporary call; should probably change to something else for now
		uint16_t ads_reading = ((uint16_t)rxData[0] << 8) | rxData[1];
		// rxData[0] is DATA MSB, rxData[1] is DATA LSB
		float voltage = parseVoltage(ads, ads_reading);
		return voltage;
	}
	return 0;
}

float parseVoltage(ADS* ads, uint16_t ads_reading) {
	return ((ads_reading/32768.0)*(ads->FSR));
}

