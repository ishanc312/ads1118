/*
 * ads1118.c
 *
 *  Created on: Apr 16, 2025
 *      Author: ishanchitale
 */


#include "ads1118.h"

void initADS(ADS* ads, SPI_HandleTypeDef* spiInstance) {
	ads->hspi = spiInstance;

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

	// 0x058B is the "RESET" Byte
	// 0000010110001011 (bit 15 to bit 0)

	// Based on our initialization:
	// We get 0000010110001011, as desired!
	// and bytes correctly shows 0-7 being 10001011
	// and 8-15 being 00000101 (need to send bytes[1] first, then bytes[0])
	// Matching up appropriately with the datasheet :)

}

bool resetConfig(ADS* ads, uint8_t* rxData) {
	uint8_t txData[4] = {0x05, 0x8B, 0x05, 0x8B};
	HAL_GPIO_WritePin(ADS_EN_PORT, ADS_EN_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(ads->hspi, txData, rxData, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(ADS_EN_PORT, ADS_EN_PIN, GPIO_PIN_SET);
	if (txData[0] == rxData[2] && txData[1] == rxData[3]) return 1;
	return 0;
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
