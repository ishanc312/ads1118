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
}

bool enableSingleshot(ADS* ads, uint8_t* rxData) {
	ads->configReg.bits.NOP = DATA_VALID;
	ads->configReg.bits.MODE = SS_EN;

	uint8_t txData[4] = {ads->configReg.bytes[1], ads->configReg.bytes[0],
			ads->configReg.bytes[1], ads->configReg.bytes[0]
	};
	HAL_GPIO_WritePin(ADS_EN_PORT, ADS_EN_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(ads->hspi, txData, rxData, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(ADS_EN_PORT, ADS_EN_PIN, GPIO_PIN_SET);

	if (rxData[3] == ads->configReg.bytes[1] && rxData[4] == ads->configReg.bytes[0]) return 1;
	return 0;
}

//bool enable_AIN0_SE(ADS* adsInstance) {
//	ads->configReg.bits.NOP = DATA_VALID;
//	ads->configReg.bits.MUX = AINPN_0_G;
//}

// Now, SG1_AX goes to AIN0
// Set MUX to 100 to let AIN0 be AIN_P, and GND be AIN_N
