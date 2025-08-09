<div align=center>
<pre>
             _____   _____ __ __ __  ___  
     /\   |  __ \ / ____/_ /_ /_ |/ _ \ 
    /  \  | |  | | (___  | || || | (_) |
   / /\ \ | |  | |\___ \ | || || |> _ < 
  / ____ \| |__| |____) || || || | (_) |
 /_/    \_\_____/|_____/ |_||_||_|\___/ 
                                        
                                        
</pre>
</div>

## Overview
An STM32 SPI Driver written for the Texas Instruments ADS1118, an external 4 Channel ADC (Analog-to-digital-converter). Tested on the STM32 L432KCU3, a microcontroller used for the DAQ units on the Bruin Formula Racing Mk. 10 Vehicle. Datasheet found here: [ADS1118](https://www.ti.com/lit/ds/symlink/ads1118.pdf?ts=1744807872165&ref_url=https%253A%252F%252Fwww.google.com%252F)

## Functionality
The header file is `ads1118.h` found in `/Inc`, and the source file is `ads1118.c` found in `/Src`. Passing in a pointer to your ADS object, SPI instance (e.g. `&hspi1`), alongside the GPIO Port and Pin Number of your CS Pin, call:
```C 
void initADS(ADS* ads, SPI_HandleTypeDef* spiInstance, GPIO_TypeDef* PORT, uint16_t GPIO_PIN)
```

- Call either `enableSingleshot(ADS* ads)` or `enableContinuousConversion(ADS* ads)` to put the sensor into Singleshot or Continuous Conversion mode; then, call `enableADCSensor(ADS* ads)` or `enableTempSensor(ADS* ads)` to use the ADS1118 either as a normal ADC or Temperature Sensor
- Call either `singleshotRead(ADS* ads)` or `continuousRead(ADS* ads)` depending on the conversion mode the ADS1118 is in (will not work unless you are in the corresponding mode); resulting value is stored in ads->voltage.
- Call `enableAIN_PN_X_Y(ADS* ads)`, where X & Y represent channels of the ADC in which we calculate the voltage difference between X & Y; note that Y can be G for ground
- Call `enableFSR_N(ADS* ads)` to set the Full Scale Range (FSR), where N/1000 represents the corresponding FSR; e.g. enableFSR_2048 corresponds to 2048/1000 = 2.048 FSR
- Call `enableSPS_N(ADS* ads)` to set the Samples per Second (SPS)

## Example Usage
```C
initADS(&StrainGaugeADS, spiInstance, ADS_EN_PORT, ADS_EN_PIN);
enableContinuousConversion(&StrainGaugeADS);
enableFSR_6144(&StrainGaugeADS);
enableSPS_250(&StrainGaugeADS);
enableAINPN_0_G(&StrainGaugeADS);

while (1) {
  continuousRead(&StrainGaugeADS);
}

```
