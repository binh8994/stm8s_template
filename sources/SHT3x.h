#ifndef __SHT3x_H
#define __SHT3x_H

#include "Wire.h"

typedef enum eSHT3X_ValueIfError //Define, what to return in case of error: Zero or previous value
{
	Zero,
	PrevValue
} SHT3X_ValueIfError;

typedef enum eSHT3xMode
{
	Single_HighRep_ClockStretch,
	Single_MediumRep_ClockStretch,
	Single_LowRep_ClockStretch,
	Single_HighRep_NoClockStretch,
	Single_MediumRep_NoClockStretch,
	Single_LowRep_NoClockStretch
} SHT3xMode;

typedef enum eSHT3xSensor
{
	SHT30,
	SHT31,
	SHT35
} SHT3xSensor;

typedef enum eSHT3X_TemperatureScale
{
	Cel,
	Far,
	Kel
} SHT3X_TemperatureScale;

typedef enum eSHT3X_AbsHumidityScale
{
	mmHg,
	Torr, //same as mm Hg
	Pa,
	Bar,
	At,	 //Techical atmosphere
	Atm, //Standart atmosphere
	mH2O,
	psi,
} SHT3X_AbsHumidityScale;

typedef struct tSHT3X_CalibrationPoints
{
	float First;
	float Second;
} SHT3X_CalibrationPoints;

typedef struct tSHT3X_CalibrationFactors
{
	float Factor;
	float Shift;
} SHT3X_CalibrationFactors;

typedef enum eSHT3x_Errs
{
	noError = 0,
	Timeout = 1,
	DataCorrupted = 2,
	WrongAddress = 3,
	//I2C errors
	TooMuchData = 4,
	AddressNACK = 5,
	DataNACK = 6,
	OtherI2CError = 7,
	UnexpectedError = 8
} SHT3x_Errs;

typedef struct tSHT3x
{
	SHT3x_Errs SHT3X_Error;
	float _TemperatureCeil;
	float _RelHumidity;
	bool _OperationEnabled;
	uint8_t _HardResetPin;
	SHT3X_ValueIfError _ValueIfError;
	uint8_t _MeasLSB; //Data read command, Most Significant Byte
	uint8_t _MeasMSB; //Data read command, Most Significant Byte
	uint8_t _Address;
	SHT3xSensor _SensorType;
	uint32_t _UpdateIntervalMillisec;
	uint32_t _LastUpdateMillisec;
	uint32_t _TimeoutMillisec;
	SHT3X_CalibrationFactors _TemperatureCalibration;
	SHT3X_CalibrationFactors _RelHumidityCalibration;
	/* 
				* 	Factors for poly for calculating absolute humidity (in Torr):
				*	P = (RelativeHumidity /100%) * sum(_AbsHumPoly[i]*T^i)
				*	where P is absolute humidity (Torr/mm Hg),
				*	T is Temperature(Kelvins degree) / 1000,
				* 	^ means power.
				*	For more data, check the NIST chemistry webbook:
				*	http://webbook.nist.gov/cgi/cbook.cgi?ID=C7732185&Units=SI&Mask=4&Type=ANTOINE&Plot=on#ANTOINE
			*/
	float _AbsHumPoly[6];
} SHT3x;

void SHT3x_Init(SHT3x *sht3x,
				uint8_t Address, SHT3X_ValueIfError Value, uint8_t HardResetPin,
				SHT3xSensor SensorType, SHT3xMode Mode);
void SHT3x_InitDefault(SHT3x *sht3x);

void SHT3x_Begin(SHT3x *sht3x);
void SHT3x_UpdateData(SHT3x *sht3x);

float SHT3x_GetTemperature(SHT3x *sht3x, SHT3X_TemperatureScale Degree);
float SHT3x_GetRelHumidity(SHT3x *sht3x);
float SHT3x_GetAbsHumidity(SHT3x *sht3x, SHT3X_AbsHumidityScale Scale);
float SHT3x_GetTempTolerance(SHT3x *sht3x, SHT3X_TemperatureScale Degree, SHT3xSensor SensorType);
float SHT3x_GetRelHumTolerance(SHT3x *sht3x, SHT3xSensor SensorType);
float SHT3x_GetAbsHumTolerance(SHT3x *sht3x, SHT3X_AbsHumidityScale Scale, SHT3xSensor SensorType);

uint8_t SHT3x_GetError(SHT3x *sht3x);

void SHT3x_SetMode(SHT3x *sht3x, SHT3xMode Mode);
void SHT3x_SetTemperatureCalibrationFactors(SHT3x *sht3x, SHT3X_CalibrationFactors *TemperatureCalibration);
void SHT3x_SetRelHumidityCalibrationFactors(SHT3x *sht3x, SHT3X_CalibrationFactors *RelHumidityCalibration);
void SHT3x_SetTemperatureCalibrationPoints(SHT3x *sht3x, SHT3X_CalibrationPoints *SensorValues, SHT3X_CalibrationPoints *Reference);
void SHT3x_SetRelHumidityCalibrationPoints(SHT3x *sht3x, SHT3X_CalibrationPoints *SensorValues, SHT3X_CalibrationPoints *Reference);

void SHT3x_SoftReset(SHT3x *sht3x);
void SHT3x_HardReset(SHT3x *sht3x);

void SHT3x_HeaterOn(SHT3x *sht3x);
void SHT3x_HeaterOff(SHT3x *sht3x);

void SHT3x_SetAddress(SHT3x *sht3x, uint8_t NewAddress);
void SHT3x_SetUpdateInterval(SHT3x *sht3x, uint32_t UpdateIntervalMillisec);
void SHT3x_SetTimeout(SHT3x *sht3x, uint32_t TimeoutMillisec);

/* Privite functions */
void SHT3x_SendCommand(SHT3x *sht3x, uint8_t MSB, uint8_t LSB);
bool SHT3x_CRC8(uint8_t MSB, uint8_t LSB, uint8_t CRC);
float SHT3x_ReturnValueIfError(SHT3x *sht3x, float Value);
void SHT3x_ToReturnIfError(SHT3x *sht3x, SHT3X_ValueIfError Value);
void SHT3x_SHT3x_I2CError(SHT3x *sht3x, uint8_t I2Canswer);

#endif //SHT3x_h
