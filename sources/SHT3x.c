#include "SHT3x.h"
#include <math.h>

#define millis GetTick

extern uint32_t GetTick(void);

void SHT3x_Init(SHT3x *sht3x,
				uint8_t Address, SHT3X_ValueIfError Value, uint8_t HardResetPin,
				SHT3xSensor SensorType, SHT3xMode Mode)
{
	sht3x->_OperationEnabled = FALSE;
	sht3x->_UpdateIntervalMillisec = 500;
	sht3x->_LastUpdateMillisec = 0;
	sht3x->_TimeoutMillisec = 0;
	sht3x->_TemperatureCalibration.Factor = 1.0;
	sht3x->_TemperatureCalibration.Shift = 0.0;
	sht3x->_RelHumidityCalibration.Factor = 1.0;
	sht3x->_RelHumidityCalibration.Shift = 0.0;

	sht3x->_AbsHumPoly[0] = -157.004;
	sht3x->_AbsHumPoly[1] = 3158.0474;
	sht3x->_AbsHumPoly[2] = -25482.532;
	sht3x->_AbsHumPoly[3] = 103180.197;
	sht3x->_AbsHumPoly[4] = -209805.497;
	sht3x->_AbsHumPoly[5] = 171539.883;

	sht3x->SHT3X_Error = noError;
	SHT3x_SetAddress(sht3x, Address);
	sht3x->_ValueIfError = Value;
	SHT3x_SetMode(sht3x, Mode);
	sht3x->_SensorType = SensorType;
	sht3x->_HardResetPin = HardResetPin;
}

void SHT3x_InitDefault(SHT3x *sht3x)
{
	SHT3x_Init(sht3x, 0x44, Zero, 0xFF, SHT30, Single_HighRep_ClockStretch);
}

void SHT3x_Begin(SHT3x *sht3x)
{
	Wire_begin();
	sht3x->_OperationEnabled = TRUE;
}

void SHT3x_UpdateData(SHT3x *sht3x)
{
	sht3x->SHT3X_Error = noError;
	if ((sht3x->_LastUpdateMillisec == 0) ||
		((millis() - sht3x->_LastUpdateMillisec) >= sht3x->_UpdateIntervalMillisec))
	{
		SHT3x_SendCommand(sht3x, sht3x->_MeasMSB, sht3x->_MeasLSB);
		if (sht3x->SHT3X_Error == noError)
		{
			Wire_requestFrom2(sht3x->_Address, 6);
			uint32_t WaitingBeginTime = millis();
			while ((Wire_available() < 6) && ((millis() - WaitingBeginTime) < sht3x->_TimeoutMillisec))
			{
				//Do nothing, just wait
			}
			if ((millis() - WaitingBeginTime) < sht3x->_TimeoutMillisec)
			{
				sht3x->_LastUpdateMillisec = WaitingBeginTime;
				uint8_t data[6];
				for (uint8_t i = 0; i < 6; i++)
				{
					data[i] = Wire_read();
				}

				if ((SHT3x_CRC8(data[0], data[1], data[2])) && (SHT3x_CRC8(data[3], data[4], data[5])))
				{
					uint16_t TemperatureRaw = (data[0] << 8) + (data[1] << 0);
					uint16_t RelHumidityRaw = (data[3] << 8) + (data[4] << 0);
					sht3x->_TemperatureCeil = ((float)TemperatureRaw) * 0.00267033 - 45.;
					sht3x->_TemperatureCeil = sht3x->_TemperatureCeil * sht3x->_TemperatureCalibration.Factor +
											  sht3x->_TemperatureCalibration.Shift;
					sht3x->_RelHumidity = ((float)RelHumidityRaw) * 0.0015259;
					sht3x->_RelHumidity = sht3x->_RelHumidity * sht3x->_RelHumidityCalibration.Factor +
										  sht3x->_RelHumidityCalibration.Shift;

					sht3x->SHT3X_Error = noError;
				}
				else
				{
					sht3x->SHT3X_Error = DataCorrupted;
				}
			}
			else //Timeout
			{
				sht3x->SHT3X_Error = Timeout;
			}
		}
		else //Error after message send
		{
			// Nothing to do, measurement commands will return NULL because of Error != noError
		}
	}
	else //LastUpdate was too recently
	{
		//Nothing to do, wait for next call
	}
}

float SHT3x_GetTemperature(SHT3x *sht3x, SHT3X_TemperatureScale Degree)
{
	//default scale is Celsius
	//At first, calculate in Celsius, than, if need, recalculate to Farenheit or Kelvin and than adjust according to calibration;
	float Temperature = sht3x->_TemperatureCeil;
	if (Degree == Kel)
	{
		Temperature += 273.15;
	}
	else if (Degree == Far)
	{
		Temperature = Temperature * 1.8 + 32.;
	}

	return SHT3x_ReturnValueIfError(sht3x, Temperature);
}

float SHT3x_GetRelHumidity(SHT3x *sht3x)
{
	return SHT3x_ReturnValueIfError(sht3x, sht3x->_RelHumidity);
}

float SHT3x_GetAbsHumidity(SHT3x *sht3x, SHT3X_AbsHumidityScale Scale)
{
	float millikelvins = SHT3x_GetTemperature(sht3x, Kel) / 1000.;
	float Pressure = 0.;
	for (uint8_t i = 0; i < 6; i++)
	{
		float term = 1.;
		for (uint8_t j = 0; j < i; j++)
		{
			term *= millikelvins;
		}
		Pressure += term * sht3x->_AbsHumPoly[i];
	}
	Pressure *= SHT3x_GetRelHumidity(sht3x);
	switch (Scale)
	{
	case Pa:
	{
		Pressure *= 133.322;
		break;
	}
	case Bar:
	{
		Pressure *= 0.0013332;
		break;
	}
	case At:
	{
		Pressure *= 0.0013595;
		break;
	}
	case Atm:
	{
		Pressure *= 0.0013158;
		break;
	}
	case mH2O:
	{
		Pressure *= 0.013595;
		break;
	}
	case psi:
	{
		Pressure *= 0.019337;
		break;
	}
	default: //mmHg, Torr
	{
		break;
	}
	}
	return SHT3x_ReturnValueIfError(sht3x, Pressure);
}

float SHT3x_GetTempTolerance(SHT3x *sht3x, SHT3X_TemperatureScale Degree, SHT3xSensor SensorType)
{
	//Temperature tolerance is similar for both SHT30 and SHT31
	//At first, calculate at Celsius (similar to Kelvins), than, if need, recalculate to Farenheit
	float Temperature = SHT3x_GetTemperature(sht3x, Cel);
	float Tolerance = 0.2;
	switch (SensorType)
	{
	case SHT30:
	{
		if ((0. <= Temperature) && (Temperature <= 65.))
		{
			Tolerance = 0.2;
		}
		else if (Temperature > 65.)
		{
			//Linear from 0.2 at 65 C to 0.6 at 125 C.
			Tolerance = 0.0067 * Temperature - 0.2333;
		}
		else //if (Temperature < 0.)
		{
			//Linear from 0.6 at -40 C to 0.2 at 0 C.
			Tolerance = -0.01 * Temperature + 0.2;
		}
		break;
	}
	case SHT31:
	{
		if ((0. <= Temperature) && (Temperature <= 90.))
		{
			Tolerance = 0.2;
		}
		else if (Temperature > 65.)
		{
			//Linear from 0.2 at 90 C to 0.5 at 125 C.
			Tolerance = 0.0086 * Temperature - 0.5714;
		}
		else //if (Temperature < 0.)
		{
			//Linear from 0.3 at -40 C to 0.2 at 0 C.
			Tolerance = -0.0025 * Temperature + 0.2;
		}
		break;
	}
	case SHT35:
	{
		if (Temperature <= 0.)
		{
			Tolerance = 0.2;
		}
		else if ((0. < Temperature) && (Temperature <= 20.))
		{
			//Linear from 0.2 at 0 C to 0.1 at 20 C.
			Tolerance = -0.005 * Temperature + 0.2;
		}
		else if ((20. < Temperature) && (Temperature <= 60.))
		{
			Tolerance = 0.1;
		}
		else if ((60. < Temperature) && (Temperature <= 90.))
		{
			//Linear from 0.1 at 60 C to 0.2 at 90 C.
			Tolerance = -0.0033 * Temperature - 0.1;
		}
		else //if (90. < Temperature)
		{
			//Linear from 0.2 at 90 C to 0.4 at 125 C.
			Tolerance = 0.0057 * Temperature - 0.3143;
		}
		break;
	}
	}
	if (Degree == Far)
	{
		Tolerance *= 1.8;
	}

	return SHT3x_ReturnValueIfError(sht3x, Tolerance);
}

float SHT3x_GetRelHumTolerance(SHT3x *sht3x, SHT3xSensor SensorType)
{
	float RelHumidity = SHT3x_GetRelHumidity(sht3x);
	float Tolerance = 2.;
	switch (SensorType)
	{
	case SHT30:
	{
		if ((10. <= RelHumidity) && (RelHumidity <= 90.))
		{
			Tolerance = 2.;
		}
		else if (RelHumidity < 10.)
		{
			//Linear from 4 at 0% to 2 at 10%
			Tolerance = -0.2 * RelHumidity + 4.;
		}
		else
		{
			//Linear from 2 at 90% to 4 at 100%
			Tolerance = 0.2 * RelHumidity - 16.;
		}
		break;
	}
	case SHT31:
	{
		Tolerance = 2.;
		break;
	}
	case SHT35:
	{
		if (RelHumidity <= 80.)
		{
			Tolerance = 1.5;
		}
		else //if (80 < RelHumidity)
		{
			//Linear from 0.5 at 80% to 2 at 100%
			Tolerance = 0.025 * RelHumidity - 0.5;
		}
		break;
	}
	}
	return SHT3x_ReturnValueIfError(sht3x, Tolerance);
}

float SHT3x_GetAbsHumTolerance(SHT3x *sht3x, SHT3X_AbsHumidityScale Scale, SHT3xSensor SensorType)
{
	/*	Dependence of absolute humidity is similar (from 0 to 80C) to P = H*a*exp(b*T),
	*	where P is absolute humidity, H is relative humidity, T is temperature (Celsius),
	*	a ~= 0.0396, b~=0.0575. 
	*	So its relative tolerance dP/P =  square root  [ (dH/H)^2 + (b*dT)^2 ].
	*/

	float RelHumidityRelTolerance = SHT3x_GetRelHumTolerance(sht3x, SensorType) / SHT3x_GetRelHumidity(sht3x);
	float TemperatureRelTolerance = 0.0575 * SHT3x_GetTempTolerance(sht3x, Cel, SensorType);
	RelHumidityRelTolerance *= RelHumidityRelTolerance;
	TemperatureRelTolerance *= TemperatureRelTolerance;
	return SHT3x_ReturnValueIfError(sht3x,
									SHT3x_GetAbsHumidity(sht3x, Scale) * sqrtf(RelHumidityRelTolerance + TemperatureRelTolerance));
}

void SHT3x_SetMode(SHT3x *sht3x, SHT3xMode Mode)
{
	switch (Mode)
	{
	case Single_HighRep_ClockStretch:
	{
		sht3x->_MeasMSB = 0x2C;
		sht3x->_MeasLSB = 0x06;
		break;
	}
	case Single_MediumRep_ClockStretch:
	{
		sht3x->_MeasMSB = 0x2C;
		sht3x->_MeasLSB = 0x0D;
		break;
	}
	case Single_LowRep_ClockStretch:
	{
		sht3x->_MeasMSB = 0x2C;
		sht3x->_MeasLSB = 0x10;
		break;
	}
	case Single_HighRep_NoClockStretch:
	{
		sht3x->_MeasMSB = 0x24;
		sht3x->_MeasLSB = 0x00;
		break;
	}
	case Single_MediumRep_NoClockStretch:
	{
		sht3x->_MeasMSB = 0x24;
		sht3x->_MeasLSB = 0x0B;
		break;
	}
	case Single_LowRep_NoClockStretch:
	{
		sht3x->_MeasMSB = 0x24;
		sht3x->_MeasLSB = 0x16;
		break;
	}
	default:
	{
		sht3x->_MeasMSB = 0x2C;
		sht3x->_MeasLSB = 0x06;
		break;
	}
	}
}

void SHT3x_SetTemperatureCalibrationFactors(SHT3x *sht3x, SHT3X_CalibrationFactors *TemperatureCalibration)
{
	sht3x->_TemperatureCalibration.Factor = TemperatureCalibration->Factor;
	sht3x->_TemperatureCalibration.Shift = TemperatureCalibration->Shift;
}

void SHT3x_SetRelHumidityCalibrationFactors(SHT3x *sht3x, SHT3X_CalibrationFactors *RelHumidityCalibration)
{
	sht3x->_RelHumidityCalibration.Factor = RelHumidityCalibration->Factor;
	sht3x->_RelHumidityCalibration.Shift = RelHumidityCalibration->Shift;
}

void SHT3x_SetTemperatureCalibrationPoints(SHT3x *sht3x, SHT3X_CalibrationPoints *SensorValues, SHT3X_CalibrationPoints *Reference)
{
	sht3x->_TemperatureCalibration.Factor = (Reference->Second - Reference->First) / (SensorValues->Second - SensorValues->First);
	sht3x->_TemperatureCalibration.Shift = Reference->First - sht3x->_TemperatureCalibration.Factor * SensorValues->First;
}

void SHT3x_SetRelHumidityCalibrationPoints(SHT3x *sht3x, SHT3X_CalibrationPoints *SensorValues, SHT3X_CalibrationPoints *Reference)
{
	sht3x->_RelHumidityCalibration.Factor = (Reference->Second - Reference->First) / (SensorValues->Second - SensorValues->First);
	sht3x->_RelHumidityCalibration.Shift = Reference->First - sht3x->_RelHumidityCalibration.Factor * SensorValues->First;
}

void SHT3x_SoftReset(SHT3x *sht3x)
{
	SHT3x_SendCommand(sht3x, 0x30, 0xA2);
}

void SHT3x_HardReset(SHT3x *sht3x)
{
	if (sht3x->_HardResetPin < 100)
	{
		//TODO
	}
}

void SHT3x_HeaterOn(SHT3x *sht3x)
{
	SHT3x_SendCommand(sht3x, 0x30, 0x6D);
}

void SHT3x_HeaterOff(SHT3x *sht3x)
{
	SHT3x_SendCommand(sht3x, 0x30, 0x66);
}

void SHT3x_SetAddress(SHT3x *sht3x, uint8_t Address)
{
	if ((Address == 0x44) || (Address == 0x45))
	{
		sht3x->_Address = Address;
	}
	else
	{
		sht3x->SHT3X_Error = WrongAddress;
	}
}

void SHT3x_SetUpdateInterval(SHT3x *sht3x, uint32_t UpdateIntervalMillisec)
{
	if (UpdateIntervalMillisec > 0)
	{
		sht3x->_UpdateIntervalMillisec = UpdateIntervalMillisec;
	}
}

void SHT3x_SetTimeout(SHT3x *sht3x, uint32_t TimeoutMillisec)
{
	if (TimeoutMillisec > 0)
	{
		sht3x->_TimeoutMillisec = TimeoutMillisec;
	}
}

void SHT3x_I2CError(SHT3x *sht3x, uint8_t I2Canswer)
{
	switch (I2Canswer)
	{
	case 0:
	{
		sht3x->SHT3X_Error = noError;
		break;
	}
	case 1:
	{
		sht3x->SHT3X_Error = TooMuchData;
		break;
	}
	case 2:
	{
		sht3x->SHT3X_Error = AddressNACK;
		break;
	}
	case 3:
	{
		sht3x->SHT3X_Error = DataNACK;
		break;
	}
	case 4:
	{
		sht3x->SHT3X_Error = OtherI2CError;
		break;
	}
	default:
	{
		sht3x->SHT3X_Error = UnexpectedError;
		break;
	}
	}
}

void SHT3x_SendCommand(SHT3x *sht3x, uint8_t MSB, uint8_t LSB)
{
	if (sht3x->_OperationEnabled)
	{
		//Everything is OK, nothing to do
	}
	else
	{
		Wire_begin();
		sht3x->_OperationEnabled = TRUE;
	}
	Wire_beginTransmission(sht3x->_Address);
	// Send Soft Reset command
	Wire_write(MSB);
	Wire_write(LSB);
	// Stop I2C transmission
	uint8_t success = Wire_endTransmission();
}

bool SHT3x_CRC8(uint8_t MSB, uint8_t LSB, uint8_t CRC)
{
	/*
	*	Name  : CRC-8
	*	Poly  : 0x31	x^8 + x^5 + x^4 + 1
	*	Init  : 0xFF
	*	Revert: FALSE
	*	XorOut: 0x00
	*	Check : for 0xBE,0xEF CRC is 0x92
	*/
	uint8_t crc = 0xFF;
	uint8_t i;
	crc ^= MSB;

	for (i = 0; i < 8; i++)
		crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

	crc ^= LSB;
	for (i = 0; i < 8; i++)
		crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

	if (crc == CRC)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

uint8_t SHT3x_GetError(SHT3x *sht3x)
{
	return sht3x->SHT3X_Error;
}

float SHT3x_ReturnValueIfError(SHT3x *sht3x, float Value)
{
	if (sht3x->SHT3X_Error == noError)
	{
		return Value;
	}
	else
	{
		if (sht3x->_ValueIfError == PrevValue)
		{
			return Value;
		}
		else
		{
			return 0.;
		}
	}
}

void SHT3x_ToReturnIfError(SHT3x *sht3x, SHT3X_ValueIfError Value)
{
	sht3x->_ValueIfError = Value;
}
