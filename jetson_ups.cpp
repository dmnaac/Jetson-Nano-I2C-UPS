#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "i2c.h"

/*Config Register (R / W)*/
#define _REG_CONFIG       0x00
/*SHUNT VOLTAGE REGISTER (R)*/
#define _REG_SHUNTVOLTAGE 0x01
/*BUS VOLTAGE REGISTER (R)*/
#define _REG_BUSVOLTAGE   0x02
/*POWER REGISTER (R)*/
#define _REG_POWER        0x03
/*CURRENT REGISTER (R)*/
#define _REG_CURRENT      0x04
/*CALIBRATION REGISTER (R/W)*/
#define _REG_CALIBRATION  0x05

enum BusVoltageRange
{
	/*Constants for bus_voltage_rage*/
	RANGE_16V = 0x00,  // set bus voltage range to 16V
	RANGE_32V = 0x01   // set bus voltage range to 32V
};

enum Gain
{
	/*Constants for gain*/
	DIV_1_40MV = 0x00,      // shunt prog.gain set to  1, 40 mV range
	DIV_2_80MV = 0x01,      // shunt prog.gain set to / 2, 80 mV range
	DIV_4_160MV = 0x02,      // shunt prog.gain set to / 4, 160 mV range
	DIV_8_320MV = 0x03       // shunt prog.gain set to / 8, 320 mV range
};

enum ADCResolution
{
	/*Constants for bus_adc_resolution or shunt_adc_resolution*/
	ADCRES_9BIT_1S = 0x00,      // 9bit, 1 sample, 84us
	ADCRES_10BIT_1S = 0x01,     // 10bit, 1 sample, 148us
	ADCRES_11BIT_1S = 0x02,     // 11 bit, 1 sample, 276us
	ADCRES_12BIT_1S = 0x03,     // 12 bit, 1 sample, 532us
	ADCRES_12BIT_2S = 0x09,     // 12 bit, 2 samples, 1.06ms
	ADCRES_12BIT_4S = 0x0A,     // 12 bit, 4 samples, 2.13ms
	ADCRES_12BIT_8S = 0x0B,     // 12bit, 8 samples, 4.26ms
	ADCRES_12BIT_16S = 0x0C,     // 12bit, 16 samples, 8.51ms
	ADCRES_12BIT_32S = 0x0D,     // 12bit, 32 samples, 17.02ms
	ADCRES_12BIT_64S = 0x0E,     // 12bit, 64 samples, 34.05ms
	ADCRES_12BIT_128S = 0x0F     // 12bit, 128 samples, 68.10ms
};

enum Mode
{
	/*Constants for mode*/
	POWERDOW = 0x00,      //power down
	SVOLT_TRIGGERED = 0x01,      // shunt voltage triggered
	BVOLT_TRIGGERED = 0x02,      // bus voltage triggered
	SANDBVOLT_TRIGGERED = 0x03,      // shunt and bus voltage triggered
	ADCOFF = 0x04,       // ADC off
	SVOLT_CONTINUOUS = 0x05,      //shunt voltage continuous
	BVOLT_CONTINUOUS = 0x06,      //bus voltage continuous
	SANDBVOLT_CONTINUOUS = 0x07       //shunt and bus voltage continuous
};

class INA219
{
public:
	INA219();
	~INA219();

	int fd;
	I2CDevice device;

	unsigned short bus_voltage_range;
	unsigned short gain;
	unsigned short bus_adc_resolution;
	unsigned short shunt_adc_resolution;
	unsigned short mode;

	bool initialize(unsigned short addr);
	void set_calibration_32V_2A();
	double getShuntVoltage_mV();
	double getBusVoltage_mV();
	double getCurrent_mA();
	double getPower_W();

private:
	int _cal_value;
	double _current_lsb;
	double _power_lsb;
	uint16_t config;
	unsigned char temp[2];
};

INA219::INA219()
{
	_current_lsb = .1;
	_cal_value = 4096;
	_power_lsb = .002;

	bus_voltage_range = BusVoltageRange::RANGE_32V;
	gain = Gain::DIV_8_320MV;
	bus_adc_resolution = ADCResolution::ADCRES_12BIT_32S;
	shunt_adc_resolution = ADCResolution::ADCRES_12BIT_32S;
	mode = Mode::SANDBVOLT_CONTINUOUS;
	config = bus_voltage_range << 13 | \
		gain << 11 | \
		bus_adc_resolution << 7 | \
		shunt_adc_resolution << 3 | \
		mode;

	temp[1] = _cal_value & 0xFF;
	temp[0] = (_cal_value & 0xFF00) >> 8;
}

INA219::~INA219()
{
	i2c_close(fd);
}

bool INA219::initialize(unsigned short addr)
{
	/* First open i2c bus */
	if ((fd = i2c_open("/dev/i2c-1")) == -1)
	{
		perror("Open i2c bus 1 failed.\n");
		return false;
	}
	else
	{
		/* Fill i2c device struct */
		INA219::device.bus = fd;
		INA219::device.addr = addr;
		INA219::device.tenbit = 0;
		INA219::device.delay = 10;
		INA219::device.flags = 0;
		INA219::device.page_bytes = 16;
		INA219::device.iaddr_bytes = 1; /* Set this to zero, and using i2c_ioctl_xxxx API will ignore chip internal address */
		printf("Open i2c bus succeeded.\n");
		INA219::set_calibration_32V_2A();
	}
}

void INA219::set_calibration_32V_2A()
{
	/*Configures to INA219 to be able to measure up to 32V and
	2A of current. Counter overflow occurs at 3.2A*/
	ssize_t ret;
	
	ret = i2c_ioctl_write(&device, 0x05, temp, sizeof(temp));
	if (ret == -1 || (size_t)ret != sizeof(temp))
	{
		perror("Set calibration register failed");
	}

	temp[1] = config & 0xFF;
	temp[0] = (config & 0xFF00) >> 8;

	ret = i2c_ioctl_write(&device, _REG_CONFIG, temp, 2);
	if (ret == -1 || (size_t)ret != sizeof(config))
	{
		perror("Set config register failed");
	}
}

double INA219::getShuntVoltage_mV()
{
	unsigned char buffer[2];
	ssize_t size = sizeof(buffer);
	ssize_t ret;

	if ((i2c_ioctl_read(&device, _REG_SHUNTVOLTAGE, buffer, size)) != size)
	{
		perror("Read shunt voltage value failed.\n");
	}
	else
	{
		int16_t data = buffer[0] * 256 + buffer[1];
		if (data > 32767)
		{
			data -= 65535;
		}
		double value = data * 0.01;
		return value;
	}
}

double INA219::getBusVoltage_mV()
{
	unsigned char buffer[2];
	ssize_t size = sizeof(buffer);
	ssize_t ret;

	if ((i2c_ioctl_read(&device, _REG_BUSVOLTAGE, buffer, size)) != size)
	{
		perror("Read bus voltage value failed.\n");
	}
	else
	{
		int16_t data = buffer[0] * 256 + buffer[1];
		data = data >> 3;
		double value = data * 0.004;
		return value;
	}
}

double INA219::getCurrent_mA()
{
	unsigned char buffer[2];
	ssize_t size = sizeof(buffer);

	if ((i2c_ioctl_read(&device, _REG_CURRENT, buffer, size)) != size)
	{
		perror("Read current value failed.\n");
	}
	else
	{
		int16_t data = buffer[0] * 256 + buffer[1];
		if (data > 32767)
		{
			data -= 65535;
		}
		double value = data * _current_lsb;
		return value;
	}
}

double INA219::getPower_W()
{
	unsigned char buffer[2];
	ssize_t size = sizeof(buffer);
	ssize_t ret;

	if ((i2c_ioctl_read(&device, _REG_POWER, buffer, size)) != size)
	{
		perror("Read power value failed.\n");
	}
	else
	{
		int16_t data = buffer[0] * 256 + buffer[1];
		if (data > 32767)
		{
			data -= 65535;
		}
		double value = data * _power_lsb;
		return value;
	}
}

int main()
{
	INA219 ina219;
	ina219.initialize(0x42);

	double bus_voltage;
	double current;
	double power;
	double percent;

	while(true)
	{
		bus_voltage = ina219.getBusVoltage_mV();
		current = ina219.getCurrent_mA();
		power = ina219.getPower_W();
		percent = (bus_voltage - 6) / 2.4 * 100;

		if (percent > 100)
		{
			percent = 100;
		}
		if (percent < 0)
		{
			percent = 0;
		}

		printf("Load Voltage: %f V\n", bus_voltage);
		printf("Current: %f A\n", current / 1000);
		printf("Power: %f W\n", power);
		printf("Percent: %f\n", percent);
		printf("\n");

		sleep(1);
	}
}
