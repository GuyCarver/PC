//----------------------------------------------------------------------
// Copyright (c) 2021, gcarver
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright notice,
//       this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright notice,
//       this list of conditions and the following disclaimer in the documentation
//       and/or other materials provided with the distribution.
//
//     * The name of Guy Carver may not be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// FILE    mpu9250.cpp
// BY      gcarver
// DATE    01/25/2021 09:00 AM
//----------------------------------------------------------------------

//Module to handle communication with the mpu9850 9 axis accelerometer
// This controller communicates using I2C.  This board has an mpu-9250 and gy-6500

#include <windows.h>
#include <iostream>
#include <cstdint>
// #include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include "matrix3.h"
#include <CH341DLL.H>							// For interface to I2C controller

// Possibly move all initialization into the background thread?
// Lock data reading from the main thread?
// Move mwick into here.

namespace
{
	constexpr float M_PI = 3.1415926536f;

	constexpr uint32_t ONETENTH = 100000;		// 1/10th second in microseconds

	constexpr uint8_t MPU9050_ADDRESS_68 = 0x68;	// gy-6500 address.

	constexpr uint8_t PWR_MGMT_1 = 0x6B;		// Power Management 1
	constexpr uint8_t PWR_MGMT_2 = 0x6C;		// Power Management 2
	constexpr uint8_t CONFIG = 0x1A;			// Configuration
	constexpr uint8_t SMPLRT_DIV = 0x19;		// Sample rate divider
	constexpr uint8_t GYRO_CONFIG = 0x1B;		// Gyroscope configuration
	constexpr uint8_t ACCEL_CONFIG = 0x1C;		// Accelerometer Configuration
	constexpr uint8_t ACCEL_CONFIG_2 = 0x1D;	// Accelerometer Configuration 2

	constexpr uint8_t FIFO_EN = 0x23;
	constexpr uint8_t I2C_MST_CTRL = 0x24;
	constexpr uint8_t INT_ENABLE = 0x38;

	constexpr uint8_t ACCEL_XOUT_H = 0x3B;
	constexpr uint8_t GYRO_XOUT_H = 0x43;

	constexpr uint8_t FIFO_COUNTH = 0x72;
	constexpr uint8_t FIFO_R_W = 0x74;

	// INT Pin / Bypass Enable Configuration
	// BYPASS_EN[1]:
	// When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into ‘bypass mode’ when the i2c master interface is disabled.
	// The pins will float high due to the internal pull-up if not enabled and the i2c master interface is disabled.
	constexpr uint8_t INT_PIN_CFG = 0x37;

	// User Control
	// I2C_MST_EN[5]:
	// 1 – Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK.
	// 0 – Disable I2C Master I/F module; pins ES_DA and ES_SCL are logically driven by pins SDA/SDI and SCL/ SCLK.
	constexpr uint8_t USER_CTRL = 0x6A;


	// Gyro Full Scale Select
	constexpr uint8_t GFS_250 = 0x00;	// 250dps
	constexpr uint8_t GFS_500 = 0x01;	// 500dps
	constexpr uint8_t GFS_1000 = 0x02;	// 1000dps
	constexpr uint8_t GFS_2000 = 0x03;	// 2000dps

	// Accel Full Scale Select
	constexpr uint8_t AFS_2G = 0x00;	// 2G
	constexpr uint8_t AFS_4G = 0x01;	// 4G
	constexpr uint8_t AFS_8G = 0x02;	// 8G
	constexpr uint8_t AFS_16G = 0x03;	// 16G

	// Accelerometer Scale Modifiers
	constexpr float ACCEL_SCALE_MODIFIER_2G = 2.0f / 32768.0f;
	constexpr float ACCEL_SCALE_MODIFIER_4G = 4.0f / 32768.0f;
	constexpr float ACCEL_SCALE_MODIFIER_8G = 8.0f / 32768.0f;
	constexpr float ACCEL_SCALE_MODIFIER_16G = 16.0f / 32768.0f;

	// Gyroscope Scale Modifiers
	constexpr float GYRO_SCALE_MODIFIER_250DEG = 250.0f / 32768.0f;
	constexpr float GYRO_SCALE_MODIFIER_500DEG = 500.0f / 32768.0f;
	constexpr float GYRO_SCALE_MODIFIER_1000DEG = 1000.0f / 32768.0f;
	constexpr float GYRO_SCALE_MODIFIER_2000DEG = 2000.0f / 32768.0f;

//--------------------------------------------------------
	constexpr uint32_t AK8963_ADDRESS = 0x0C;

	// Magneto Scale Select
	constexpr uint8_t AK_BIT_14 = 0x00;	// 14bit output
	constexpr uint8_t AK_BIT_16 = 0x01;	// 16bit output

	// Continous data output
	constexpr uint32_t AK_MODE_C8HZ = 0x02;		// 8Hz
	constexpr uint32_t AK_MODE_C100HZ = 0x06;	// 100Hz

	constexpr float MAGNOMETER_SCALE_MODIFIER_BIT_14 = 4912.0f / 8190.0f;
	constexpr float MAGNOMETER_SCALE_MODIFIER_BIT_16 = 4912.0f / 32760.0f;

	constexpr uint8_t AK_ST1 = 0x02;
	constexpr uint8_t AK_ST2 = 0x09;
	constexpr uint8_t AK_CNTL1 = 0x0A;
	constexpr uint8_t AK_CNTL2 = 0x0B;
	constexpr uint8_t AK_HX = 0x03;
	constexpr uint8_t AK_ASAX = 0x10;

	constexpr uint8_t MODE_FUSE_ROM_ACCESS = 0x0F;

	constexpr uint32_t _DEFAULTFREQ = 100;
}	//namespace

//--------------------------------------------------------
void delayMicroseconds( uint32_t aMS )
{
	std::this_thread::sleep_for(std::chrono::microseconds(aMS));
}

//--------------------------------------------------------
class mpu9250
{
public:

	//--------------------------------------------------------
	mpu9250( bool bHigh = false )
	{
		_pinstance = this;
		_address = MPU9050_ADDRESS_68 + (bHigh ? 1 : 0);

		_magangle.MakeZRotation(M_PI / -4.0f);

		CH341OpenDevice(0);
		CH341SetStream(0, 1);					// Set stream speed to default, Set to 2 for fast

		delayMicroseconds(50);					// Wait for init to settle.

		_pLoop = new std::thread([] () { _pinstance->MainLoop(); });
	}

	//--------------------------------------------------------
	~mpu9250(  )
	{
		_bRunning = false;						// Turn off loop.
		Suspend(false);							// Make sure we haven't suspending the BG thread.
		_pLoop->join();							// Wait for _pLoop to exit.
		delete _pLoop;
		CH341CloseDevice(0);
		_pinstance = nullptr;
	}

	static mpu9250 *QInstance(  ) { return _pinstance; }

	//--------------------------------------------------------
	void Suspend( bool abTF )
	{
		if (abTF) {
			if (!_bSuspended) {
				_bSuspended = true;
				_suspend.lock();
			}
		}
		else if (_bSuspended) {
			_bSuspended = false;
			_suspend.unlock();
		}
	}

	//--------------------------------------------------------
	const float *GetAccelTempRot(  )
	{
		//Might want to lock before reading.
		return _atp;
	}

	//--------------------------------------------------------
	const float *GetMag(  )
	{
		//Might want to lock before reading.
		return _mag;
	}

	//--------------------------------------------------------
	//Set the mag rotation for x/y to the given angle in radians.
	void SetMagAngle( float aAngle )
	{
		_magangle.MakeZRotation(aAngle);
	}

private:
	Matrix3 _magangle;

	std::thread *_pLoop = nullptr;
	std::mutex _suspend;

	float _buffer[7];

	float _atp[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float _mag[3] = {0.0f, 0.0f, 0.0f};

	float _magcalibration[3] = {1.0f, 1.0f, 1.0f};
	float _magscale[3] = {1.0f, 1.0f, 1.0f};

	float _gbias[3] = {0.0f, 0.0f, 0.0f};
	float _abias[3] = {0.0f, 0.0f, 0.0f};
	float _mbias[3] = {0.0f, 0.0f, 0.0f};

	uint8_t _gfs = GFS_2000;
	uint8_t _afs = AFS_16G;
	uint8_t _mfs = AK_BIT_16;
	uint8_t _mode = AK_MODE_C100HZ;

	float _gres = 1.0f;
	float _ares = 1.0f;
	float _mres = 1.0f;

	uint8_t _address = MPU9050_ADDRESS_68;

	bool _bRunning = true;
	bool _bSuspended = false;

	static mpu9250 *_pinstance;

	//--------------------------------------------------------
	void Configure(  )
	{
		ConfigureMPU6500();
		ConfigureAK8963();
	}

	//--------------------------------------------------------
	void ConfigureMPU6500(  )
	{
		switch (_gfs) {
			case GFS_250:
				_gres = GYRO_SCALE_MODIFIER_250DEG;
				break;
			case GFS_500:
				_gres = GYRO_SCALE_MODIFIER_500DEG;
				break;
			case GFS_1000:
				_gres = GYRO_SCALE_MODIFIER_1000DEG;
				break;
			default:
				_gres = GYRO_SCALE_MODIFIER_2000DEG;
				break;
		}

		switch (_afs) {
			case AFS_2G:
				_ares = ACCEL_SCALE_MODIFIER_2G;
				break;
			case AFS_4G:
				_ares = ACCEL_SCALE_MODIFIER_4G;
				break;
			case AFS_8G:
				_ares = ACCEL_SCALE_MODIFIER_8G;
				break;
			default:
				_ares = ACCEL_SCALE_MODIFIER_16G;
				break;
		}

		//Sleep off
		_write8(_address, 0, PWR_MGMT_1, ONETENTH);
		//Auto select clock source.
		_write8(_address, 1, PWR_MGMT_1, ONETENTH);
		//DLPF_CFG
		_write8(_address, 0, CONFIG);		//Or 3.
		//Sample rate divider
		_write8(_address, 0, SMPLRT_DIV);	//Or 4.
		//Gyro full scale select
		_write8(_address, _gfs << 3, GYRO_CONFIG);
		//Accel full scale select
		_write8(_address, _afs << 3, ACCEL_CONFIG);
		//A_DLPFCFG
		_write8(_address, 0, ACCEL_CONFIG_2);
		//BYPASS_EN
		_write8(_address, 2, INT_PIN_CFG, ONETENTH);
		//Disable master
		_write8(_address, 0, USER_CTRL, ONETENTH);
	}

	//--------------------------------------------------------
	void ConfigureAK8963(  )
	{
		_mres = (_mfs == AK_BIT_14)
			? MAGNOMETER_SCALE_MODIFIER_BIT_14
			: MAGNOMETER_SCALE_MODIFIER_BIT_16;

		//Set power down mode
		_write8(AK8963_ADDRESS, 0, AK_CNTL1, ONETENTH);

		//Set read FuseROM mode
		_write8(AK8963_ADDRESS, MODE_FUSE_ROM_ACCESS, AK_CNTL1, ONETENTH);

		//Read coef data

		auto makemagadj = []( int8_t aVal ) {
			return ((aVal - 128) / 256.0f) + 1.0f;
		};

		_mbias[0] = makemagadj(_read8(AK8963_ADDRESS, AK_ASAX));
		_mbias[1] = makemagadj(_read8(AK8963_ADDRESS, AK_ASAX + 1));
		_mbias[2] = makemagadj(_read8(AK8963_ADDRESS, AK_ASAX + 2));

		//Set power down mode
		_write8(AK8963_ADDRESS, 0, AK_CNTL1, ONETENTH);

		//Set scale and continuous mode
		_write8(AK8963_ADDRESS, (_mfs << 4 | _mode), AK_CNTL1, ONETENTH);
	}

	//--------------------------------------------------------
	void Reset(  )
	{
		_write8(_address, 0x80, PWR_MGMT_1, ONETENTH);
		//Gyro full scale select
		_write8(_address, _gfs << 3, GYRO_CONFIG);
		//Accel full scale select
		_write8(_address, _afs << 3, ACCEL_CONFIG);
	}

	//--------------------------------------------------------
	// Read 8 bit value and return.
	uint8_t _read8( uint8_t aAddress, uint8_t aLoc )
	{
		uint8_t v = 0;
		CH341ReadI2C(0, aAddress, aLoc, &v);
		return v;
	}

	//--------------------------------------------------------
	int16_t _read16( uint8_t aAddress, uint8_t aLoc )
	{
		uint8_t vh = 0;
		uint8_t vl = 0;
		CH341ReadI2C(0, aAddress, aLoc, &vh);
		CH341ReadI2C(0, aAddress, aLoc + 1, &vl);
		return (vh << 8) | (vl & 0xFF);
	}

	//--------------------------------------------------------
	float *_readbuffer( uint8_t aAddress, uint8_t aLoc, uint8_t aCount )
	{
		uint8_t data[2];
		uint8_t rdata[32];
		data[0] = aAddress << 1;
		data[1] = aLoc;

		CH341StreamI2C(0, 2, data, aCount * 2, rdata);

		auto data16 = rdata;

		for ( uint32_t i = 0; i < aCount; ++i, data16 += 2) {
			int16_t v = ((data16[0] << 8) | data16[1]);
			_buffer[i] = static_cast<float>(v);
		}

		return _buffer;
	}

	//--------------------------------------------------------
	float *_readbufferLH( uint8_t aAddress, uint8_t aLoc, uint8_t aCount )
	{
		uint8_t data[2];
		uint8_t rdata[32];
		data[0] = aAddress << 1;
		data[1] = aLoc;

		CH341StreamI2C(0, 2, data, aCount * 2, rdata);

		auto data16 = reinterpret_cast<int16_t*>(rdata);

		for (uint32_t i = 0; i < aCount; ++i) {
			auto v = *data16++;
			_buffer[i] = static_cast<float>(v);
		}

		return _buffer;
	}

	//--------------------------------------------------------
	// Write 8 bit integer aVal to given address aLoc.
	void _write8( uint8_t aAddress, uint8_t aValue, uint8_t aLoc, uint32_t aDelay = 50 )
	{
		CH341WriteI2C(0, aAddress, aLoc, aValue);
		if (aDelay) {
			delayMicroseconds(aDelay);
		}
	}

	//--------------------------------------------------------
	void Calibrate(  )
	{
		CalibrateAK8963();
		CalibrateMPU6500();
	}

	//--------------------------------------------------------
	void CalibrateMPU6500(  )
	{

//#define FIFO_CALIBRATE 1  //NOTE: This does not seem to be returning good values.
#ifdef FIFO_CALIBRATE
		Reset();

		//Get stable time source; Auto select clock source to be PLL gyroscope reference if ready, else use the internal oscillator, bits 2:0 = 001
		_write8(_address, 1, PWR_MGMT_1);
		_write8(_address, 0, PWR_MGMT_2, ONETENTH * 2);

		//Configure device for bias calculation
		_write8(_address, 0, INT_ENABLE);			// Disable Interrupts
		_write8(_address, 0, FIFO_EN);				// Disable FIFO
		_write8(_address, 0, PWR_MGMT_1);			// Turn on internal clock source
		_write8(_address, 0, I2C_MST_CTRL);			// Disable I2C master
		_write8(_address, 0, USER_CTRL);			// Disable FIFO and I2C master modes
		_write8(_address, 0x0C, USER_CTRL);			// Reset FIFO and DMP

		_write8(_address, 1, CONFIG);				// Set low-pass filter to 188hz
		_write8(_address, 0, SMPLRT_DIV);			// Set sample rate to 1khz
		_write8(_address, 0, GYRO_CONFIG);			// 250 dps.
		_write8(_address, 0, ACCEL_CONFIG);			// 2g

		_write8(_address, 0x40, USER_CTRL);			// Enable FIFO.
		_write8(_address, 0x78, FIFO_EN, 40000);	// Enable gyro/accel sensors and wait 4/100ths of a second
		_write8(_address, 0x00, FIFO_EN);			// Disable gyro/accel sensors for FIFO

		int16_t count = _read16(_address, FIFO_COUNTH) / 12;
		std::cout << "count:" << count << std::endl;
		if (count > 0) {
			for ( uint16_t i = 0; i < count; ++i) {
				auto pdata = _readbuffer(_address, FIFO_R_W, 6);
				for ( uint32_t j = 0; j < 6; ++j) {
					std::cout << pdata[j] << ", ";
				}
				std::cout << std::endl;
				_abias[0] += pdata[0];
				_abias[1] += pdata[1];
				_abias[2] += pdata[2];
				_gbias[0] += pdata[3];
				_gbias[1] += pdata[4];
				_gbias[2] += pdata[5];
			}

			float f = 1.0f / static_cast<float>(count);
			_abias[0] = _abias[0] * f * ACCEL_SCALE_MODIFIER_2G;
			_abias[1] = _abias[1] * f * ACCEL_SCALE_MODIFIER_2G;
			_abias[2] = _abias[2] * f;
			//Remove gravity from the z-axis.
			// This assumes the chip is oriented to z axis as up/down, but does
			//  support upside down.
			_abias[2] += 1.0f / (
				(_abias[2] > 0.0f)
					? -ACCEL_SCALE_MODIFIER_2G
					: ACCEL_SCALE_MODIFIER_2G
				);

			_abias[2] *= ACCEL_SCALE_MODIFIER_2G;
			_abias[2] += 0.53f;					// Additional value to get z to 0-1 range.

			_gbias[0] = _gbias[0] * f * GYRO_SCALE_MODIFIER_250DEG;
			_gbias[1] = _gbias[1] * f * GYRO_SCALE_MODIFIER_250DEG;
			_gbias[2] = _gbias[2] * f * GYRO_SCALE_MODIFIER_250DEG;

		}

		Reset();
#else //FIFO_CALIBRATE
		constexpr uint32_t CALIBRATE_COUNT = 200;

		//Reset the bias values.
		for ( auto &v : _abias ) {
			v = 0.0f;
		}
		for ( auto &v : _gbias ) {
			v = 0.0f;
		}

		for ( uint32_t i = 0; i < CALIBRATE_COUNT; ++i) {
			UpdateAccelTempRot();
			auto pdata = GetAccelTempRot();
			for ( uint32_t i = 0; i < 7; ++i) {
				std::cout << pdata[i] << ", ";
			}
			std::cout << std::endl;

			_abias[0] += pdata[0];
			_abias[1] += pdata[1];
			_abias[2] += pdata[2];
			_gbias[0] += pdata[4];				// Index 4 as 3 is the temperature.
			_gbias[1] += pdata[5];
			_gbias[2] += pdata[6];
			delayMicroseconds(20);
		}

		//Get the average for bias values.
		for ( auto &v : _abias ) {
			v /= static_cast<float>(CALIBRATE_COUNT);
		}
		for ( auto &v : _gbias ) {
			v /= static_cast<float>(CALIBRATE_COUNT);
		}
#endif //FIFO_CALIBRATE
	}

	//--------------------------------------------------------
	void CalibrateAK8963(  )
	{
		//Note: We don't currently bother doing this.
	}

	//--------------------------------------------------------
	void UpdateAccelTempRot(  )
	{
		auto pdata = _readbuffer(_address, ACCEL_XOUT_H, 7);
		uint32_t i = 0;
		for ( ; i < 3; ++i) {
			_atp[i] = (pdata[i] * _ares) - _abias[i];
		}

		//Convert temperature value to celcius.
		_atp[i] = (pdata[i] / 333.87f) + 21.0f;		// Just copy the temperature over.
		++i;

		for ( uint32_t j = 0; j < 3; ++i, ++j) {
			_atp[i] = (pdata[i] * _gres) - _gbias[j];
		}
	}

	//--------------------------------------------------------
	void UpdateMag(  )
	{
		float *pdata = nullptr;
		pdata = _readbufferLH(AK8963_ADDRESS, AK_HX, 3);
		auto ov = _read8(AK8963_ADDRESS, AK_ST2);	// Check for overflow.

		if (ov == 0x10) {
			//Convert to mag data to +/-.
			for ( uint32_t i = 0; i < 3; ++i) {
				float v = (pdata[i] * _mres * _magcalibration[i]) - _mbias[i];
				_mag[i] = v * _magscale[i];
			}

// 			_magangle.Rotate(_mag);
		}
	}

	//--------------------------------------------------------
	void MainLoop(  )
	{
		Configure();
//		Calibrate();

		while(_bRunning) {
			_suspend.lock();
			UpdateAccelTempRot();
			UpdateMag();
			_suspend.unlock();
			delayMicroseconds(33333);			// We are going to run this at 30hz.
		}
	}
};

mpu9250 *mpu9250::_pinstance = nullptr;

// int32_t main(  )
// {
// 	auto g = mpu9250();
// 	while (true) {
// 		auto m = g.GetMag();
// 		for ( uint32_t i = 0; i < 3; ++i) {
// 			std::cout << m[i] << ',';
// 		}
// 		std::cout << "                \r";
// 	}
// 	return 0;
// }

extern "C"
{

__declspec(dllexport) void Startup(  )
{
	if (mpu9250::QInstance() == nullptr) {
		auto p = new mpu9250();
	}
}

__declspec(dllexport) void Suspend( bool abTF )
{
	auto p = mpu9250::QInstance();
	if (p) {
		p->Suspend(abTF);
	}
}

__declspec(dllexport) void Shutdown(  )
{
	auto p = mpu9250::QInstance();
	if (p) {
		delete p;
	}
}

//--------------------------------------------------------
__declspec(dllexport) const float *GetAccelTempRot(  )
{
	const float *d = nullptr;
	auto p = mpu9250::QInstance();
	if (p) {
		d = p->GetAccelTempRot();
	}
	return d;
}

//--------------------------------------------------------
__declspec(dllexport) const float *GetMag(  )
{
	const float *d = nullptr;
	auto p = mpu9250::QInstance();
	if (p) {
		d = p->GetMag();
	}
	return d;
}

//--------------------------------------------------------
__declspec(dllexport) void SetMagAngle( float aAngle )
{
	auto p = mpu9250::QInstance();
	if (p) {
		p->SetMagAngle(aAngle);
	}
}

} //extern "C"

int main()
{
	Startup();
//	delayMicroseconds(10000000);

	while (true) {
		auto atr = GetAccelTempRot();
		auto mag = GetMag();
		std::cout << "atr: ";
		for ( uint32_t i = 0; i < 7; ++i) {
			std::cout << atr[i] << ", ";
		}
		std::cout << "mag: ";
		for ( uint32_t i = 0; i < 3; ++i) {
			std::cout << mag[i] << ", ";
		}
		std::cout << std::endl;
		delayMicroseconds(ONETENTH);
	}
}

