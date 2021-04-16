//----------------------------------------------------------------------
// Copyright (c) 2020, gcarver
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
// FILE    pca9865.cpp
// DATE    08/14/2020 10:25 PM
//----------------------------------------------------------------------

//Module to handle communication with the PCA9865 16 servo controller
// This controller communicates using I2C by way of a CH341 usb device.

//NOTE: This system uses a singleton instance which is not very flexible.
// It would be best to pass the instance ptr back to the caller to use as a handle
// for operations.

#include <windows.h>
#include <iostream>
#include <cstdint>
#include <chrono>
#include <thread>
#include "CH341DLL.H"							// For interface to I2C controller

namespace
{
	//--------------------------------------------------------
	//Device addresses

	constexpr uint8_t _ADDRESS = 0x40;
	constexpr uint8_t _MODE1 = 0x0;
	constexpr uint8_t _PRESCALE = 0xFE;
	constexpr uint8_t _LED0_ON_L = 0x6;			// We only use LED0 and offset 0-16 from it
// 	constexpr uint8_t _LED0_ON_H = 0x7;
// 	constexpr uint8_t _LED0_OFF_L = 0x8;
// 	constexpr uint8_t _LED0_OFF_H = 0x9;
	constexpr uint8_t _ALLLED_ON_L = 0xFA;
// 	constexpr uint8_t _ALLLED_ON_H = 0xFB;
// 	constexpr uint8_t _ALLLED_OFF_L = 0xFC;
// 	constexpr uint8_t _ALLLED_OFF_H = 0xFD;

	//--------------------------------------------------------
	//Various constants

	constexpr uint32_t I2C_SMBUS_BLOCK_MAX = 1024u;
	constexpr uint32_t _DEFAULTFREQ = 100;
	//The min/max were determined using trial and error with a frequency of 100
	constexpr uint32_t _MINPULSE = 200;
	constexpr uint32_t _MAXPULSE = 930;
	constexpr uint32_t _RANGE = _MAXPULSE - _MINPULSE;
	constexpr uint32_t _END = 4095;
}	//namespace

//--------------------------------------------------------
void delayMicroseconds( uint32_t aMS )
{
	std::this_thread::sleep_for(std::chrono::microseconds(aMS));
}

//--------------------------------------------------------
class pca9865
{
public:

	//--------------------------------------------------------
	pca9865( uint32_t aFreq = _DEFAULTFREQ )
	{
		_i2c = CH341OpenDevice(0);
		CH341SetStream(0, 1);					// Set stream speed to default, Set to 2 for fast

		delayMicroseconds(50);					// Wait for init to settle
		_write8(0, _MODE1);
		bGood = true;

		setfreq(aFreq);
		alloff();								// Make sure we don't move to 0 or things jerk

		_instance = this;						// We only have 1 instance of this object
	}

	//--------------------------------------------------------
	~pca9865(  )
	{
		alloff();
		CH341CloseDevice(0);
		_instance = nullptr;
	}

	//--------------------------------------------------------
	// Set frequency for all servos.  A good value is 60hz (default).
	void setfreq( uint32_t aFreq )
	{
		auto f = static_cast<float>(aFreq) * 0.9999f;	// Correct for overshoot in frequency setting.
		if (f < 1.0f) { f = 1.0f; } else if (f > 3500.0f) { f = 3500.0f; }
		float prescalefloat = (6103.51562f / f) - 1.0f;  // 25000000 / 4096 / freq.
		auto prescale = static_cast<uint8_t>(prescalefloat + 0.5f);

		uint8_t oldmode = _read(_MODE1);
		uint8_t newmode = (oldmode & 0x7F) | 0x10;
		_write8(newmode, _MODE1);
		_write8(prescale, _PRESCALE);
		_write8(oldmode, _MODE1);
		delayMicroseconds(50);
		_write8(oldmode | 0xA1, _MODE1);			// This sets the MODE1 register to turn on auto increment.
	}

	//--------------------------------------------------------
	// Turn off a servo.
	void off( uint8_t aServo )
	{
		_setpwm(aServo, 0, 0);
	}

	//--------------------------------------------------------
	// Turn all servos off.
	void alloff(  )
	{
		static const uint8_t buffer[4] = { 0, 0, 0, 0};
		// Data = on-low, on-high, off-low and off-high.  That's 4 bytes each servo.
		_writebuffer(buffer, 4, _ALLLED_ON_L);
	}

	//--------------------------------------------------------
	// Perc is an integer value from 0-100 * 100 to include 2 digits of fractional precision.
	void set( uint8_t aServo, uint32_t aPerc )
	{
		if (aPerc < 0) {
			off(aServo);
		}
		else {
			uint32_t base = _RANGE * aServo;

			//Range times percentage then divided by 100% and 100 for 2 decimal digits.
			uint32_t val = (_MINPULSE + base + ((_RANGE * aPerc) / 10000u)) & _END;
// 			std::cout << "Vals: " << base << val << std::endl;
			_setpwm(aServo, base & _END, val);
		}
	}

	//--------------------------------------------------------
	// Set angle -90 to +90.  < -90 is off.
	void setangle( uint8_t aServo, float aAngle )
	{
		// (a + 90.0) / 180.0
		float perc = (aAngle + 90.0f) * 55.56f;  //Convert angle +/- 90 to 0.0-10000
		set(aServo, static_cast<int32_t>(perc));
	}

	//--------------------------------------------------------
	void setpwm( uint8_t aServo, uint32_t aOn, uint32_t aOff )
	{
		_setpwm(aServo, aOn, aOff);
	}

	//--------------------------------------------------------
	static pca9865 *QInstance(  ) { return _instance; }

	//--------------------------------------------------------
	bool QGood( ) const { return bGood; }

private:
	HANDLE _i2c = 0;
	bool bGood = true;

	static pca9865 *_instance;

	//--------------------------------------------------------
	// Read 8 bit value and return.
	const uint8_t _read( uint8_t aLoc )
	{
		uint8_t v = 0;
		CH341ReadI2C(0, _ADDRESS, aLoc, &v);
		return v;
	}

	//--------------------------------------------------------
	// Write 8 bit integer aVal to given address aLoc.
	void _write8( uint8_t aValue, uint8_t aLoc )
	{
		CH341WriteI2C(0, _ADDRESS, aLoc, aValue);
	}

	//--------------------------------------------------------
	// Write 8 bit buffer to given address.
	void _writebuffer( const uint8_t *apBuffer, uint32_t aLen, uint8_t aLoc )
	{
		uint8_t data[I2C_SMBUS_BLOCK_MAX + 2];
		data[0] = _ADDRESS << 1; //Shifter 1 for CH341StreamI2C() for some stupid reason
		data[1] = aLoc;

		while (aLen) {
			uint32_t ds = min(I2C_SMBUS_BLOCK_MAX, aLen);
			aLen -= ds;
			memcpy(data + 2, apBuffer, ds);
			apBuffer += ds;
			CH341StreamI2C(0, ds + 2, data, 0, nullptr);
		}
	}

	//--------------------------------------------------------
	// aServo = 0-15.
	// aOn = 16 bit on value.
	// aOff = 16 bit off value.
	void _setpwm( uint8_t aServo, uint32_t aOn, uint32_t aOff )
	{
// 		std::cout << aOn << ", " << aOff << std::endl;
		if ((0 <= aServo) && (aServo <= 15)) {
			uint8_t buffer[4];
			// Data = on-low, on-high, off-low and off-high.  That's 4 bytes each servo.
			uint8_t loc = _LED0_ON_L + (aServo * 4);
			buffer[0] = static_cast<uint8_t>(aOn & 0xFF);
			buffer[1] = static_cast<uint8_t>(aOn >> 8);
			buffer[2] = static_cast<uint8_t>(aOff & 0xFF);
			buffer[3] = static_cast<uint8_t>(aOff >> 8);
			_writebuffer(buffer, 4, loc);
		}
	}
};

pca9865 *pca9865::_instance = nullptr;

//Following are the external interface functions.
extern "C"
{

//--------------------------------------------------------
bool Startup(  )
{
	if (!pca9865::QInstance()) {
		new pca9865();

	}
	return pca9865::QInstance()->QGood();
}

//--------------------------------------------------------
bool IsGood(  )
{
	auto p = pca9865::QInstance();
	return p ? p->QGood() : false;
}

//--------------------------------------------------------
void Shutdown(  )
{
	auto p = pca9865::QInstance();
	if (p) {
		delete p;
	}
}

//--------------------------------------------------------
void SetFreq( uint32_t aFreq )
{
	auto p = pca9865::QInstance();
	if (p) {
		p->setfreq(aFreq);
	}
}

//--------------------------------------------------------
void Off( uint8_t aServo )
{
	auto p = pca9865::QInstance();
	if (p) {
		p->off(aServo);
	}
}

//--------------------------------------------------------
void AllOff(  )
{
	auto p = pca9865::QInstance();
	if (p) {
		p->alloff();
	}
}

//--------------------------------------------------------
// Set servo to percentage (0.0-1.0)
void Set( uint8_t aServo, float aPerc )
{
	auto p = pca9865::QInstance();
	if (p) {
// 		std::cout << "setting: " << aServo << " to " << aPerc << std::endl;
		//Convert percentage 0.0-1.0 into 0-100 plus 2 digits of precision.
		p->set(aServo, static_cast<int32_t>(aPerc * 10000.0f));
	}
}

//--------------------------------------------------------
// Set angle to range -90/+90
void SetAngle( uint8_t aServo, float aAngle )
{
	auto p = pca9865::QInstance();
	if (p) {
		p->setangle(aServo, aAngle);
	}
}

//--------------------------------------------------------
//Set PWM values 0-4095
void SetPWM( uint8_t aServo, uint32_t aOn, uint32_t aOff )
{
	auto p = pca9865::QInstance();
	if (p) {
		p->setpwm(aServo, aOn, aOff);
	}
}

} //extern C

//--------------------------------------------------------
int32_t main(  )
{
	Startup();
	SetAngle(6, -90);
	std::cout << "-90\n";
	delayMicroseconds(4000000);
	SetAngle(6, 0);
	std::cout << "0\n";
	delayMicroseconds(4000000);
	SetAngle(6, 90);
	std::cout << "90\n";
	delayMicroseconds(4000000);
	AllOff();
	std::cout << "Off\n";
	delayMicroseconds(1000);
	Shutdown();
}

