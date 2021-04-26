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
// FILE    adc.cpp
// DATE    04/21/2021 09:13 PM
//----------------------------------------------------------------------

//Module to handle communication with the ads1115 Analog to Digital converter.
// This controller communicates using I2C by way of a CH341 usb device.

#include <windows.h>
#include <iostream>
#include <cstdint>
#include <chrono>
#include <thread>
#include <CH341DLL.H>

namespace
{
	constexpr uint8_t _ADDRESS = 0x48;

	constexpr uint8_t _RA_CONVERSION = 0x00;
	constexpr uint8_t _RA_CONFIG = 0x01;

	constexpr uint16_t _OS_SINGLE = 0x8000;		// Write: Set to start a single-conversion
	
	constexpr uint16_t _OS_NOTBUSY = 0x80;	// Bit set to 1 on _RA_CONFIG read when no conversion in progress

	constexpr uint16_t _CQUE_NONE = 0x03;
	constexpr uint16_t _MODE_SINGLESHOT = 0x0100;


	//--------------------------------------------------------
	//These values need to be << 12 before merged with _mode for write to the ADC.
	enum class MUX : uint16_t
	{
		_MUX_DIFF_0_1,		// 0x0000 Differential P  =  AIN0, N  =  AIN1 (default)
		_MUX_DIFF_0_3,		// 0x1000 Differential P  =  AIN0, N  =  AIN3
		_MUX_DIFF_1_3,		// 0x2000 Differential P  =  AIN1, N  =  AIN3
		_MUX_DIFF_2_3,		// 0x3000 Differential P  =  AIN2, N  =  AIN3
		_MUX_SINGLE_0,		// 0x4000 Single-ended AIN0
		_MUX_SINGLE_1,		// 0x5000 Single-ended AIN1
		_MUX_SINGLE_2,		// 0x6000 Single-ended AIN2
		_MUX_SINGLE_3		// 0x7000 Single-ended AIN3
	};

	constexpr uint16_t _RATEMASK = 0x7 << 5;
	constexpr uint16_t _GAINMASK = 0x7 << 9;

	//--------------------------------------------------------
	void delayMicroseconds( uint32_t aMS )
	{
		std::this_thread::sleep_for(std::chrono::microseconds(aMS));
	}
}	//namespace

//--------------------------------------------------------
class adc
{
public:
	//--------------------------------------------------------
	adc( uint8_t aAddress = _ADDRESS, uint16_t aRate = 4, uint16_t aGain = 2 )
	: _address(aAddress)
	{
		_i2c = CH341OpenDevice(0);
		CH341SetStream(0, 1);					// Set stream speed to default, Set to 2 for fast

		SetRate(aRate);
		SetGain(aGain);
	}

	//--------------------------------------------------------
	~adc(  )
	{
		CH341CloseDevice(0);
	}

	//--------------------------------------------------------
	void SetRate( uint16_t aRate )
	{
		_mode = (_mode & ~_RATEMASK) | (min(7, aRate) << 5);
	}

	//--------------------------------------------------------
	void SetGain( uint16_t aGain )
	{
		_mode = (_mode & ~_GAINMASK) | (min(5, aGain) << 9);
	}

	void Trigger( MUX aMux )
	{
		_write16(_mode | (static_cast<uint16_t>(aMux) << 12), _RA_CONFIG);
	}

	//--------------------------------------------------------
	uint16_t Read( MUX aMux, bool abTrigger = false )
	{
		if (abTrigger) {
			_write16(_mode | (static_cast<uint16_t>(aMux) << 12), _RA_CONFIG);
		}

		//When bit is set the conversion is inactive.
		while ((_read8(_RA_CONFIG) & _OS_NOTBUSY) == 0) {
			std::cout << "waiting\n";
			delayMicroseconds(1);
		}
		return _read16(_RA_CONVERSION);
	}

private:
	HANDLE _i2c = 0;
	uint16_t _mode = _CQUE_NONE | _MODE_SINGLESHOT | _OS_SINGLE;
	uint8_t _address;

	//--------------------------------------------------------
	uint8_t _read8( uint8_t aLoc )
	{
		uint8_t v = 0;
		CH341ReadI2C(0, _address, aLoc, &v);
		return v;
	}

	//--------------------------------------------------------
	uint16_t _read16( uint8_t aLoc )
	{
		uint8_t vh = 0;
		uint8_t vl = 0;
		CH341ReadI2C(0, _address, aLoc, &vh);
		CH341ReadI2C(0, _address, aLoc + 1, &vl);
		return (vh << 8) | (vl & 0xFF);
	}

	//--------------------------------------------------------
	// Write 16 bit integer aVal to given address aLoc.
	void _write16( uint16_t aValue, uint8_t aLoc, uint32_t aDelay = 5 )
	{
		uint8_t data[4];
		data[0] = _address << 1; //Shifted 1 for CH341StreamI2C() for some stupid reason
		data[1] = aLoc;
		data[2] = static_cast<uint8_t>(aValue >> 8);
		data[3] = static_cast<uint8_t>(aValue);

		CH341StreamI2C(0, 4, data, 0, nullptr);

// 		CH341WriteI2C(0, _address, aLoc, static_cast<uint8_t>(aValue >> 8));
// 		CH341WriteI2C(0, _address, aLoc + 1, static_cast<uint8_t>(aValue));
		if (aDelay) {
			delayMicroseconds(aDelay);
		}
	}
};

extern "C"
{

//--------------------------------------------------------
__declspec(dllexport) void *Create(  )
{
	return new adc();
}

//--------------------------------------------------------
__declspec(dllexport) void Release( void *apInstance )
{
	auto padc = reinterpret_cast<adc*>(apInstance);
	delete padc;
}

//--------------------------------------------------------
__declspec(dllexport) void Trigger( void* apInstance, uint16_t aMux )
{
	auto padc = reinterpret_cast<adc*>(apInstance);
	return padc->Trigger(static_cast<MUX>(aMux & 7));
}

//--------------------------------------------------------
__declspec(dllexport) uint16_t Read( void *apInstance, uint16_t aMux, bool abTrigger = false )
{
	auto padc = reinterpret_cast<adc*>(apInstance);
	return padc->Read(static_cast<MUX>(aMux & 7), abTrigger);
}
} //extern C

#if !defined(_WINDLL)

//--------------------------------------------------------
uint32_t main(  )
{
	auto a = Create();
	auto m = static_cast<uint16_t>(MUX::_MUX_SINGLE_0);
	Trigger(a, m);

	while (true) {
		delayMicroseconds(10000);
		uint16_t v = Read(a, m);
		std::cout << "Data: " << v << "      \r";
		Trigger(a, m);
	}
}
#endif //!_WINDLL
