#!/usr/bin/env python3
# ads1115 adc driver.
#The Address pin must be set to ground in order for the default address to work.
# If set to sda, scl or vcc then different addresses are set.
# At this time the module does not support different addresses.
# Addresses start at 0x48 and go up in this order - GND, VCC, SDA, SCL.

from ctypes import CDLL, c_void_p, c_ubyte

_lib = CDLL(__path__[0] + '/adc.dll')
_lib.Create.restype = c_void_p
_lib.Create.argtypes = [c_ubyte]

def create( aAddress ):
  return _lib.Create(aAddress)

def release( aInstance ):
  _lib.Release(aInstance)

def read( aInstance, aMux ):
  '''aMux = value from 0 - 7 representing read type.
     _MUX_DIFF_0_1,    // 0x0000 Differential P  =  AIN0, N  =  AIN1 (default)
     _MUX_DIFF_0_3,    // 0x1000 Differential P  =  AIN0, N  =  AIN3
     _MUX_DIFF_1_3,    // 0x2000 Differential P  =  AIN1, N  =  AIN3
     _MUX_DIFF_2_3,    // 0x3000 Differential P  =  AIN2, N  =  AIN3
     _MUX_SINGLE_0,    // 0x4000 Single-ended AIN0
     _MUX_SINGLE_1,    // 0x5000 Single-ended AIN1
     _MUX_SINGLE_2,    // 0x6000 Single-ended AIN2
     _MUX_SINGLE_3   // 0x7000 Single-ended AIN3
  '''
  return _lib.Read(aInstance, aMux)


