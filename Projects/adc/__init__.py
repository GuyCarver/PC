#!/usr/bin/env python3
# ads1115 adc driver.

from ctypes import CDLL, c_void_p

_lib = CDLL(__path__[0] + '/adc.dll')
_lib.Create.restype = c_void_p

def create(  ):
  return _lib.Create()

def release( aInstance ):
  _lib.Release(aInstance)

def read( aMux ):
  '''aMux = '''
  return _lib.Read(aMux)

