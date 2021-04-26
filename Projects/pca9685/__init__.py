#!/usr/bin/env python3
# pca9865 servo controller driver.

from ctypes import CDLL, c_bool, c_float

# Located in usr/local/bin
_lib = CDLL(__path__[0] + '/pca9685.dll')

def startup(  ):
  ''' Initialize the singleton version of the system. '''
  b = c_bool(_lib.Startup())
  return b.value

def isgood(  ):
  ''' Returns true if the system is setup successfully. '''
  b = c_bool(_lib.IsGood())
  return b.value

def shutdown(  ):
  ''' Shut down the singleton instance of this system. '''
  _lib.Shutdown()

def setfreq( aFreq ):
  ''' Set the PWM signal frequency. '''
  _lib.SetFreq(aFreq)

def setpwm( aIndex, aOn, aOff ):
  ''' Set PWM for given index. Values from 0-4095. '''
  _lib.SetPWM(aIndex, aOn, aOff)

def off( aIndex ):
  ''' Turn given index off. '''
  _lib.Off(aIndex)

def alloff(  ):
  ''' Turn all slots off. '''
  _lib.AllOff()

def set( aIndex, aValue ):
  ''' Set index to value 0.0 - 1.0. '''
  _lib.Set(aIndex, c_float(aValue))

def setangle( aIndex, aAngle ):
  ''' Set index to angle -90.0 - 90.0. '''
  _lib.SetAngle(aIndex, c_float(aAngle))
