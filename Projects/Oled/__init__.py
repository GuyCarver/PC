#!/usr/bin/env python3

#Python interface to the oledlib.so module.

from ctypes import CDLL, POINTER, c_uint, c_ubyte, c_char_p, c_bool

_lib = CDLL(__path__[0] + '/oled.dll')
_lib.Text.argtypes = [c_uint, c_uint, c_char_p, c_ubyte, c_uint, c_uint]
_lib.GetSize.restype = POINTER(c_uint * 2)
_lib.GetOn.restype = c_bool
_lib.GetInverted.restype = c_bool
_lib.GetRotation.restype = c_ubyte
_lib.GetDim.restype = c_ubyte

#--------------------------------------------------------
#Scrolling values.
STOP = c_ubyte.in_dll(_lib, 'STOP')
LEFT = c_ubyte.in_dll(_lib, 'LEFT')
RIGHT = c_ubyte.in_dll(_lib, 'RIGHT')
DIAGLEFT = c_ubyte.in_dll(_lib, 'DIAGLEFT')
DIAGRIGHT = c_ubyte.in_dll(_lib, 'DIAGRIGHT')

#--------------------------------------------------------
#Fonts for text and char.
TERMINAL = c_ubyte.in_dll(_lib, 'TERMINAL').value
SYS = c_ubyte.in_dll(_lib, 'SYS').value
SERIF = c_ubyte.in_dll(_lib, 'SERIF').value

#--------------------------------------------------------
def startup(  ):
  ''' Initialize the singleton system. '''
  _lib.Startup()

#--------------------------------------------------------
def shutdown(  ):
  ''' Shut down the singleton system. '''
  _lib.Shutdown()

#--------------------------------------------------------
def getsize(  ):
  ''' Get display size (w,h). '''
  return _lib.GetSize().contents

#--------------------------------------------------------
def display(  ):
  ''' Push any text or line drawing to the display. '''
  _lib.Display()

#--------------------------------------------------------
def seton( aTF ):
  ''' Set the display on/off. '''
  _lib.SetOn(aTF)

#--------------------------------------------------------
def geton(  ):
  ''' Get the display on/off. '''
  return _lib.GetOn()

#--------------------------------------------------------
def on(  ):
  ''' Set the display on. '''
  seton(True)

#--------------------------------------------------------
def off(  ):
  ''' Set the display off. '''
  seton(False)

#--------------------------------------------------------
def setinverted( aTF ):
  ''' Set the display inversion. '''
  _lib.SetInverted(aTF)

#--------------------------------------------------------
def getinverted(  ):
  ''' Get the display on/off. '''
  return _lib.GetInverted()

#--------------------------------------------------------
def setrotation( aRotation ):
  ''' Set the rotation to a value 0-3. '''
  _lib.SetRotation(c_ubyte(aRotation))

#--------------------------------------------------------
def getrotation(  ):
  ''' Get the rotation to a value 0-3. '''
  return _lib.GetRotation()

#--------------------------------------------------------
def setdim( aDim ):
  ''' Set the display brightness 0x0-0xFF. '''
  _lib.SetDim(c_ubyte(aDim))

#--------------------------------------------------------
def getdim(  ):
  ''' Get the display brightness 0x0-0xFF. '''
  return _lib.GetDim()

#--------------------------------------------------------
def fill( aValue ):
  ''' Fill the screen with the given value. Each bit represents a horizontal line. '''
  _lib.Fill(c_ubyte(aValue))

#--------------------------------------------------------
def clear(  ):
  ''' Clear the whole screen. '''
  _lib.Clear()

#--------------------------------------------------------
def pixel( aPos, aOn = True ):
  ''' Set the pixel at aPos(x,y). '''
  _lib.Pixel(*aPos, aOn)

#--------------------------------------------------------
def line( aStart, aEnd, aOn = True ):
  ''' Draw a line from aStart(x,y) to aEnd(x,y). '''
  _lib.Line(*aStart, *aEnd, aOn)

#--------------------------------------------------------
def rect( aStart, aSize, aOn = True ):
  ''' Fill given rectangle from aStart(x,y) by aSize(w,h). '''
  _lib.FillRect(*aStart, *aSize, aOn)

#--------------------------------------------------------
def char(aPos, aChar, aOn = True, aFont = TERMINAL, aSize = 1 ):
  ''' Print given text at Pos (x,y). aSize may be an integer value or
       a tuple (x,y). '''
  if type(aSize) == int:
    sx = sy = aSize
  else:
    sx, sy = aSize

  _lib.Char(*aPos, aChar, aOn, aFont, sx, sy)

#--------------------------------------------------------
def text(aPos, aString, aOn = True, aFont = TERMINAL, aSize = 1 ):
  ''' Print given text at Pos (x,y). aSize may be an integer value or
       a tuple (x,y). '''
  if type(aSize) == int:
    sx = sy = aSize
  else:
    sx, sy = aSize

  _lib.Text(*aPos, aString.encode('utf-8'), aOn, aFont, sx, sy)

#--------------------------------------------------------
def scroll( aDirection, aStart, aStop ):
  ''' Scroll the display in the given direction (0-4).
       The area is controlled by start and stop (0-7). '''
  _lib.Scroll(aDirection, aStart, aStop)