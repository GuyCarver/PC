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
// FILE    Matrix3.cpp
// DATE    03/04/2021 08:42 AM
//----------------------------------------------------------------------

#include "matrix3.h"
#include <cmath>
#include <cstdint>

Matrix3::Matrix3(  )
{
	MakeIdentity();
}

void Matrix3::MakeIdentity(  )
{
	Entries[0][0] = 1.0f;
	Entries[0][1] = 0.0f;
	Entries[0][2] = 0.0f;
	Entries[1][0] = 0.0f;
	Entries[1][1] = 1.0f;
	Entries[1][2] = 0.0f;
	Entries[2][0] = 0.0f;
	Entries[2][1] = 0.0f;
	Entries[2][2] = 1.0f;
}

///
/// <summary> Create matrix from given array of 3 euler angles (XYZ) in XYZ order. </summary>
///
void Matrix3::EulerXYZ( float *aEulers )
{
	Matrix3 x, y, z;

	x.MakeXRotation(aEulers[0]);
	y.MakeYRotation(aEulers[1]);
	z.MakeZRotation(aEulers[2]);

	*this = z * y * x;
}

void Matrix3::MakeXRotation( float aAngle )
{
	float s = sin(aAngle);
	float c = cos(aAngle);

	Entries[0][0] = 1.0f;
	Entries[0][1] = 0.0f;
	Entries[0][2] = 0.0f;
	Entries[1][0] = 0.0f;
	Entries[1][1] = c;
	Entries[1][2] = s;
	Entries[2][0] = 0.0f;
	Entries[2][1] = -s;
	Entries[2][2] = c;
}

void Matrix3::MakeYRotation( float aAngle )
{
	float s = sin(aAngle);
	float c = cos(aAngle);

	Entries[0][0] = c;
	Entries[0][1] = 0.0f;
	Entries[0][2] = -s;
	Entries[1][0] = 0.0f;
	Entries[1][1] = 1.0f;
	Entries[1][2] = 0.0f;
	Entries[2][0] = s;
	Entries[2][1] = 0.0f;
	Entries[2][2] = c;
}

void Matrix3::MakeZRotation( float aAngle )
{
	float s = sin(aAngle);
	float c = cos(aAngle);

	Entries[0][0] = c;
	Entries[0][1] = s;
	Entries[0][2] = 0.0f;
	Entries[1][0] = -s;
	Entries[1][1] = c;
	Entries[1][2] = 0.0f;
	Entries[2][0] = 0.0f;
	Entries[2][1] = 0.0f;
	Entries[2][2] = 1.0f;
}

Matrix3 Matrix3::operator *( const Matrix3 &aMat) const
{
	Matrix3 prd;

	prd.Entries[0][0] =
		Entries[0][0] * aMat.Entries[0][0]+
		Entries[1][0] * aMat.Entries[0][1]+
		Entries[2][0] * aMat.Entries[0][2];
	prd.Entries[0][1] =
		Entries[0][1] * aMat.Entries[0][0]+
		Entries[1][1] * aMat.Entries[0][1]+
		Entries[2][1] * aMat.Entries[0][2];
	prd.Entries[0][2] =
		Entries[0][2] * aMat.Entries[0][0]+
		Entries[1][2] * aMat.Entries[0][1]+
		Entries[2][2] * aMat.Entries[0][2];
	prd.Entries[1][0] =
		Entries[0][0] * aMat.Entries[1][0]+
		Entries[1][0] * aMat.Entries[1][1]+
		Entries[2][0] * aMat.Entries[1][2];
	prd.Entries[1][1] =
		Entries[0][1] * aMat.Entries[1][0]+
		Entries[1][1] * aMat.Entries[1][1]+
		Entries[2][1] * aMat.Entries[1][2];
	prd.Entries[1][2] =
		Entries[0][2] * aMat.Entries[1][0]+
		Entries[1][2] * aMat.Entries[1][1]+
		Entries[2][2] * aMat.Entries[1][2];
	prd.Entries[2][0] =
		Entries[0][0] * aMat.Entries[2][0]+
		Entries[1][0] * aMat.Entries[2][1]+
		Entries[2][0] * aMat.Entries[2][2];
	prd.Entries[2][1] =
		Entries[0][1] * aMat.Entries[2][0]+
		Entries[1][1] * aMat.Entries[2][1]+
		Entries[2][1] * aMat.Entries[2][2];
	prd.Entries[2][2] =
		Entries[0][2] * aMat.Entries[2][0]+
		Entries[1][2] * aMat.Entries[2][1]+
		Entries[2][2] * aMat.Entries[2][2];

	return prd;
}

void Matrix3::Rotate( float *aVector )
{
	float x = aVector[0];
	float y = aVector[1];
	float z = aVector[2];

	aVector[0] = Entries[0][0] * x + Entries[1][0] * y + Entries[2][0] * z;
	aVector[1] = Entries[0][1] * x + Entries[1][1] * y + Entries[2][1] * z;
	aVector[2] = Entries[0][2] * x + Entries[1][2] * y + Entries[2][2] * z;
}

extern "C"
{
	void *ZRotation( float aAngle )
	{
		Matrix3 *pmat = new Matrix3();
		pmat->MakeZRotation(aAngle);
		return pmat;
	}

	void Release( void *Handle )
	{
		auto pmat = reinterpret_cast<Matrix3*>(Handle);
		delete pmat;
	}

	float *Rotate( void *Handle, float x, float y, float z )
	{
		static float vals[3];
		vals[0] = x;
		vals[1] = y;
		vals[2] = z;

		auto pmat = reinterpret_cast<Matrix3*>(Handle);
		pmat->Rotate(vals);
		return vals;
	}
} //extern "C"