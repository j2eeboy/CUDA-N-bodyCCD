/*************************************************************************\

  Copyright 2010 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
   fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             GAMMA Research Group at UNC
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:              geom@cs.unc.edu; tang_m@zju.edu.cn


\**************************************************************************/

#pragma once

#include <math.h>

#include "forceline.h"
#include "cuda_callable.h"
#define     GLH_ZERO            float(0.0)
#define     GLH_EPSILON         float(10e-6)
#define		GLH_EPSILON_2		float(10e-12)
#define     equivalent_float(a,b)     (((a < b + GLH_EPSILON) && (a > b - GLH_EPSILON)) ? true : false)


#ifndef M_PI
#define M_PI 3.14159f
#endif

#include <assert.h>

class vec3f {
public:

    union {
        struct {
            float x, y, z;
        };
        struct {
            float v[3];
        };
    };

	FORCEINLINE float length() const {
		return float(sqrt(x*x + y*y + z*z));
	}

	FORCEINLINE vec3f getUnit() const {
		return (*this) / length();
	}

    CUDA_CALLABLE_MEMBER FORCEINLINE vec3f ()
    {x=0; y=0; z=0;}

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f(const vec3f &v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f(const float *v)
    {
        x = v[0];
        y = v[1];
        z = v[2];
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE float operator [] (int i) const { return v[i]; }

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f &operator += (const vec3f &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f &operator -= (const vec3f &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE void negate() {
        x = -x;
        y = -y;
        z = -z;
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f operator - () const {
        return vec3f(-x, -y, -z);
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f operator+ (const vec3f &v) const
    {
        return vec3f(x+v.x, y+v.y, z+v.z);
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f operator- (const vec3f &v) const
    {
        return vec3f(x-v.x, y-v.y, z-v.z);
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f operator *(float t) const
    {
        return vec3f(x*t, y*t, z*t);
    }
	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f operator /(float t) const
    {
        return vec3f(x/t, y/t, z/t);
    }
    // cross product
	CUDA_CALLABLE_MEMBER FORCEINLINE const vec3f cross(const vec3f &vec) const
    {
        return vec3f(y*vec.z - z*vec.y, z*vec.x - x*vec.z, x*vec.y - y*vec.x);
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE float dot(const vec3f &vec) const {
        return x*vec.x+y*vec.y+z*vec.z;
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE void normalize()
    {
        float sum = x*x+y*y+z*z;
        if (sum > GLH_EPSILON_2) {
            float base = float(1.0/sqrt(sum));
            x *= base;
            y *= base;
            z *= base;
        }
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE float length() {
        return float(sqrt(x*x + y*y + z*z));
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE vec3f & set_value(const float &vx, const float &vy, const float &vz)
    { x = vx; y = vy; z = vz; return *this; }

	CUDA_CALLABLE_MEMBER FORCEINLINE bool equal_abs(const vec3f &other) {
        return x == other.x && y == other.y && z == other.z;
    }

	CUDA_CALLABLE_MEMBER FORCEINLINE float square_norm() const {
        return x*x+y*y+z*z;
    }
};

#include <vector>
typedef std::vector<vec3f> vec3f_list;
