#pragma once

#include "vec3f.h"
#include <float.h>
#include <stdlib.h>
#include <cuda_runtime.h>

#define MAX(a,b)	((a) > (b) ? (a) : (b))
#define MIN(a,b)	((a) < (b) ? (a) : (b))

FORCEINLINE void
vmin(vec3f &a, const vec3f &b)
{
	a.set_value(
		MIN(a[0], b[0]),
		MIN(a[1], b[1]),
		MIN(a[2], b[2]));
}

FORCEINLINE void
vmax(vec3f &a, const vec3f &b)
{
	a.set_value(
		MAX(a[0], b[0]),
		MAX(a[1], b[1]),
		MAX(a[2], b[2]));
}


class aabb {
public:
	vec3f _min;
	vec3f _max;

	FORCEINLINE __host__ __device__ aabb() {
		empty();
	}

	FORCEINLINE __host__ __device__ aabb(const vec3f &v) {
		_min = _max = v;
	}

	FORCEINLINE aabb(const vec3f &a, const vec3f &b) {
		_min = a;
		_max = a;
		vmin(_min, b);
		vmax(_max, b);
	}

	FORCEINLINE bool __device__ __host__ overlaps(const aabb& b) const
	{
		if (_min[0] > b._max[0]) return false;
		if (_min[1] > b._max[1]) return false;
		if (_min[2] > b._max[2]) return false;

		if (_max[0] < b._min[0]) return false;
		if (_max[1] < b._min[1]) return false;
		if (_max[2] < b._min[2]) return false;

		return true;
	}

	FORCEINLINE bool overlaps(const aabb &b, aabb &ret) const
	{
		if (!overlaps(b))
			return false;

		ret._min.set_value(
			MAX(_min[0],  b._min[0]),
			MAX(_min[1],  b._min[1]),
			MAX(_min[2],  b._min[2]));

		ret._max.set_value(
			MIN(_max[0], b._max[0]),
			MIN(_max[1], b._max[1]),
			MIN(_max[2], b._max[2]));

		return true;
	}

	FORCEINLINE bool inside(const vec3f &p) const
	{
		if (p[0] < _min[0] || p[0] > _max[0]) return false;
		if (p[1] < _min[1] || p[1] > _max[1]) return false;
		if (p[2] < _min[2] || p[2] > _max[2]) return false;

		return true;
	}

	FORCEINLINE aabb &operator += (const vec3f &p)
	{
		vmin(_min, p);
		vmax(_max, p);
		return *this;
	}

	FORCEINLINE aabb &operator += (const aabb &b)
	{
		vmin(_min, b._min);
		vmax(_max, b._max);
		return *this;
	}

	FORCEINLINE __host__ __device__ aabb operator + (const aabb &v) const
		{ aabb rt(*this); return rt += v; }

	FORCEINLINE float width()  const { return _max[0] - _min[0]; }
	FORCEINLINE float height() const { return _max[1] - _min[1]; }
	FORCEINLINE float depth()  const { return _max[2] - _min[2]; }
	FORCEINLINE vec3f center() const { return (_min+_max)*0.5; }
	FORCEINLINE float volume() const { return width()*height()*depth(); }

	FORCEINLINE void empty() {
		_max = vec3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		_min = vec3f(FLT_MAX, FLT_MAX, FLT_MAX);
	}

	FORCEINLINE void enlarge(float thickness) {
		_max += vec3f(thickness, thickness, thickness);
		_min -= vec3f(thickness, thickness, thickness);
	}

	FORCEINLINE __host__ __device__  vec3f min_value(const vec3f &a, const vec3f &b)
	{
		return vec3f(a.x < b.x ? a.x : b.x,
			a.y < b.y ? a.y : b.y,
			a.z < b.z ? a.z : b.z);
	}

	FORCEINLINE __host__ __device__  vec3f max_value(const vec3f &a, const vec3f &b)
	{
		return vec3f(a.x > b.x ? a.x : b.x,
			a.y > b.y ? a.y : b.y,
			a.z > b.z ? a.z : b.z);
	}

	FORCEINLINE __host__ __device__  void set(const aabb &a, const aabb &b)
	{
		_min = min_value(a._min, b._min);
		_max = max_value(a._max, b._max);
	}

	FORCEINLINE __host__ __device__  void enlarge(const aabb &a)
	{
		_min = min_value(_min, a._min);
		_max = max_value(_max, a._max);
	}

	FORCEINLINE __host__ __device__  void set(vec3f &a){
		_min = _max = a;
	}

	vec3f getMax() { return _max; }
	vec3f getMin() { return _min; }

//	void visualize();
};