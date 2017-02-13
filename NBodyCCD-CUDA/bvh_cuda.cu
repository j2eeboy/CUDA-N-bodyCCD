#pragma once
#include "bvh.h"
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>
#include "Cuda_Support.h"
#include"myfeature.h"
#include "result2b.h"

#define zeroRes float(10e-8)
#define IsZero(x) ((x) < zeroRes && (x) > -zeroRes ? true : false)
#define M_PI        3.14159265358979323846f
#define EQN_EPS float(1e-9)
#define ONE_DIV_3  0.33333333333333333f
#define MAX_SIZE 1000
int *g_flag_collide;


typedef struct {
	float3 ad, bd, cd, pd;
	float3 a0, b0, c0, p0;
} NewtonCheckData;

typedef struct {
	int _num;
	int *_bvh_ids;
	aabb *_bxs;
} g_bvh;

typedef struct {
	int _num_vtx, _num_tri, _num_edge;
	tri3f *_tris, *_tri_edges;
	vec3f *_orig_vtxs, *_init_vtxs, *_vtxs;
	aabb *_fbxs;
	unsigned int *_masks;
	edge4f *_edges;
} g_mesh;

inline __device__  int isZero(float x)
{
	return x > -EQN_EPS && x < EQN_EPS;
}


thrust::device_vector<g_bvh> bvh_list;
thrust::device_vector<g_mesh> mesh_list;
thrust::device_vector<float> trans_list;

__global__ void kernelUpdateBox(KernelArray<g_mesh> gmesh_list, int size)
{
	int idx = blockDim.x*blockIdx.x + threadIdx.x;
	if (idx >= size)  return;
	g_mesh &mesh_ptr = gmesh_list[idx];
	for (int i = 0; i < mesh_ptr._num_tri; i++) {
		mesh_ptr._fbxs[i].set(mesh_ptr._orig_vtxs[mesh_ptr._tris[i]._ids[0]], mesh_ptr._orig_vtxs[mesh_ptr._tris[i]._ids[1]]);
	}
}


inline __device__ int getTriID(uint i, int *bvh_ids)
{
	return bvh_ids[i];
}

inline __device__ bool isLeaf(uint i, int *bvh_ids)
{
	return bvh_ids[i] >= 0;
}

inline __device__ int getLeftChild(uint i, int *bvh_ids)
{
	return i - bvh_ids[i];
}

inline __device__ int getRightChild(uint i, int *bvh_ids)
{
	return i - bvh_ids[i] + 1;
}

__global__ void kernelRefit(KernelArray<g_mesh> gmesh_list, KernelArray<g_bvh> gbvh_list, int size){
	int idx = blockDim.x*blockIdx.x + threadIdx.x;
	if (idx >= size) return;
	g_mesh &mesh_ptr = gmesh_list[idx];
	g_bvh &bvh_ptr = gbvh_list[idx];
	for (int i = bvh_ptr._num - 1; i >= 0; i--) {
		if (isLeaf(i, bvh_ptr._bvh_ids)) // isLeaf
		{
			bvh_ptr._bxs[i] = mesh_ptr._fbxs[getTriID(i, bvh_ptr._bvh_ids)];
		}
		else
		{
			int left = getLeftChild(i, bvh_ptr._bvh_ids);
			int right = getRightChild(i, bvh_ptr._bvh_ids);

			bvh_ptr._bxs[i].set(bvh_ptr._bxs[left], bvh_ptr._bxs[right]);
		}
	}
}
 
inline __device__ bool overlaps(int i, aabb *btx1, int j, aabb *btx2)
{
	if (btx1[i]._min.x > btx2[j]._max.x) return false;
	if (btx1[i]._min.y > btx2[j]._max.y) return false;
	if (btx1[i]._min.z > btx2[j]._max.z) return false;

	if (btx1[i]._max.x < btx2[j]._min.x) return false;
	if (btx1[i]._max.y < btx2[j]._min.y) return false;
	if (btx1[i]._max.z < btx2[j]._min.z) return false;

	return true;
}

inline __device__ bool VtxMask(uint *maskes, uint tri_id, uint i)
{
	return maskes[tri_id] & (0x1 << i) ? true : false;
}

inline __device__ void _equateCubic_VF(
	float3 a0, float3 ad, float3 b0, float3 bd,
	float3 c0, float3 cd, float3 p0, float3 pd,
	float &a, float &b, float &c, float &d)
{
	/*
	* For definitions & notation refer to the semester thesis doc.
	*/
	float3 dab, dac, dap;
	float3 oab, oac, oap;
	float3 dabXdac, dabXoac, oabXdac, oabXoac;

	dab = bd - ad, dac = cd - ad, dap = pd - ad;
	oab = b0 - a0, oac = c0 - a0, oap = p0 - a0;
	dabXdac = cross(dab, dac);
	dabXoac = cross(dab, oac);
	oabXdac = cross(oab, dac);
	oabXoac = cross(oab, oac);

	a = dot(dap, dabXdac);
	b = dot(oap, dabXdac) + dot(dap, dabXoac + oabXdac);
	c = dot(dap, oabXoac) + dot(oap, dabXoac + oabXdac);
	d = dot(oap, oabXoac);

	//if (d > 0)
	//	d -= DEFAULT_THICKNESS;
}

inline __device__ int solveLinear(float c[2], float s[1])
{
	if (isZero(c[1]))
		return 0;
	s[0] = -c[0] / c[1];
	return 1;
}

inline __device__ int solveQuadric(float c[3], float s[2])
{
	float p, q, D;

	// make sure we have a d2 equation

	if (isZero(c[2]))
		return solveLinear(c, s);

	// normal for: x^2 + px + q
	p = c[1] / (2.0f * c[2]);
	q = c[0] / c[2];
	D = p * p - q;

	if (isZero(D))
	{
		// one float root
		s[0] = s[1] = -p;
		return 1;
	}

	if (D < 0.0f)
		// no real root
		return 0;

	else
	{
		// two real roots
		float sqrt_D = sqrt(D);
		s[0] = sqrt_D - p;
		s[1] = -sqrt_D - p;
		return 2;
	}
}

inline __device__ int solveCubic(float c[4], float s[3])
{
	int	i, num;
	float	sub,
		A, B, C,
		sq_A, p, q,
		cb_p, D;

	// make sure we have a d2 equation

	if (isZero(c[3])) {
		return solveQuadric(c, s);
	}

	// normalize the equation:x ^ 3 + Ax ^ 2 + Bx  + C = 0
	A = c[2] / c[3];
	B = c[1] / c[3];
	C = c[0] / c[3];

	// substitute x = y - A / 3 to eliminate the quadric term: x^3 + px + q = 0

	sq_A = A * A;
	p = ONE_DIV_3 * (-ONE_DIV_3 * sq_A + B);
	q = 0.5f * (2.0f / 27.0f * A *sq_A - ONE_DIV_3 * A * B + C);

	// use Cardano's formula

	cb_p = p * p * p;
	D = q * q + cb_p;

	if (isZero(D))
	{
		if (isZero(q))
		{
			// one triple solution
			s[0] = 0.0f;
			num = 1;
		}
		else
		{
			// one single and one float solution
			float u = cbrt(-q);
			s[0] = 2.0f * u;
			s[1] = -u;
			num = 2;
		}
	}
	else
		if (D < 0.0f)
		{
		// casus irreductibilis: three real solutions
		float phi = ONE_DIV_3 * acos(-q / sqrt(-cb_p));
		float t = 2.0f * sqrt(-p);
		s[0] = t * cos(phi);
		s[1] = -t * cos(phi + M_PI / 3.0f);
		s[2] = -t * cos(phi - M_PI / 3.0f);
		num = 3;
		}
		else
		{
			// one real solution
			float sqrt_D = sqrt(D);
			float u = cbrt(sqrt_D + fabs(q));
			if (q > 0.0f)
				s[0] = -u + p / u;
			else
				s[0] = u - p / u;
			num = 1;
		}

	// resubstitute
	sub = ONE_DIV_3 * A;
	for (i = 0; i < num; i++)
		s[i] -= sub;
	return num;
}

inline __device__ bool _insideTriangle(float3 a, float3 b, float3 c, float3 p, float3 &baryc)
{
	float3 n, da, db, dc;
	float wa, wb, wc;

	float3 ba = b - a;
	float3 ca = c - a;
	n = cross(ba, ca);

	da = a - p, db = b - p, dc = c - p;
	if ((wa = dot(cross(db, dc), n)) < 0.0f) return false;
	if ((wb = dot(cross(dc, da), n)) < 0.0f) return false;
	if ((wc = dot(cross(da, db), n)) < 0.0f) return false;

	//Compute barycentric coordinates
	float area2 = dot(n, n);
	wa /= area2, wb /= area2, wc /= area2;

	baryc = make_float3(wa, wb, wc);

	return true;
}

inline __device__ bool checkRootValidity_VF(float t, float3 & baryc, NewtonCheckData &data) {
	return _insideTriangle(
		data.ad*t + data.a0,
		data.bd*t + data.b0,
		data.cd*t + data.c0,
		data.pd*t + data.p0,
		baryc);
}

inline __device__ float IntersectVF(float3 ta0, float3 tb0, float3 tc0,
	float3 ta1, float3 tb1, float3 tc1,
	float3 q0, float3 q1,
	float3 &qi, float3 &baryc)
{
	/* Default value returned if no collision occurs */
	float collisionTime = -1.0f;

	float3 qd, ad, bd, cd;
	/* diff. vectors for linear interpolation */
	qd = q1 - q0, ad = ta1 - ta0, bd = tb1 - tb0, cd = tc1 - tc0;

	/*
	* Compute scalar coefficients by evaluating dot and cross-products.
	*/
	float a, b, c, d; /* cubic polynomial coefficients */
	_equateCubic_VF(ta0, ad, tb0, bd, tc0, cd, q0, qd, a, b, c, d);

	if (IsZero(a) && IsZero(b) && IsZero(c) && IsZero(d))
		return -1.f;

	float roots[3];
	float coeffs[4];
	coeffs[3] = a, coeffs[2] = b, coeffs[1] = c, coeffs[0] = d;
	int num = solveCubic(coeffs, roots);

	if (num == 0)
		return -1.f;

	NewtonCheckData data;
	data.a0 = ta0, data.b0 = tb0;
	data.c0 = tc0, data.p0 = q0;
	data.ad = ad, data.bd = bd;
	data.cd = cd, data.pd = qd;

	for (int i = 0; i<num; i++) {
		float r = roots[i];
		if (r < 0 || r > 1) continue;

		if (checkRootValidity_VF(r, baryc, data)) {
			collisionTime = r;
			break;
		}
	}

	if (collisionTime >= 0)
		qi = qd*collisionTime + q0;

	return collisionTime;
}

inline __device__ bool LineLineIntersect(
	float3 p1, float3 p2, float3 p3, float3 p4,
	float3 &pa, float3 &pb, float &mua, float &mub)
{
	float3 p13, p43, p21;
	float d1343, d4321, d1321, d4343, d2121;
	float numer, denom;

	p13 = p1 - p3;
	p43 = p4 - p3;
	if (fabs(p43.x)  < GLH_EPSILON && fabs(p43.y)  < GLH_EPSILON && fabs(p43.z)  < GLH_EPSILON)
		return false;

	p21 = p2 - p1;
	if (fabs(p21.x)  < GLH_EPSILON && fabs(p21.y)  < GLH_EPSILON && fabs(p21.z)  < GLH_EPSILON)
		return false;

	d1343 = dot(p13, p43);
	d4321 = dot(p43, p21);
	d1321 = dot(p13, p21);
	d4343 = dot(p43, p43);
	d2121 = dot(p21, p21);

	denom = d2121 * d4343 - d4321 * d4321;
	if (fabs(denom) < GLH_EPSILON_2)
		return false;
	numer = d1343 * d4321 - d1321 * d4343;

	mua = numer / denom;
	if (mua < 0 || mua > 1)
		return false;

	mub = (d1343 + d4321 * mua) / d4343;
	if (mub < 0 || mub > 1)
		return false;

	pa = p1 + p21*mua;
	pb = p3 + p43*mub;
	return true;
}

inline __device__ bool checkRootValidity_EE(float t, float3 &pt, NewtonCheckData &data) {
	float3 a = data.ad*t + data.a0;
	float3 b = data.bd*t + data.b0;
	float3 c = data.cd*t + data.c0;
	float3 d = data.pd*t + data.p0;

	float3 p1, p2;
	float tab, tcd;

	if (LineLineIntersect(a, b, c, d, p1, p2, tab, tcd)) {
		t = tab;
		pt = p1;
		return true;
	}

	return false;
}

inline __device__ void _equateCubic_EE(float3 a0, float3 ad, float3 b0, float3 bd,
	float3 c0, float3 cd, float3 d0, float3 dd,
	float &a, float &b, float &c, float &d)
{
	/*
	* For definitions & notation refer to the semester thesis doc.
	*/
	float3 dba, ddc, dca;
	float3 odc, oba, oca;
	float3 dbaXddc, dbaXodc, obaXddc, obaXodc;

	dba = bd - ad, ddc = dd - cd, dca = cd - ad;
	odc = d0 - c0, oba = b0 - a0, oca = c0 - a0;
	dbaXddc = cross(dba, ddc);
	dbaXodc = cross(dba, odc);
	obaXddc = cross(oba, ddc);
	obaXodc = cross(oba, odc);

	a = dot(dca, dbaXddc);
	b = dot(oca, dbaXddc) + dot(dca, dbaXodc + obaXddc);
	c = dot(dca, obaXodc) + dot(oca, dbaXodc + obaXddc);
	d = dot(oca, obaXodc);

	//if (d > 0)
	//	d -= DEFAULT_THICKNESS;
}

inline __device__ bool IntersectEE(
	float3 ta0, float3 tb0, float3 tc0, float3 td0,
	float3 ta1, float3 tb1, float3 tc1, float3 td1, float3 &qi)
{
	/* Default value returned if no collision occurs */
	float collisionTime = -1.0f;

	float3 ad, bd, cd, dd;
	/* diff. vectors for linear interpolation */
	dd = td1 - td0, ad = ta1 - ta0, bd = tb1 - tb0, cd = tc1 - tc0;

	/*
	* Compute scalar coefficients by evaluating dot and cross-products.
	*/
	float a, b, c, d; /* cubic polynomial coefficients */
	_equateCubic_EE(ta0, ad, tb0, bd, tc0, cd, td0, dd, a, b, c, d);

	if (IsZero(a) && IsZero(b) && IsZero(c) && IsZero(d))
		return -1.f;

	float roots[3];
	float coeffs[4];
	coeffs[3] = a, coeffs[2] = b, coeffs[1] = c, coeffs[0] = d;
	int num = solveCubic(coeffs, roots);

	if (num == 0)
		return -1.f;

	NewtonCheckData data;
	data.a0 = ta0, data.b0 = tb0;
	data.c0 = tc0, data.p0 = td0;
	data.ad = ad, data.bd = bd;
	data.cd = cd, data.pd = dd;

	for (int i = 0; i<num; i++) {
		float r = roots[i];
		if (r < 0 || r > 1) continue;

		if (checkRootValidity_EE(r, qi, data)) {
			collisionTime = r;
			break;
		}
	}

	return collisionTime;
}

inline __device__ void doCCDEE(uint Ae, uint Be, edge4f *Aedges, vec3f *AorigVtxs, vec3f *AVtxs, edge4f *Bedges, vec3f*BorigVtxs, vec3f *BVtxs,
	int idx, int *flag_collide, int &nIdx)
{
	edge4f e1 = Aedges[Ae];
	edge4f e2 = Bedges[Be];

	float3 x1 = make_float3(AVtxs[e1._vids[0]].x, AVtxs[e1._vids[0]].y, AVtxs[e1._vids[0]].z);
	float3 x2 = make_float3(AVtxs[e1._vids[1]].x, AVtxs[e1._vids[1]].y, AVtxs[e1._vids[1]].z);
	float3 x3 = make_float3(BVtxs[e2._vids[0]].x, BVtxs[e2._vids[0]].y, BVtxs[e2._vids[0]].z);
	float3 x4 = make_float3(BVtxs[e2._vids[1]].x, BVtxs[e2._vids[1]].y, BVtxs[e2._vids[1]].z);

	float3 ox1 = make_float3(AorigVtxs[e1._vids[0]].x, AorigVtxs[e1._vids[0]].y, AorigVtxs[e1._vids[0]].z);
	float3 ox2 = make_float3(AorigVtxs[e1._vids[1]].x, AorigVtxs[e1._vids[1]].y, AorigVtxs[e1._vids[1]].z);
	float3 ox3 = make_float3(BorigVtxs[e2._vids[0]].x, BorigVtxs[e2._vids[0]].y, BorigVtxs[e2._vids[0]].z);
	float3 ox4 = make_float3(BorigVtxs[e2._vids[1]].x, BorigVtxs[e2._vids[1]].y, BorigVtxs[e2._vids[1]].z);
	float3 qi;

	float ret = IntersectEE(ox1, ox2, ox3, ox4, x1, x2, x3, x4, qi);
	if (ret < -0.5f)
		return;
	nIdx=0;
	flag_collide[idx]++;
	return;
}

inline __device__ void doCCDVF(uint vid, vec3f *origVtxs1, vec3f *Vtxs1, uint fid, tri3f *tris2, vec3f *origVtxs2, vec3f *Vtxs2, 
	int idx, int *flag_collide, int &nIdx)
{
	tri3f t = tris2[fid];
	float3 ox1 = make_float3(origVtxs2[t._ids[0]].x, origVtxs2[t._ids[0]].y, origVtxs2[t._ids[0]].z);
	float3 ox2 = make_float3(origVtxs2[t._ids[1]].x, origVtxs2[t._ids[1]].y, origVtxs2[t._ids[1]].z);
	float3 ox3 = make_float3(origVtxs2[t._ids[2]].x, origVtxs2[t._ids[2]].y, origVtxs2[t._ids[2]].z);
	float3 ox4 = make_float3(origVtxs1[vid].x, origVtxs1[vid].y, origVtxs1[vid].z);
	float3 x1 = make_float3(Vtxs2[t._ids[0]].x, Vtxs2[t._ids[0]].y, Vtxs2[t._ids[0]].z);
	float3 x2 = make_float3(Vtxs2[t._ids[1]].x, Vtxs2[t._ids[1]].y, Vtxs2[t._ids[1]].z);
	float3 x3 = make_float3(Vtxs2[t._ids[2]].x, Vtxs2[t._ids[2]].y, Vtxs2[t._ids[2]].z);
	float3 x4 = make_float3(Vtxs1[vid].x, Vtxs1[vid].y, Vtxs1[vid].z);

	float3 qi, w;
	float ret = IntersectVF(ox1, ox2, ox3, x1, x2, x3, ox4, x4, qi, w);

	if (ret < -0.5f)
		return;
	nIdx = 0;
	flag_collide[idx]++;
	return;
}

inline __device__	bool EdgeMask(uint *maskes, uint tri_id, uint i)
{
	return maskes[tri_id] & (0x8 << i) ? true : false;
}

#define DO_CCD_VF doCCDVF(vid, org_vtxs1, vtxs1, j, tris2, org_vtxs2, vtxs2, idx, flag_collide, nIdx)
#define DO_CCD_FV doCCDVF(vid, org_vtxs2, vtxs2, i, tris1, org_vtxs1, vtxs1, idx, flag_collide, nIdx)
#define DO_CCD_EE doCCDEE(eid1, eid2, edge1, org_vtxs1, vtxs1, edge2, org_vtxs2, vtxs2, idx, flag_collide, nIdx)
__device__ void intersect(int i, tri3f *tris1, vec3f *org_vtxs1, vec3f *vtxs1, uint *masks1,
	int j, tri3f *tris2, vec3f *org_vtxs2, vec3f *vtxs2, uint *masks2, tri3f *tri_edge1, tri3f *tri_edge2, edge4f *edge1, edge4f *edge2, 
	int idx, int *flag_collide, int &nIdx)
{
	{
	tri3f face1 = tris1[i], face2 = tris2[j];
	uint vid = face1._ids[0];
	if (VtxMask(masks1, i, 0))
		DO_CCD_VF;
	vid = face1._ids[1];
	if (VtxMask(masks1, i, 1))
		DO_CCD_VF;
	vid = face1._ids[2];
	if (VtxMask(masks1, i, 2))
		DO_CCD_VF;
	vid = face2._ids[0];
	if (VtxMask(masks2, j, 0))
		DO_CCD_FV;
	vid = face2._ids[1];
	if (VtxMask(masks2, j, 1))
		DO_CCD_FV;
	vid = face2._ids[2];
	if (VtxMask(masks2, j, 2))
		DO_CCD_FV;
	}
	{
	tri3f tri_e1 = tri_edge1[i], tri_e2 = tri_edge2[j];
	uint eid1 = tri_e1._ids[0], eid2 = tri_e1._ids[0];
	if (EdgeMask(masks1, i, 0) && EdgeMask(masks2, j, 0))
		DO_CCD_EE;
	eid2 = tri_e1._ids[1];
	if (EdgeMask(masks1, i, 0) && EdgeMask(masks2, j, 1))
		DO_CCD_EE;
	eid2 = tri_e1._ids[2];
	if (EdgeMask(masks1, i, 0) && EdgeMask(masks2, j, 2))
		DO_CCD_EE;
	eid1 = tri_e1._ids[1]; eid2 = tri_e1._ids[0];
	if (EdgeMask(masks1, i, 1) && EdgeMask(masks2, j, 0))
		DO_CCD_EE;
	eid2 = tri_e1._ids[1];
	if (EdgeMask(masks1, i, 1) && EdgeMask(masks2, j, 1))
		DO_CCD_EE;
	eid2 = tri_e1._ids[2];
	if (EdgeMask(masks1, i, 1) && EdgeMask(masks2, j, 2))
		DO_CCD_EE;
	eid1 = tri_e1._ids[2]; eid2 = tri_e1._ids[0];
	if (EdgeMask(masks1, i, 2) && EdgeMask(masks2, j, 0))
		DO_CCD_EE;
	eid2 = tri_e1._ids[1];
	if (EdgeMask(masks1, i, 2) && EdgeMask(masks2, j, 1))
		DO_CCD_EE;
	eid2 = tri_e1._ids[2];
	if (EdgeMask(masks1, i, 2) && EdgeMask(masks2, j, 2))
		DO_CCD_EE;
	}
}

#define STACK_SIZE 50
#define EMPTY (nIdx == 0)

#define PUSH_PAIR(nd1, nd2)  {\
	nStack[nIdx].x = nd1;\
	nStack[nIdx].y = nd2;\
	nIdx++;\
}

#define POP_PAIR(nd1, nd2) {\
	nIdx--;\
	nd1 = nStack[nIdx].x;\
	nd2 = nStack[nIdx].y;\
}

#define NEXT(n1, n2) 	POP_PAIR(n1, n2)

__device__ void collide(int left, int *bvhA, aabb *bxsA, tri3f *trisA, vec3f *org_vtxsA, vec3f *vtxsA, uint *masksA, tri3f *tri_edge1, edge4f *edges1,
	int right, int *bvhB, aabb *bxsB, tri3f *trisB, vec3f *org_vtxsB, vec3f *vtxsB, uint *masksB, tri3f *tri_edge2, edge4f *edges2, 
	int idx, int *flag_collide){
	int nIdx = 0;
	if (isLeaf(left, bvhA) && isLeaf(right,bvhB)) {
		intersect(getTriID(left, bvhA), trisA, org_vtxsA, vtxsA, masksA, getTriID(right, bvhB), trisB, org_vtxsB, vtxsB, masksB,
			tri_edge1, tri_edge2, edges1, edges2, idx, flag_collide, nIdx);
		return;
	}
	if (!overlaps(left,bxsA,right,bxsB)) {
		return;
	}

	uint2 nStack[STACK_SIZE];
	//int nIdx = 0;
	if (isLeaf(left, bvhA)) {
		PUSH_PAIR(left, getLeftChild(right, bvhB));
		PUSH_PAIR(left, getLeftChild(right, bvhB));
	}
	else {
		PUSH_PAIR(getLeftChild(left, bvhA), right);
		PUSH_PAIR(getRightChild(left, bvhA), right);
	}
	NEXT(left, right);
	while (1)
	{
		if (isLeaf(left, bvhA) && isLeaf(right, bvhB)) {
			if (overlaps(left, bxsA, right, bxsB))
				intersect(getTriID(left, bvhA), trisA, org_vtxsA, vtxsA, masksA, getTriID(right, bvhB), trisB, org_vtxsB, vtxsB, masksB,
				tri_edge1, tri_edge2, edges1, edges2, idx, flag_collide, nIdx);
		}
		else {
			if (overlaps(left, bxsA, right, bxsB)) {
				if (isLeaf(left, bvhA)) {
					PUSH_PAIR(left, getLeftChild(right, bvhB));
					PUSH_PAIR(left, getRightChild(right, bvhB));
				}
				else {
					PUSH_PAIR(getLeftChild(left, bvhA), right);
					PUSH_PAIR(getRightChild(left, bvhA), right);
				}
			}
		}

		if (EMPTY)
			return;

		NEXT(left, right);
	}
	return;
}

__global__ void kernelBVTT(KernelArray<g_bvh> gbvh_list, KernelArray<g_mesh> gmesh_list, int *flag_collide, int size){
	int idx = blockDim.x*blockIdx.x + threadIdx.x;
	if (idx >= size) return;
	g_bvh &bvh_ptr1 = gbvh_list[idx];
	g_bvh &bvh_ptr2 = gbvh_list[idx+1];

	g_mesh &mesh_ptr1 = gmesh_list[idx];
	g_mesh &mesh_ptr2 = gmesh_list[idx + 1];

	collide(0, bvh_ptr1._bvh_ids, bvh_ptr1._bxs, mesh_ptr1._tris, mesh_ptr1._orig_vtxs, mesh_ptr1._vtxs, mesh_ptr1._masks, 
		mesh_ptr1._tri_edges, mesh_ptr1._edges,
		0, bvh_ptr2._bvh_ids, bvh_ptr2._bxs, mesh_ptr2._tris, mesh_ptr2._orig_vtxs, mesh_ptr2._vtxs, mesh_ptr2._masks, 
		mesh_ptr2._tri_edges, mesh_ptr2._edges, idx, flag_collide);
}

void push2GPU(mesh &mesh1, bvh &bvh1){
	g_mesh mesh_cuda;
	mesh_cuda._num_vtx = mesh1._num_vtx;
	mesh_cuda._num_tri = mesh1._num_tri;
	mesh_cuda._num_edge = mesh1._num_edge;

	cudaMalloc((void**)&mesh_cuda._tris, sizeof(tri3f)*mesh1._num_tri);
	cudaMemcpy(mesh_cuda._tris, mesh1._tris, sizeof(tri3f)*mesh1._num_tri, cudaMemcpyHostToDevice);

	cudaMalloc((void**)&mesh_cuda._tri_edges, sizeof(tri3f)*mesh1._num_tri);
	cudaMemcpy(mesh_cuda._tri_edges, mesh1._tri_edges, sizeof(tri3f)*mesh1._num_tri, cudaMemcpyHostToDevice);

	cudaMalloc((void**)&mesh_cuda._edges, sizeof(edge4f)*mesh1._num_edge);
	cudaMemcpy(mesh_cuda._edges, mesh1._edges, sizeof(edge4f)*mesh1._num_edge, cudaMemcpyHostToDevice);

	cudaMalloc((void**)&mesh_cuda._init_vtxs, sizeof(vec3f)*mesh1._num_tri);
	cudaMemcpy(mesh_cuda._init_vtxs, mesh1._init_vtxs, sizeof(vec3f)*mesh1._num_vtx, cudaMemcpyHostToDevice);

	cudaMalloc((void**)&mesh_cuda._orig_vtxs, sizeof(vec3f)*mesh1._num_vtx);
	cudaMalloc((void**)&mesh_cuda._vtxs, sizeof(vec3f)*mesh1._num_vtx);

	cudaMalloc((void**)&mesh_cuda._fbxs, sizeof(aabb)*mesh1._num_tri);

	cudaMalloc((void**)&mesh_cuda._masks, sizeof(unsigned int)*mesh1._num_tri);
	cudaMemcpy(mesh_cuda._masks, mesh1._mask, sizeof(unsigned int)*mesh1._num_tri, cudaMemcpyHostToDevice);

	mesh_list.push_back(mesh_cuda);


	g_bvh bvh_cuda;

	bvh_cuda._num = mesh1._num_tri * 2 - 1;

	int *ids = new int[bvh_cuda._num];
	for (unsigned int i = 0; i<bvh_cuda._num; i++)
		ids[i] = (bvh1.root() + i)->triID();

	cudaMalloc((void**)&bvh_cuda._bvh_ids, bvh_cuda._num*sizeof(int));
	cudaMemcpy(bvh_cuda._bvh_ids, ids, bvh_cuda._num*sizeof(int), cudaMemcpyHostToDevice);

	cudaMalloc((void**)&bvh_cuda._bxs, bvh_cuda._num*sizeof(aabb));

	bvh_list.push_back(bvh_cuda);

}

__global__ void kernelUpdate(KernelArray<g_mesh> gmesh_list, KernelArray<Result2B> collision_pair, 
	KernelArray<AABB> aabb, KernelArray<uint> aabb_map){
	int idx = blockDim.x*blockIdx.x + threadIdx.x;
	if (idx >= collision_pair._size*2) //return if invalid index 
		return;
	g_mesh &mesh_ptr = gmesh_list[idx];
	Result2B& res = collision_pair[idx/2];
	AABB& rhs = aabb[res.fid(idx%2)];
	char prev = (rhs.cur + 1) % 2;
	for (int i = 0; i < mesh_ptr._num_vtx; i++) {
		mesh_ptr._orig_vtxs[i].x = mesh_ptr._init_vtxs[i].x *rhs.trans[rhs.cur][0] + mesh_ptr._init_vtxs[i].y*rhs.trans[rhs.cur][4] +
			mesh_ptr._init_vtxs[i].z*rhs.trans[rhs.cur][8] + rhs.trans[rhs.cur][12];
		mesh_ptr._orig_vtxs[i].y = mesh_ptr._init_vtxs[i].x*rhs.trans[rhs.cur][1] + mesh_ptr._init_vtxs[i].y*rhs.trans[rhs.cur][5] +
			mesh_ptr._init_vtxs[i].z*rhs.trans[rhs.cur][9] + rhs.trans[rhs.cur][13];
		mesh_ptr._orig_vtxs[i].z = mesh_ptr._init_vtxs[i].x*rhs.trans[rhs.cur][2] + mesh_ptr._init_vtxs[i].y*rhs.trans[rhs.cur][6] +
			mesh_ptr._init_vtxs[i].z*rhs.trans[rhs.cur][10] + rhs.trans[rhs.cur][14];

		mesh_ptr._vtxs[i].x = mesh_ptr._init_vtxs[i].x*rhs.trans[prev][0] + mesh_ptr._init_vtxs[i].y*rhs.trans[prev][4] +
			mesh_ptr._init_vtxs[i].z*rhs.trans[prev][8] + rhs.trans[prev][12];
		mesh_ptr._vtxs[i].y = mesh_ptr._init_vtxs[i].x*rhs.trans[prev][1] + mesh_ptr._init_vtxs[i].y*rhs.trans[prev][5] +
			mesh_ptr._init_vtxs[i].z*rhs.trans[prev][9] + rhs.trans[prev][13];
		mesh_ptr._vtxs[i].z = mesh_ptr._init_vtxs[i].x*rhs.trans[prev][2] + mesh_ptr._init_vtxs[i].y*rhs.trans[prev][6] +
			mesh_ptr._init_vtxs[i].z*rhs.trans[prev][10] + rhs.trans[prev][14];
	}
}
void init(mesh &mesh1, bvh &bvh1){
	cudaMalloc((void**)&g_flag_collide, sizeof(int)*MAX_SIZE);
	for (int i = 0; i < 1000; i++){
		push2GPU(mesh1, bvh1);
		push2GPU(mesh1, bvh1);
	}
}
void free(){
	cudaFree(g_flag_collide);
}
void test(KernelArray<Result2B> collision_pair_arr, KernelArray<AABB> aabb_arr, KernelArray<uint> aabb_map_arr, mesh &mesh1, bvh &bvh1){
	if (collision_pair_arr._size == 0)
		return;
	cudaMemset(g_flag_collide, 0, sizeof(int)*MAX_SIZE);
	kernelUpdate << < 1 + collision_pair_arr._size*2/32, 32 >> >(mesh_list, collision_pair_arr, aabb_arr, aabb_map_arr);
	kernelUpdateBox << < 1 + collision_pair_arr._size * 2 / 32, 32 >> >(mesh_list, collision_pair_arr._size * 2);
	kernelRefit << < 1 + collision_pair_arr._size * 2 / 32, 32 >> >(mesh_list, bvh_list, collision_pair_arr._size * 2);
	kernelBVTT << < 1 + collision_pair_arr._size / 32, 32 >> >(bvh_list, mesh_list, g_flag_collide, collision_pair_arr._size);
}