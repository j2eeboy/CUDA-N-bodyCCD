#pragma once
#include <list>
#include "ccdAPI.h"
//#include "DeformModel.h"
#include<iostream>
#include "cuda_callable.h"
using namespace std;
class EndPoint;
class AABB;
class Cell;
const int ENDPOINT_MIN = 1;
const int ENDPOINT_MAX = 2;
#define M_PI 3.14159265358979323846

inline float RAD2DEG(float x)
{
	return (180.0*(x) / M_PI);
}

class EndPoint
{
public:
	char        minmax;     //whether the instance represents a "min" or a "max" end-point.

	float      val[3];     //the coordinates of the EndPoint.
	EndPoint    *prev[3];   //for maintaining the three linked
	EndPoint    *next[3];   //lists.

	AABB        *aabb;      //back pointer to the parent AABB.
	unsigned int id;
};
struct MyStruct
{
	AABB *pair[2];
	MyStruct *next;
	char level;
	float t[2];
	MyStruct(AABB *a1, AABB *a2, char level1, float t0, float t1){
		pair[0] = a1;
		pair[1] = a2;
		next = NULL;
		level = level1;
		t[0] = t0;
		t[1] = t1;
	}
};
class AABB
{
public:
	unsigned int  id;             // the id of the enclosed object
	//float min[3], max[3];
	EndPoint     *lo;            // the (min_x, min_y, min_z) and
	EndPoint     *hi;            // (max_x, max_y, max_z) corners of
	float radius;
	Cell *cell;

	vec3f position, velocity, orietation, thelta;     //the coordinates of the EndPoint.
	vec3f prev_center, cur_center, orig_center;
	float trans[2][16];
	char cur;//trans flag
	CUDA_CALLABLE_MEMBER AABB(unsigned int id1, float orig[], float pos[], float vel[], float ori[], float thel[],
		float &radius1, Cell *cell_father) :id(id1), position(pos), velocity(vel), orietation(ori), thelta(thel),
		orig_center(orig), radius(radius1)
	{
		cell = cell_father;
		cur = 1;
		float theta_len = orietation.length();
		vec3f theta = orietation;
		theta.normalize();
		Rotated(RAD2DEG(theta_len), theta[0], theta[1], theta[2], trans[1]);
		trans[1][12] = position[0];
		trans[1][13] = position[1];
		trans[1][14] = position[2];
		cur_center.x = orig_center[0] * trans[1][0] + orig_center[1] * trans[1][4] + orig_center[2] * trans[1][8] + trans[1][12];
		cur_center.y = orig_center[0] * trans[1][1] + orig_center[1] * trans[1][5] + orig_center[2] * trans[1][9] + trans[1][13];
		cur_center.z = orig_center[0] * trans[1][2] + orig_center[1] * trans[1][6] + orig_center[2] * trans[1][10] + trans[1][14];

		theta = orietation - thelta;
		theta_len = theta.length();
		theta.normalize();
		Rotated(RAD2DEG(theta_len), theta[0], theta[1], theta[2], trans[0]);
		trans[0][12] = position[0] - velocity[0];
		trans[0][13] = position[1] - velocity[1];
		trans[0][14] = position[2] - velocity[2];
		prev_center.x = orig_center[0] * trans[0][0] + orig_center[1] * trans[0][4] + orig_center[2] * trans[0][8] + trans[0][12];
		prev_center.y = orig_center[0] * trans[0][1] + orig_center[1] * trans[0][5] + orig_center[2] * trans[0][9] + trans[0][13];
		prev_center.z = orig_center[0] * trans[0][2] + orig_center[1] * trans[0][6] + orig_center[2] * trans[0][10] + trans[0][14];

		id = id1;
		lo = new EndPoint;
		lo->id = this->id;	
		lo->aabb = this;
		lo->minmax = ENDPOINT_MIN;
		hi = new EndPoint;
		hi->id = this->id;
		hi->aabb = this;
		hi->minmax = ENDPOINT_MAX;


		update(prev_center, cur_center);
	}


	CUDA_CALLABLE_MEMBER AABB(){
// 		lo = new EndPoint();
// 		hi = new EndPoint();
	}

// 	CUDA_CALLABLE_MEMBER const AABB& operator=(const AABB& rhs){
// 		id = rhs.id;
// 		radius = rhs.radius;
// 		position = rhs.position;
// 		velocity = rhs.velocity;
// 		orietation = rhs.orietation;
// 		thelta = rhs.thelta;
// 		prev_center = rhs.prev_center;
// 		cur_center = rhs.cur_center;
// 		orig_center = rhs.orig_center;
// 		cur = rhs.cur;
// 		*lo = *rhs.lo;
// 		*hi = *rhs.hi;
//  		return rhs;
// 	}


// 	CUDA_CALLABLE_MEMBER AABB(const AABB& rhs)	:
// 		id(rhs.id),
// 		radius(rhs.radius),
// 		position(rhs.position),
// 		velocity(rhs.velocity),
// 		orietation(rhs.orietation),
// 		thelta(rhs.thelta),
// 		prev_center(rhs.prev_center),
// 		cur_center(rhs.cur_center),
// 		orig_center(rhs.orig_center),
// 		cur(rhs.cur)
// 	{
// // 		auto trans_size = sizeof(trans);
// // 		memcpy(trans, rhs.trans, trans_size);
// 		lo = new EndPoint(*(rhs.lo));
// 		hi = new EndPoint(*(rhs.hi));
// 	}


	CUDA_CALLABLE_MEMBER void updateCenter(){
		cur = cur == 1 ? 0 : 1;
		prev_center = cur_center;
		position = position + velocity;
		orietation = orietation + thelta;

		vec3f theta = orietation;
		float theta_len = theta.length();
		theta.normalize();
		Rotated(RAD2DEG(theta_len), theta[0], theta[1], theta[2], trans[cur]);
		trans[cur][12] = position[0];
		trans[cur][13] = position[1];
		trans[cur][14] = position[2];
		cur_center.x = orig_center[0] * trans[cur][0] + orig_center[1] * trans[cur][4] + orig_center[2] * trans[cur][8] + trans[cur][12];
		cur_center.y = orig_center[0] * trans[cur][1] + orig_center[1] * trans[cur][5] + orig_center[2] * trans[cur][9] + trans[cur][13];
		cur_center.z = orig_center[0] * trans[cur][2] + orig_center[1] * trans[cur][6] + orig_center[2] * trans[cur][10] + trans[cur][14];
		update(prev_center, cur_center);
	}
/*	CUDA_CALLABLE_MEMBER void updatePos(vec3f_list &vtxs, DeformModel *mdl){
		char prev = (cur + 1) % 2;
		for (int i = 0; i < mdl->_num_vtx; i++) {
			mdl->_prev_vtxs[i].x = vtxs[i].x*trans[prev][0] + vtxs[i].y*trans[prev][4] + vtxs[i].z*trans[prev][8] + trans[prev][12];
			mdl->_prev_vtxs[i].y = vtxs[i].x*trans[prev][1] + vtxs[i].y*trans[prev][5] + vtxs[i].z*trans[prev][9] + trans[prev][13];
			mdl->_prev_vtxs[i].z = vtxs[i].x*trans[prev][2] + vtxs[i].y*trans[prev][6] + vtxs[i].z*trans[prev][10] + trans[prev][14];

			mdl->_cur_vtxs[i].x = vtxs[i].x*trans[cur][0] + vtxs[i].y*trans[cur][4] + vtxs[i].z*trans[cur][8] + trans[cur][12];
			mdl->_cur_vtxs[i].y = vtxs[i].x*trans[cur][1] + vtxs[i].y*trans[cur][5] + vtxs[i].z*trans[cur][9] + trans[cur][13];
			mdl->_cur_vtxs[i].z = vtxs[i].x*trans[cur][2] + vtxs[i].y*trans[cur][6] + vtxs[i].z*trans[cur][10] + trans[cur][14];
		}
		mdl->UpdateBoxes();
	}*/
	CUDA_CALLABLE_MEMBER void Rotated(float angle, float x, float y, float z, float m[])
	{
		float mag, s, c;
		float xx, yy, zz, xy, yz, zx, xs, ys, zs, one_c;

		s = sin(angle * (M_PI / 180.0));
		c = cos(angle * (M_PI / 180.0));

		mag = sqrt(x*x + y*y + z*z);

		if (mag == 0.0) {
			/* generate an identity matrix and return */
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					m[4 * j + i] = ((i == j) ? 1.0 : 0.0);

			return;
		}

		x /= mag;
		y /= mag;
		z /= mag;

#define M(row,col)  m[col*4+row]

		xx = x * x;
		yy = y * y;
		zz = z * z;
		xy = x * y;
		yz = y * z;
		zx = z * x;
		xs = x * s;
		ys = y * s;
		zs = z * s;
		one_c = 1.0F - c;

		M(0, 0) = (one_c * xx) + c;
		M(0, 1) = (one_c * xy) - zs;
		M(0, 2) = (one_c * zx) + ys;
		M(0, 3) = 0.0F;

		M(1, 0) = (one_c * xy) + zs;
		M(1, 1) = (one_c * yy) + c;
		M(1, 2) = (one_c * yz) - xs;
		M(1, 3) = 0.0F;

		M(2, 0) = (one_c * zx) - ys;
		M(2, 1) = (one_c * yz) + xs;
		M(2, 2) = (one_c * zz) + c;
		M(2, 3) = 0.0F;

		M(3, 0) = 0.0F;
		M(3, 1) = 0.0F;
		M(3, 2) = 0.0F;
		M(3, 3) = 1.0F;

#undef M
	}

	CUDA_CALLABLE_MEMBER AABB(vec3f prev, vec3f cur, const float &radius1) {
		//id = id1;
		lo = new EndPoint;
		lo->aabb = this;
		lo->minmax = ENDPOINT_MIN;
		hi = new EndPoint;
		hi->aabb = this;
		hi->minmax = ENDPOINT_MAX;

		radius = radius1;
		update(prev, cur);
		//printf("%f %f %d===========\n", lo->val[0], hi->val[0], id);
	}
	CUDA_CALLABLE_MEMBER void update(vec3f &prev, vec3f &cur){
		for (int i = 0; i < 3; i++){
			float distance = fabs(cur[i] - prev[i])*0.5;
			lo->val[i] = (cur[i] + prev[i])*0.5 - distance - radius;
			hi->val[i] = (cur[i] + prev[i])*0.5 + distance + radius;
		}
		//printf("%f %f===========\n", lo->val[2], hi->val[2], id);
	}
	CUDA_CALLABLE_MEMBER bool overlaps(const AABB& b) const
	{
		if (lo->val[0] > b.hi->val[0]) return false;
		if (lo->val[1] > b.hi->val[1]) return false;
		if (lo->val[2] > b.hi->val[2]) return false;

		if (hi->val[0] < b.lo->val[0]) return false;
		if (hi->val[1] < b.lo->val[1]) return false;
		if (hi->val[2] < b.lo->val[2]) return false;

		return true;
	}
	CUDA_CALLABLE_MEMBER bool recursiveOverlaps(AABB* b)
	{
		MyStruct *temp = new MyStruct(this, b, 0, 0.f, 1.f);
		while (temp != NULL)
		{
			if (temp->level == 2){
				if (temp->pair[0]->overlaps(*temp->pair[1])){
					while (temp != NULL){
						MyStruct *temp1 = temp;
						temp = temp->next;
						delete temp1;
					}
					return true;
				}
			}
			if (temp->pair[0]->overlaps(*temp->pair[1])){
				char flag = temp->level;
				vec3f lo0(temp->pair[0]->lo->val[0], temp->pair[0]->lo->val[1], temp->pair[0]->lo->val[2]);
				vec3f hi0(temp->pair[0]->hi->val[0], temp->pair[0]->hi->val[1], temp->pair[0]->hi->val[2]);
				//vec3f cur0 = lo0*((temp->t[1]-temp->t[0])/2)+hi0*(temp->t[0]*1.5+temp->t[1]*0.5);
				vec3f cur0 = (lo0 + hi0)*0.5;

				vec3f lo1(temp->pair[1]->lo->val[0], temp->pair[1]->lo->val[1], temp->pair[1]->lo->val[2]);
				vec3f hi1(temp->pair[1]->hi->val[0], temp->pair[1]->hi->val[1], temp->pair[1]->hi->val[2]);
				//vec3f cur1 = lo1*((temp->t[1]-temp->t[0])/2)+hi1*(temp->t[0]*1.5+temp->t[1]*0.5);
				vec3f cur1 = (lo1 + hi1)*0.5;

				AABB *node1 = new AABB(lo0, cur0, temp->pair[0]->radius);
				AABB *node3 = new AABB(cur0, hi0, temp->pair[0]->radius);

				AABB *node2 = new AABB(lo1, cur1, temp->pair[1]->radius);
				AABB *node4 = new AABB(cur1, hi1, temp->pair[1]->radius);
				MyStruct *new_node1 = new MyStruct(node1, node2, flag + 1, temp->t[0], (temp->t[0] + temp->t[1]) / 2);
				MyStruct *new_node2 = new MyStruct(node3, node4, flag + 1, (temp->t[0] + temp->t[1]) / 2, temp->t[1]);
				MyStruct *temp1 = temp;
				temp->next = new_node1;
				new_node1->next = new_node2;
				temp = temp->next;
				delete temp1;
			}
			else{
				MyStruct *temp1 = temp;
				temp = temp->next;
				delete temp1;
			}
		}
		return false;
	}
	CUDA_CALLABLE_MEMBER void deleteEndPoint(EndPoint *head[], EndPoint *cur[]){
		for (int i = 0; i < 1; i++){
			//delete low
			if (lo->prev[i] == NULL){
				lo->next[i]->prev[i] = NULL;
				head[i] = lo->next[i];
			}
			else{
				lo->prev[i]->next[i] = lo->next[i];
				lo->next[i]->prev[i] = lo->prev[i];
			}
			if (hi->next[i] == NULL){
				if (hi->prev[i] == NULL)
					head[i] = cur[i] = NULL;
				else
					hi->prev[i]->next[i] = NULL;
				cur[i] = hi->prev[i];
			}
			else{
				if (hi->prev[i] != NULL)
					hi->prev[i]->next[i] = hi->next[i];
				else{
					head[i] = hi->next[i];
					head[i]->prev[i] = NULL;
				}

				hi->next[i]->prev[i] = hi->prev[i];
			}
		}
		//delete hi;
		//delete lo;
	}
	CUDA_CALLABLE_MEMBER ~AABB(){
#ifndef __CUDACC__
//		if (hi)
		{
			delete hi; hi = NULL;
		}
//		if (lo)
		{
			delete lo; lo = NULL;
		}
#endif
	}
};
class Cell{
public:
    char id[2];
	list<AABB *> aabb_list;
	EndPoint *curr[3], *head[3];
};

#undef M_PI