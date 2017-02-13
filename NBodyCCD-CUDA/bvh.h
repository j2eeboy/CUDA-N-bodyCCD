#pragma once
#include "forceline.h"
#include "mesh.h"
#include <cuda_runtime.h>

class bvh;
class front_list;

class aap {
public:
	char _xyz;
	float _p;

	FORCEINLINE aap(const BOX &total) {
		vec3f center = total.center();
		char xyz = 2;

		if (total.width() >= total.height() && total.width() >= total.depth()) {
			xyz = 0;
		} else
		if (total.height() >= total.width() && total.height() >= total.depth()) {
			xyz = 1;
		}

		_xyz = xyz;
		_p = center[xyz];
	}

	FORCEINLINE bool inside(const vec3f &mid) const {
		return mid[_xyz]>_p;
	}
};

class pair_info {
	mesh *_A, *_B;
	unsigned int _a, _b;

public:
	pair_info(mesh *A, unsigned int a, mesh *B, unsigned int b) : _A(A), _B(B), _a(a), _b(b) {
		NULL;
	}

	void retrieve(mesh *&A, unsigned int &a, mesh *&B, unsigned int &b) {
		A = _A, B = _B;
		a = _a, b = _b;
	}

	bool operator < (const pair_info &other) const {
		if (_A != other._A)
			return _A < other._A;

		if (_B != other._B)
			return _B < other._B;

		if (_a != other._a)
			return _a < other._a;
		
		return _b < other._b;
	}

	bool operator == (const pair_info &other) const {
		return (_A == other._A && _B == other._B && _a == other._a && _b == other._b);
			//||(_A == other._B && _B == other._A && _a == other._b && _b == other._a);
	}
};

void clear_pair_banks();
void get_pairs(set<pair_info> *&vfs, set<pair_info> *&ees);
void intersect_vf(mesh *A, unsigned int fid, mesh *B, unsigned int vid);
void intersect_ee(mesh *A, unsigned int eA, mesh *B, unsigned int eB);

class bvh_node {
public:
	BOX _box;
	int _child; // >=0 leaf with tri_id, <0 left & right


	bvh_node() {
		_child = 0;
	}

	~bvh_node() {
		NULL;
	}

	void construct(unsigned int id);
	void construct(unsigned int *lst, unsigned int num);

#ifdef NO_FRONT
	void collide(bvh_node *);
	void self_collide();
#else
	void collide(bvh_node *, front_list &, int level=0);
	void self_collide(front_list &);
#endif

	void sprouting(bvh_node *, front_list &, mesh *, mesh *);
	void visualize(int level);
	void refit();
	__host__ __device__ void refit(mesh &mesh_cuda);

	FORCEINLINE __device__ __host__ BOX &box() { return _box; }
	FORCEINLINE __device__ __host__  bvh_node *left() { return this - _child; }
	FORCEINLINE __device__ __host__  bvh_node *right() { return this - _child + 1; }
	FORCEINLINE int triID() { return _child; }
	FORCEINLINE __device__ __host__ int isLeaf() { return _child >= 0; }

	FORCEINLINE void getLevel(int current, int &max_level) {
		if (current > max_level)
			max_level = current;

		if (isLeaf()) return;
		left()->getLevel(current+1, max_level);
		right()->getLevel(current+1, max_level);
	}

	FORCEINLINE void getLevelIdx(int current, unsigned int *idx) {
		idx[current]++;

		if (isLeaf()) return;
		left()->getLevelIdx(current+1, idx);
		right()->getLevelIdx(current+1, idx);
	}

friend class bvh;
};

bool collide_leaves(bvh_node *, bvh_node *b, mesh *ma, mesh *mb);

// a float bvh
class bvh {

#ifdef VIS_BXS
	BOX *_bxs; // for visualization
#endif

	void construct();

public:
	mesh *_mesh;
	bvh_node *_nodes;
	bvh(mesh *m) {
		_mesh = m;
		_nodes = NULL;
#ifdef VIS_BXS
		_bxs = NULL;
#endif
		construct();
	}
	bvh(){
		_mesh = NULL;
		_nodes = NULL;
	}
	bvh(int i){
		_nodes = new bvh_node[i];
	}

	~bvh() {
		if (_nodes)
			delete [] _nodes;

#ifdef VIS_BXS
		if (_bxs)
			delete [] _bxs;
#endif
	}

	void visualize(int level)  {
#ifdef VIS_BXS
		//root()->visualize(level);
		if (level < 2) return;

		int length1 = MIN(pow(2.f, level-1), _mesh->getNbFaces()*2-1);
		int length2 = MIN(pow(2.f, level-2), _mesh->getNbFaces()*2-1);
		for (int i=length2+1; i<length1; i++)
			_bxs[i].visualize();
#endif
	}

	void refit();
	__device__ __host__ void refit(mesh &mesh_cuda);

#ifdef NO_FRONT
	void collide(bvh *);
	void self_collide();
#else
	void collide(bvh *, front_list &);
	void self_collide(front_list &);
#endif

	__host__ __device__ BOX box() {
		return root()->_box;
	}

	__host__ __device__ bvh_node *root() { return _nodes; }

	void push2GPU(bool);

friend class mesh;
friend class bvh_node;
};