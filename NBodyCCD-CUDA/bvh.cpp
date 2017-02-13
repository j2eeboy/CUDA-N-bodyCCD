#include "bvh.h"
//#include "front.h"

#include <queue>

static bvh_node *s_current = NULL;
static unsigned int *s_idx_buffer;
static mesh *s_mesh = NULL;
static mesh *s_meshA = NULL, *s_meshB = NULL;

static set<pair_info> ee_bank, vf_bank;

/*extern "C" void pushBVH(unsigned int, int *, bool);
extern "C" void pushBVHIdx(int, unsigned int *, bool);
extern "C" void refitBVH(bool);
extern "C" void refitBVH_Serial(bool, int=0);*/

void
clear_pair_banks()
{
	ee_bank.clear();
	vf_bank.clear();
}

void
get_pairs(set<pair_info>* &vfs, set<pair_info>* &ees)
{
	vfs = &vf_bank;
	ees = &ee_bank;
}

void
intersect_ee(mesh *A, unsigned int eA, mesh *B, unsigned int eB)
{
	if (A == B && eA > eB)
		::swap(eA, eB);

	ee_bank.insert(pair_info(A, eA, B, eB));
}

void
intersect_vf(mesh *A, unsigned int fid, mesh *B, unsigned int vid)
{
	vf_bank.insert(pair_info(A, fid, B, vid));
}

void
intersect_ee(mesh *A, unsigned int e1, unsigned int e2, unsigned int f1, unsigned int f2)
{
	if (!A->getEBox(e1).overlaps(A->getEBox(e2)))
		return;

	unsigned int e[2];
	unsigned int f[2];

	if (e1 > e2) {
		e[0] = e1, e[1] = e2;
		f[0] = f1, f[1] = f2;
	} else {
		e[0] = e2, e[1] = e1;
		f[0] = f2, f[1] = f1;
	}

	for (int i=0; i<2; i++)
		for (int j=0; j<2; j++) {
			unsigned int ff1 = A->getEFaceId(e[0], i);
			unsigned int ff2 = A->getEFaceId(e[1], j);

			if (ff1 == -1 || ff2 == -1)
				continue;

			if (!A->FCovertex(ff1, ff2)) {
				if (ff1 == f[0] && ff2 == f[1]) {
					intersect_ee(A, e1, A, e2);
				}else
					return;
			}
		}
}

void
intersect_vf(mesh *A, unsigned int fid1, unsigned int vid2, unsigned int fid2)
{
	if (!A->getFBox(fid1).overlaps(A->getVBox(vid2)))
		return;

	id_list &lst = A->getVFids(vid2);
	for (id_list::iterator it=lst.begin(); it!=lst.end(); it++) {
		unsigned int fid = *it;

		if (!A->FCovertex(fid1, fid)) {
			if (fid == fid2) {
				intersect_vf(A, fid1, A, vid2);
			} else
				return;
		}
	}
}

void test_features(mesh *A, unsigned int tA, mesh *B, unsigned tB)
{
	if (A != B) {
		// 6 VF test
		for (int i=0; i<3; i++) {
			intersect_vf(A, tA, B, B->getVertexId(tB, i));
			intersect_vf(B, tB, A, A->getVertexId(tA, i));
		}

		// 9 EE test
		for (int i=0; i<3; i++)
			for (int j=0; j<3; j++) {
			unsigned int e0 = A->getEdgeId(tA, i);
			unsigned int e1 = B->getEdgeId(tB, j);
			
			intersect_ee(A, e0, B, e1);
		}
	} else {
		// 6 VF test
		for (int i=0; i<3; i++) {
			intersect_vf(A, tA, A->getVertexId(tB, i), tB);
			intersect_vf(A, tB, A->getVertexId(tA, i), tA);
		}

		// 9 EE test
		for (int i=0; i<3; i++)
			for (int j=0; j<3; j++) {
				unsigned int e0 = A->getEdgeId(tA, i);
				unsigned int e1 = A->getEdgeId(tB, j);
			
			intersect_ee(A, e0, e1, tA, tB);
		}
	}
}

bool
collide_leaves(bvh_node *a, bvh_node *b, mesh *ma, mesh *mb)
{
		bool cov = false;

		if (ma == mb)
			cov = ma->FCovertex(a->triID(), b->triID());

		if (cov) return true;
		
		if (!a->box().overlaps(b->box())) return false;

		test_features(ma, a->triID(), mb, b->triID());
		return false;
}

#ifdef NO_FRONT
void
bvh_node::collide(bvh_node *other)
{
	if (isLeaf() && other->isLeaf()) {
		collide_leaves(this, other, s_meshA, s_meshB);
		return;
	}

	if (!_box.overlaps(other->_box)) return;

	if (isLeaf()) {
		collide(other->left());
		collide(other->right());
	} else {
		left()->collide(other);
		right()->collide(other);
	}
}

void
bvh_node::self_collide()
{
	if (isLeaf())
		return;

	left()->self_collide();
	right()->self_collide();
	left()->collide(right());
}
#else
void
bvh_node::collide(bvh_node *other, front_list &f, int level)
{
	if (isLeaf() && other->isLeaf()) {
#ifdef CLOTH
		if (collide_leaves(this, other, s_meshA, s_meshB) == false)
			f.push_back(front_node(this, other));
#endif
#ifdef DP
		if (s_meshA->_tris[this->triID()].flag() != s_meshA->_tris[other->triID()].flag())
		{
			//printf("==================%d %d\n",s_meshA->_tris[this->triID()].flag(),s_meshA->_tris[other->triID()].flag());
			f.push_back(front_node(this, other));
		}
#endif
			

		return;
	}

	if (!_box.overlaps(other->_box) || level >100) {
		//f.push_back(front_node(this, other));
		return;
	}

	if (isLeaf()) {
		collide(other->left(), f, level++);
		collide(other->right(), f, level++);
	} else {
		left()->collide(other, f, level++);
		right()->collide(other, f, level++);
	}
}

void
bvh_node::self_collide(front_list &lst)
{
	if (isLeaf())
		return;

	left()->self_collide(lst);
	right()->self_collide(lst);
	left()->collide(right(), lst);
}
#endif

void
bvh_node::refit()
{
	if (isLeaf()) {
		_box = s_mesh->getFBox(_child);
	} else {
		left()->refit();
		right()->refit();

		_box = left()->_box + right()->_box;
	}
}

void
bvh_node::visualize(int level)
{
	//if (isLeaf())
	//	_box.visualize();
	//else
	//	if ((level > 0)) {
	//		if (level == 1)
	//			_box.visualize();
	//		else {
	//			if (left())
	//				left()->visualize(level-1);
	//			if (right())
	//				right()->visualize(level-1);
	//		}
	//	}
}

void
bvh_node::construct(unsigned int id)
{
	_child = id;
	_box = s_mesh->getFBox(id);
}

void
bvh_node::construct(unsigned int *lst, unsigned int num)
{
	for (unsigned int i=0; i<num; i++)
		_box += s_mesh->getFBox(lst[i]);

	if (num == 1) {
		_child = lst[0];
		return;
	}

	// try to split them
	_child = int(this-s_current);
	s_current += 2;

	if (num == 2) {
		left()->construct(lst[0]);
		right()->construct(lst[1]);
		return;
	}

	aap pln(_box);
	unsigned int left_idx=0, right_idx=num-1;
	for (unsigned int t=0; t<num; t++) {
		int i=lst[left_idx];

		if (pln.inside(s_mesh->getFCenter(i)))
			left_idx++;
		else {// swap it
			unsigned int tmp=lst[left_idx];
			lst[left_idx] = lst[right_idx];
			lst[right_idx--] = tmp;
		}
	}

	int half = num/2;

	if (left_idx == 0 || left_idx == num) {
		left()->construct(lst, half);
		right()->construct(lst+half, num-half);
	} else {
		left()->construct(lst, left_idx);
		right()->construct(lst+left_idx, num-left_idx);
	}
}

void
bvh::construct()
{
	BOX total;
	for (unsigned int i=0; i<_mesh->getNbVertices(); i++)
		total += _mesh->getVtx(i);

	int num = _mesh->getNbFaces();
	_mesh->collectFCenter();

	aap pln(total);
	s_idx_buffer = new unsigned int[num];
	unsigned int left_idx = 0, right_idx = num, tri_idx = 0;

	for (unsigned int i=0; i<_mesh->getNbFaces(); i++) {
		if (pln.inside(_mesh->getFCenter(i)))
			s_idx_buffer[left_idx++] = i;
		else
			s_idx_buffer[--right_idx] = i;
	}

#ifdef VIS_BXS
	_bxs = new BOX[num*2-1];
#endif

	_nodes = new bvh_node[num*2-1];
	_nodes[0]._box = total;
	s_current = _nodes+3;
	s_mesh = _mesh;

	if (num == 1) {
		_nodes[0]._child = 0;
	} else {
		_nodes[0]._child = -1;

		if (left_idx == 0 || left_idx == num)
			left_idx = num/2;

		_nodes[0].left()->construct(s_idx_buffer, left_idx);
		_nodes[0].right()->construct(s_idx_buffer+left_idx, num-left_idx);
	}

	_mesh->clearFCenter();
	delete [] s_idx_buffer;
	s_idx_buffer = NULL;

	if (true) 
	{
	queue<bvh_node *> q;

	// We need to perform a breadth-first traversal to fill the ids

	// the first pass get idx for each node ...
	int *buffer = new int[num*2-1];
	int idx = 0;
	q.push(root());
	while (!q.empty()) {
		bvh_node *node = q.front();
		buffer[node-_nodes] = idx++;
		q.pop();

		if (!node->isLeaf()) {
			q.push(node->left());
			q.push(node->right());
		}
	}

	// the 2nd pass, get right nodes ...
	bvh_node *new_nodes = new bvh_node[num*2-1];
	idx=0;
	q.push(root());
	while (!q.empty()) {
		bvh_node *node = q.front();
		q.pop();

		new_nodes[idx] = *node;
		if (!node->isLeaf()) {
			int loc = node->left()-_nodes;
			new_nodes[idx]._child = idx-buffer[loc];
		}
		idx++;

		if (!node->isLeaf()) {
			q.push(node->left());
			q.push(node->right());
		}
	}

	delete [] buffer;
	delete [] _nodes;
	_nodes = new_nodes;
	}
}

void
bvh::refit()
{
	s_mesh = _mesh;
	root()->refit();
}

void
bvh::push2GPU(bool isCloth)
{
	unsigned int length = _mesh->getNbFaces()*2-1;
	int *ids = new int[length];

	for (unsigned int i=0; i<length; i++)
		ids[i] = (root()+i)->triID();

//	pushBVH(length, ids, isCloth);
	delete [] ids;

	if (isCloth) {// push information for refit
		int max_level = 0;
		root()->getLevel(0, max_level);
		max_level++;

		unsigned int *level_idx = new unsigned int [max_level];
		unsigned int *level_buffer = new unsigned int [max_level];
		for (int i=0; i<max_level; i++)
			level_idx[i] = level_buffer[i] = 0;

		root()->getLevelIdx(0, level_buffer);
		for (int i=1; i<max_level; i++)
			for (int j=0; j<i; j++)
				level_idx[i] += level_buffer[j];

		delete [] level_buffer;
//		pushBVHIdx(max_level, level_idx, isCloth);
		delete [] level_idx;
	}

//	refitBVH_Serial(isCloth);
	
#ifdef MY_DEBUG
/*	g_box a[3];
	
	if (isCloth)
		 (cudaMemcpy(a, clothBVH._bxs, 3*sizeof(g_box), cudaMemcpyDeviceToHost));
	else
		 (cudaMemcpy(a, objBVH._bxs, 3*sizeof(g_box), cudaMemcpyDeviceToHost));
	
	a[0].print();
	a[1].print();
	a[2].print();*/
#endif
}

#ifdef NO_FRONT
void
bvh::collide(bvh *other)
{
	s_meshA = _mesh;
	s_meshB = other->_mesh;

	root()->collide(other->root());
}

void
bvh::self_collide()
{
	s_meshA = s_meshB = _mesh;

	root()->self_collide();
}
#else

void
bvh::collide(bvh *other, front_list &f)
{
	s_meshA = _mesh;
	s_meshB = other->_mesh;
//	f.clear();

	root()->collide(other->root(), f);
}

void
bvh::self_collide(front_list &f)
{
	s_meshA = s_meshB = _mesh;
//	f.clear();

	root()->self_collide(f);
}
#endif
