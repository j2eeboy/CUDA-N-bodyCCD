#pragma once

#include "vec3f.h"
#include "feature.h"
#include "edge.h"
#include "box.h"

#include <set>
#include <vector>

typedef vector<unsigned int> id_list;

class bvh;

class contactInfo {
	unsigned int _vid; // vertex id
	float _t;               // t \in [0, 1]
	vec3f _loc;          // the location

public:
	contactInfo(unsigned int id, float t, vec3f &i) : _vid(id), _t(t), _loc(i) {
		NULL;
	}

	unsigned int vid() { return _vid; }
	vec3f loc() { return _loc; }
	float t() { return _t; }
	void set_t(float t) { _t = t; }
	void set_loc(vec3f &v) { _loc = v; }

	bool operator < (const contactInfo &o) const
	{
		return _vid < o._vid;
	}

};

class mesh {
public:
	unsigned int _num_vtx;
	unsigned int _num_tri;

	tri3f  *_tri_edges;
	unsigned int *_mask;
	//tri3f1 *_tris1;
	
	// used by time integration
	vec3f *_vtxs, *_orig_vtxs, *_init_vtxs;
	vec3f *_vels, *_orig_vels;
	id_list *_vtx_fids;

	vec3f *_nrms, *_texs;

	unsigned int _num_edge;
	edge4f *_edges;
	float *_rest_lengths;
	unsigned int *_eidx;

	void buildEdges(bool isCloth);
	void buildMask();

	BOX *_vbxs, *_fbxs, *_ebxs;
	float _thickness, _friction, _edge_epsilon;
	vec3f *_fcenters;

	vec3f *_avg_vels;
	vec3f *_impls;
	int *_impl_nums;
	set<contactInfo> _contacts;

	bvh *_bvh;


	tri3f *_tris;
	mesh(unsigned int numVtx, unsigned int numTri, tri3f *tris, vec3f *vtxs, vec3f *texs=NULL, bool isCloth=true);
	mesh(char *);
	mesh(){}
	~mesh();

	unsigned int getNbVertices() const { return _num_vtx; }
	unsigned int getNbFaces() const { return _num_tri; }
	unsigned int getNbEdges() const { return _num_edge; }

	bool equalFlag(int id1, int id2){
		if(_tris[id1].flag() == _tris[id2].flag())
			return true;
		return false;
	}

	float calcArea(unsigned int i) const {
		vec3f v1= _vtxs[_tris[i].id0()] - _vtxs[_tris[i].id1()];
		vec3f v2= _vtxs[_tris[i].id1()] - _vtxs[_tris[i].id2()];
		return v1.cross( v2 ).length() * 0.5f;
	}

	unsigned int getEdgeId(unsigned int i, unsigned int j) const {
		return _tri_edges[i].id(j);
	}

	unsigned int getVertexId(unsigned int i, unsigned int j) const {
		return _tris[i].id(j);
	}

	unsigned int getFaceId(unsigned int i) const {
		return i;
	}

	vec3f calcNonUnitNormal(unsigned int i) const {
		vec3f v1= _vtxs[_tris[i].id0()] - _vtxs[_tris[i].id1()];
		vec3f v2= _vtxs[_tris[i].id1()] - _vtxs[_tris[i].id2()];
		return v1.cross( v2 );
	}

	vec3f calcNormal(unsigned int i) const {
	    return calcNonUnitNormal(i).getUnit();
	}

	unsigned int getFNbVertices(unsigned int i) const {
		return 3;
	}

//#################################
	unsigned int getEFaceId(unsigned int eid, int i) const {
		return _edges[eid].fid(i);
	}

	id_list &getVFids(unsigned int vid)
	{
		return _vtx_fids[vid];
	}

	FORCEINLINE bool FCovertex(unsigned int id1, unsigned int id2) {
		for (int i=0; i<3; i++)
			for (int j=0; j<3; j++) {
				if (_tris[id1].id(i) == _tris[id2].id(j))
					return true;
			}

		return false;
	}

	FORCEINLINE void collectFCenter() {
		_fcenters = new vec3f[_num_tri];
		for (unsigned int i=0; i<_num_tri; i++)
			_fcenters[i] = (_vtxs[_tris[i].id0()] + _vtxs[_tris[i].id1()] + _vtxs[_tris[i].id2()])/3.f;
	}

	void clearFCenter() {
		delete [] _fcenters;
	}

	vec3f getFCenter(unsigned int i) {
		return _fcenters[i];
	}

	edge4f getEdge(unsigned int i) {
		return _edges[i];
	}

	BOX getEBox(unsigned int i) {
		return _ebxs[i];
	}

	BOX getVBox(unsigned int i) {
		return _vbxs[i];
	}

	__host__ __device__  BOX getFBox(unsigned int i) {
		return _fbxs[i];
	}

	vec3f getOVtx(unsigned int i) {
		return _orig_vtxs[i];
	}

	vec3f &getVtx(unsigned int i) {
		return _vtxs[i];
	}

	float getEdgeEpsilon() const {
		return _edge_epsilon;
	}

	vec3f getAVel(unsigned int i) {
		return _avg_vels[i];
	}

	float getFriction() const {
		return _friction;
	}

	tri3f getTriangle(unsigned int i) {
		return _tris[i];
	}

	float getThickness() const {
		return _thickness;
	}
//#################################

	vec3f *getOrigVertices() const {
		return _orig_vtxs;
	}

	vec3f *getVertices() const {
		return _vtxs;
	}

	vec3f *getVelocities() const {
		return _vels;
	}

	vec3f getInitVertex(unsigned int i) const {
		return _init_vtxs[i];
	}

	vec3f getVertex(unsigned int i) const {
		return _vtxs[i];
	}

	vec3f getVelocity(unsigned int i) const {
		return _vels[i];
	}

	void setVertex(unsigned int i, vec3f &p) {
		_vtxs[i] = p;
	}

	void setVelocity(unsigned int i, vec3f &v) {
		_vels[i] = v;
	}

	vec3f getTextureVertex(unsigned int i) const {
		return _texs[i];
	}

	void getEVertexId(unsigned int eid, unsigned int ids[]) const {
		memcpy(ids, _edges[eid].vtxs(), 4*sizeof(unsigned int));
	}

	void getEFaceId(unsigned int eid, unsigned int &fA, unsigned int &fB) const {
		fA = _edges[eid].fid0();
		fB = _edges[eid].fid1();
	}

	bool hasTwin(unsigned int eid) const {
		return _edges[eid].fid1() != -1;
	}

	// calc norms, and prepare for display ...
	void updateNrms();

	// really displaying ...
	void display(bool tri, bool pnt, bool edge, int level, bool rigid=false);

	// povray file output
	void povray(char *fname, bool first);

	// obj file output
	//void objfile(char *fname);

	// backup positions and velocities
	void backup() {
		memcpy(_orig_vtxs, _vtxs, sizeof(vec3f)*_num_vtx);
		memcpy(_orig_vels, _vels, sizeof(vec3f)*_num_vtx);
	}

	void restore() {
		memcpy(_vtxs, _orig_vtxs, sizeof(vec3f)*_num_vtx);
		memcpy(_vels, _orig_vels, sizeof(vec3f)*_num_vtx);
	}

	//collision handling on GPU
	bool collisionHandlingGPU(vector<mesh *> &lst, float t, double &);
	bool proximityHandlingGPU(vector<mesh *> &lst, float t, int, bool);
	void proximitySelfGPU(int);
	void proximityOtherGPU(int);

	bool collisionCheckingGPU(vector<mesh *> &lst);
	bool gatherImpulseGPU(float t, bool);

	//collision handling
	bool collisionHandling(vector<mesh *> &lst, float t, double &);
	bool proximityHandling(vector<mesh *> &lst, float t);
	bool ccdHandling(vector<mesh *> &lst, float t);

	bool proximitySelf(float t);
	bool proximityOther(mesh *, float t);
	bool ccdSelf(float t);
	bool ccdOther(mesh *, float t);
	
	bool ccdRigid(mesh *, float t);
	bool proximityRigid(mesh *, float);

	void initImpulse(float t);
	void addImpulse(unsigned int idx, vec3f &i);
	void applyImpulse(float t);

	void addContact(unsigned int idx, float t, vec3f &i);
	void applyContacts();

	void updateBoxes(bool ccd = false); // if ccd == false, only use _orig_vtxs, otherwise, use both _orig_vtxs and _vtxs

	//void dump(FILE *);
	void load(char *);

	//void save(char *);

	bool push2GPU(bool isCloth);
	bool fetchGPU(bool isCloth);
	void update2GPU(bool isCloth);
	void updateFront(mesh *);

	void rotate();
};
