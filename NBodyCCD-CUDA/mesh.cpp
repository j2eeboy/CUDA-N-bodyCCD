#if defined(WIN32)
#define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif

#include "mesh.h"
#include "bvh.h"
//#include "front.h"

#include <set>
using namespace std;

// for fopen
#pragma warning(disable: 4996)

////////////////////////////////////////////////////////////////////////////////////////////
/*extern "C" void initMesh(bool);
extern "C" void pushMeshX(void *buffer, int size, bool);
extern "C" void pushMeshV(void *buffer, int size, bool);
extern "C" void pushMeshEdges(void *buffer, int size, bool);
extern "C" void pushMeshTris(void *buffer, int size, bool);
extern "C" void pushMeshEs(void *buffer, int size, bool);
extern "C" void pushMeshMask(bool isCloth, void *data);
extern "C" void pushMeshNum(int vtxNum, int triNum, int edgeNum, bool);
extern "C" void pushMeshMisc(bool);
extern "C" void fetchMeshX(void *buffer, int size, bool);
extern "C" void fetchMeshV(void *buffer, int size, bool);
extern "C" void fetchMeshOX(void *buffer, int size);
extern "C" void pushNewFront(void *buffer, int size);
extern "C" void pushSelfFront(void *buffer, int size);
extern "C" void pushSelfFid(void *fidIdx, void *fidAll, int fidNum);
extern "C" void fetchBox(void *bxs, bool); // for visualization
extern "C" void reportMemory();*/
//extern "C" void 

class edgeMerger : public set <edge4f> {
public:
	void add(edge4f e) {
		set<edge4f>::iterator it = find(e);

		if (it == end())
			insert(e);
		else { // need to merge them
			edge4f old = *it;

			//assert(old._fids[1] == -1);
			assert(old.fid1() == -1);

			old.set_fid(1, e.fid0());
			old.set_vtxs(3, e.vtxs(0));
			erase(it);
			insert(old);
		}
	}
};
////yunyyyun
//void mesh::updateBoxes(bool ccd)
//{
//	if (ccd == false) {
//		for (unsigned int i=0; i<_num_vtx; i++) {
//			_vbxs[i] = _orig_vtxs[i];
//			_vbxs[i].enlarge(_thickness);
//		}
//
//		for (unsigned int i=0; i<_num_tri; i++) {
//			tri3f &t = _tris[i];
//
//			_fbxs[i] = BOX(_orig_vtxs[t.id0()], _orig_vtxs[t.id1()]) + _orig_vtxs[t.id2()];
//			_fbxs[i].enlarge(_thickness);
//		}
//
//		for (unsigned int i=0; i<_num_edge; i++) {
//			edge4f &e = _edges[i];
//
//			_ebxs[i] = BOX(_orig_vtxs[e.vid0()], _orig_vtxs[e.vid1()]);
//			_ebxs[i].enlarge(_thickness);
//		}
//	} else {
//		for (unsigned int i=0; i<_num_vtx; i++) {
//			_vbxs[i] += _vtxs[i];
//			_vbxs[i].enlarge(_thickness);
//		}
//
//		for (unsigned int i=0; i<_num_tri; i++) {
//			tri3f &t = _tris[i];
//
//			_fbxs[i] += BOX(_vtxs[t.id0()], _vtxs[t.id1()]) + _vtxs[t.id2()];
//			_fbxs[i].enlarge(_thickness);
//		}
//
//		for (unsigned int i=0; i<_num_edge; i++) {
//			edge4f &e = _edges[i];
//
//			_ebxs[i] += BOX(_vtxs[e.vid0()], _vtxs[e.vid1()]);
//			_ebxs[i].enlarge(_thickness);
//		}
//	}
//}//yunyyyun
void mesh::buildMask()
{
	_mask = new unsigned int[_num_tri];

	bool *vtx_marked = new bool[_num_vtx];
	for (unsigned int i=0; i<_num_vtx; i++) {
		vtx_marked[i] = false;
	}

	bool *edge_marked = new bool[_num_edge];
	for (unsigned int i=0; i<_num_edge; i++)
		edge_marked[i] = false;

	for (unsigned int i=0; i<_num_tri; i++) {
		_mask[i] = 0;

		//		if (i == 15615 || i == 16127)
		//			printf("here\n");

		tri3f *vtx = _tris+i;
		for (int j=0; j<3; j++) {
			unsigned int vid = vtx->id(j);
			if (vtx_marked[vid] == false) {
				//				if (vid == 8108)
				//					printf("here\n");

				_mask[i] |= (0x1 << j);
				vtx_marked[vid] = true;
			}
		}

		tri3f *edge = _tri_edges+i;
		for (int j=0; j<3; j++) {
			unsigned int eid = edge->id(j);
			if (edge_marked[eid] == false) {
				_mask[i] |= (0x8 << j);
				edge_marked[eid] = true;
			}
		}
	}

	delete [] vtx_marked;
	delete [] edge_marked;
}

//#define BULID_NEW

void mesh::buildEdges(bool isCloth)
{
	edgeMerger all;

	for (unsigned int i=0; i<_num_tri; i++) {
		tri3f &t = _tris[i];

		all.add(edge4f(t.id0(), t.id1(), i, t.id2()));
		all.add(edge4f(t.id1(), t.id2(), i, t.id0()));
		all.add(edge4f(t.id2(), t.id0(), i, t.id1()));
	}

	_num_edge = (unsigned int)all.size();
	//printf("%d\n", _num_edge);
	_edges = new edge4f[_num_edge];
	//_rest_lengths = new float[_num_edge];
	//_eidx = new unsigned int[_num_edge*2];
	_tri_edges = new tri3f[_num_tri];


	//if (!isCloth)
		//if (true)
	{
		printf("start building edge map1\n");
		int idx = 0;
		//int idx2 = 0;
		for (set<edge4f>::iterator it=all.begin(); it != all.end(); it++) {
			//(*it).sort();
			edge4f tep=*it;
			if ((tep.fid1()!= -1) &&!(tep._vids[0] < tep._vids[1])){
				std::swap(tep._vids[0], tep._vids[1]);
				std::swap(tep._fids[0], tep._fids[1]);
				std::swap(tep._vtxs[0], tep._vtxs[3]);
				std::swap(tep._vtxs[1], tep._vtxs[2]);
			}
			//all.erase(it);
			//all.insert(tep);

		//	_rest_lengths[idx] = (_vtxs[(*it).vid0()]-_vtxs[(*it).vid1()]).length();
			_edges[idx++] = tep;

			//_eidx[idx2++] = (*it).vid0();
			//_eidx[idx2++] = (*it).vid1();
		}

		std::sort(_edges, _edges+_num_edge);
		printf("start building edge map2\n");

		for (unsigned int i=0; i<_num_tri; i++) {
			tri3f &t=_tris[i];

			edge4f *id0 = std::find(_edges, _edges+_num_edge,
				edge4f(t.id1(), t.id2(), -1, -1));
			edge4f *id1 = std::find(_edges, _edges+_num_edge,
				edge4f(t.id2(), t.id0(), -1, -1));
			edge4f *id2 = std::find(_edges, _edges+_num_edge,
				edge4f(t.id0(), t.id1(), -1, -1));

			assert(id0);
			assert(id1);
			assert(id2);

			_tri_edges[i].set(id0-_edges, id1-_edges, id2-_edges);
			//printf("%d of %d\n", i,_num_tri);
		}
		//printf("start building edge map3\n");

		//if (isCloth) {
		//	FILE *fp = fopen("c:\\temp\\tri.dat", "wb");

		//	fwrite(_edges, sizeof(edge4f), _num_edge, fp);
		//	fwrite(_rest_lengths, sizeof(float), _num_edge, fp);
		//	fwrite(_eidx, sizeof(unsigned int), _num_edge*2, fp);
		//	fwrite(_tri_edges, sizeof(tri3f), _num_tri, fp);

		//	fclose(fp);

		//	printf("Building triangle data done!\n");
		//	//exit(0);
		//}
	}
//	else {
//#ifdef HI_RES
//		// for 1M resolution
//		FILE *fp = fopen("c:\\temp\\tri-301-ro.dat", "rb");
//#else
//		FILE *fp = fopen("c:\\temp\\tri.dat", "rb");
//#endif
//
//		fread(_edges, sizeof(edge4f), _num_edge, fp);
//		fread(_rest_lengths, sizeof(float), _num_edge, fp);
//		fread(_eidx, sizeof(unsigned int), _num_edge*2, fp);
//		fread(_tri_edges, sizeof(tri3f), _num_tri, fp);
//
//		for (unsigned int i=0; i<_num_edge; i++) {
//			edge4f &e=_edges[i];
//			vec3f deltaP = _vtxs[e.vid0()]-_vtxs[e.vid1()];
//			_rest_lengths[i] = deltaP.length();
//		}
//
//		fclose(fp);
//		printf("Building triangle data done!\n");
//	}

	// build _vtx_fids
	//_vtx_fids = new id_list[_num_vtx];
	//for (unsigned int t=0; t<_num_tri; t++) {
	//	for (int i=0; i<3; i++) {
	//		_vtx_fids[_tris[t].id(i)].push_back(t);
	//	}
	//}
}

inline vec3f update(vec3f &v1, vec3f &v2, vec3f &v3)
{
	vec3f s = (v2-v1);
	return s.cross(v3-v1);
}

inline vec3f
update(tri3f &tri, vec3f *vtxs)
{
	vec3f &v1 = vtxs[tri.id0()];
	vec3f &v2 = vtxs[tri.id1()];
	vec3f &v3 = vtxs[tri.id2()];

	return update(v1, v2, v3);
}

void mesh::updateNrms()
{
/*	for (unsigned int i=0; i<_num_vtx; i++)
		_nrms[i] = vec3f::zero();

	for (unsigned int i=0; i<_num_tri; i++) {
		vec3f n = ::update(_tris[i], _vtxs);
		n.normalize();

		_nrms[_tris[i].id0()] += n;
		_nrms[_tris[i].id1()] += n;
		_nrms[_tris[i].id2()] += n;
	}

	for (unsigned int i=0; i<_num_vtx; i++)
		_nrms[i].normalize();*/
}

void mesh::load(char *path)
{/*
	FILE *fp = fopen(path, "rt");
	for (int i=0; i<_num_vtx; i++)
		fscanf(fp, "%g, %g, %g", &_vtxs[i].x, &_vtxs[i].y, &_vtxs[i].z);

	fclose(fp);

	pushMeshX(_vtxs, _num_vtx*sizeof(vec3f), true);*/

}

//void mesh::dump(FILE *fp)
//{
//	/*	FILE *fp = fopen(path, "wt");
//	for (int i=0; i<_num_vtx; i++) {
//	fprintf(fp, "%f, %f, %f\n", _vtxs[i].x, _vtxs[i].y, _vtxs[i].z);
//	}
//	fclose(fp);
//	*/
//
//	fwrite(_vtxs, sizeof(vec3f), _num_vtx,  fp);
//	fwrite(_vels, sizeof(vec3f), _num_vtx,  fp);
//}

void initRedMat(int side)
{
	//GLfloat matAmb[4] =    {1.0, 1.0, 1.0, 1.0};
	//GLfloat matDiff[4] =   {1.0, 0.1, 0.2, 1.0};
	//GLfloat matSpec[4] =   {1.0, 1.0, 1.0, 1.0};
	//glMaterialfv(side, GL_AMBIENT, matAmb);
	//glMaterialfv(side, GL_DIFFUSE, matDiff);
	//glMaterialfv(side, GL_SPECULAR, matSpec);
	//glMaterialf(side, GL_SHININESS, 600.0);
}

void initBlueMat(int side)
{
	//GLfloat matAmb[4] =    {1.0, 1.0, 1.0, 1.0};
	//GLfloat matDiff[4] =   {0.0, 1.0, 1.0, 1.0};
	//GLfloat matSpec[4] =   {1.0, 1.0, 1.0, 1.0};
	//glMaterialfv(side, GL_AMBIENT, matAmb);
	//glMaterialfv(side, GL_DIFFUSE, matDiff);
	//glMaterialfv(side, GL_SPECULAR, matSpec);
	//glMaterialf(side, GL_SHININESS, 60.0);
}

void initYellowMat(int side)
{
	//GLfloat matAmb[4] =    {1.0, 1.0, 1.0, 1.0};
	//GLfloat matDiff[4] =   {1.0, 1.0, 0.0, 1.0};
	//GLfloat matSpec[4] =   {1.0, 1.0, 1.0, 1.0};
	//glMaterialfv(side, GL_AMBIENT, matAmb);
	//glMaterialfv(side, GL_DIFFUSE, matDiff);
	//glMaterialfv(side, GL_SPECULAR, matSpec);
	//glMaterialf(side, GL_SHININESS, 60.0);
}

void mesh::display(bool t, bool p, bool e, int level, bool rigid)
{
//	if (rigid) {
//		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, FALSE);
//		initYellowMat(GL_FRONT);
//	} else {
//		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, TRUE);
//		initRedMat(GL_FRONT);
//		initBlueMat(GL_BACK);
//	}
//
//	glShadeModel(GL_SMOOTH);
//	glEnableClientState( GL_VERTEX_ARRAY );
//	glEnableClientState( GL_NORMAL_ARRAY );
//
//	glVertexPointer(3, GL_FLOAT, sizeof(float)*4, _vtxs);
//	glNormalPointer(GL_FLOAT, sizeof(float)*4, _nrms);
//
//	glEnable(GL_LIGHTING);
//
//	if (t)
//	{
//		//printf("hello=====%d\n",_num_tri);
//		_tris1 = new tri3f1[_num_tri];
//		for (int i=0;i<_num_tri;i++)
//			_tris1[i].set(_tris[i].id(0),_tris[i].id(1),_tris[i].id(2));
//		glDrawElements( GL_TRIANGLES, _num_tri*3, GL_UNSIGNED_INT, _tris1);
//
//	}
//
//	glDisable(GL_LIGHTING);
//
//	if (!rigid && p) {
//		//(14, 560)
//		//(15, 600)
//
//		glPointSize(10);
//		glDrawArrays(GL_POINTS, 0, _num_vtx);
//		/*		glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, &_tris[560]);
//
//		unsigned int pt=14;
//		glDrawElements(GL_POINTS, 1, GL_UNSIGNED_INT, &pt);
//		*/
//	}
//
//	if (e) {
//		//		glDrawElements(GL_LINES, _num_edge*2, GL_UNSIGNED_INT, _eidx);
//		glBegin(GL_LINES);
//		for (unsigned int i=0; i<_num_edge; i++) {
//			edge4f &e=_edges[i];
//			vec3f deltaP = _vtxs[e.vid0()]-_vtxs[e.vid1()];
//
//			float dist = deltaP.length();
//			if (dist > _rest_lengths[i]*1.02f)
//				glColor3f(1.0f, 0.f, 0.f);
//			else if (dist > _rest_lengths[i]*1.01f)
//				glColor3f(1.0f, 1.f, 0.f);
//			else if (dist < _rest_lengths[i]) {
//				glColor3f(0.f, 1.f, 0.f);
//			}
//			else
//				//if (dist < _rest_lengths[i]+0.000001f && dist > _rest_lengths[i]+0.000001f)
//				glColor3f(1.0f, 1.f, 1.f);
//
//			glVertex3fv(_vtxs[e.vid0()].v);
//			glVertex3fv(_vtxs[e.vid1()].v);
//		}
//		glEnd();
//	}
//
//	glDisableClientState( GL_VERTEX_ARRAY );
//	glDisableClientState( GL_NORMAL_ARRAY );
//
//#ifdef VIS_BXS
//	_bvh->visualize(level);
//#endif
}


#pragma warning(disable: 4996)

static char s_path[512];
static bool s_first;

//static void pr_head(FILE *fp)
//{
//	fprintf(fp, "#include \"colors.inc\"\n");
//	fprintf(fp, "#include \"textures.inc\"\n");
//	fprintf(fp, "#include \"setting.inc\"\n");
//}
//
//static void pr_tail(FILE *fp)
//{
//#ifdef FOR_SIG06
//	fprintf(fp, "#include \"tail.inc\"\n");
//#endif
//}

//static void mesh_head(FILE *fp)
//{
//	fprintf(fp, "mesh2 {\n");
//}
//
//static void mesh_tail(FILE *fp, bool color)
//{
//	if (!color) {
//		fprintf(fp, "#include \"material.inc\"\n");
//	} else
//		fprintf(fp, "\tpigment {rgb 1}\n");
//
//	fprintf(fp, "}\n");
//}

//static void vertex_part(unsigned int num, vec3f *vtxs, FILE *fp)
//{
//	fprintf(fp, "\tvertex_vectors {\n");
//	fprintf(fp, "\t\t%d,\n", num);
//	for(unsigned int i=0; i<num; i++) {
//		fprintf(fp, "\t\t<%lf, %lf, %lf>,\n", vtxs[i].x, vtxs[i].y, vtxs[i].z);
//	}
//	fprintf(fp, "\t}\n");
//
//}

//static void normal_part(unsigned int num, vec3f *nrms, FILE *fp)
//{
//	fprintf(fp, "\tnormal_vectors {\n");
//	fprintf(fp, "\t\t%d,\n", num);
//	for(unsigned int i=0; i<num; i++) {
//		fprintf(fp, "\t\t<%lf, %lf, %lf>,\n", nrms[i].x, nrms[i].y, nrms[i].z);
//	}
//	fprintf(fp, "\t}\n");
//
//}

//static void face_part(unsigned int num, tri3f *tris, FILE *fp)
//{
//	if (s_first) {
//		char fname[512];
//		strcpy(fname, s_path);
//		strcat(fname, "\\face_index.inc");
//
//		FILE *fp = fopen(fname, "wt");
//		fprintf(fp, "\tface_indices {\n");
//		fprintf(fp, "\t\t%d,\n", num);
//		for(unsigned int i=0; i<num; i++) {
//			fprintf(fp, "\t\t<%d, %d, %d>,\n", tris[i].id0(), tris[i].id1(), tris[i].id2());
//		}
//		fprintf(fp, "\t}\n");
//		fprintf(fp, "\trotate y*90\n");
//		fprintf(fp, "\trotate z*90\n");
//		fclose(fp);
//	}
//
//	fprintf(fp, "#include \"face_index.inc\"\n");
//}

/*
static void face_part(unsigned int num, tri3f *tris, unsigned int *parts, FILE *fp)
{
if (s_first) {
char fname[512];
strcpy(fname, s_path);
strcat(fname, "\\face_index.inc");

FILE *fp = fopen(fname, "wt");
fprintf(fp, "\tface_indices {\n");

int count = 0;
for (unsigned int i=0; i<num; i++)
if (parts[i] == 3)
count++;

fprintf(fp, "\t\t%d,\n", num-count);
for(unsigned int i=0; i<num; i++) {
if (parts[i] == 3) continue; // skip the bottom plane
fprintf(fp, "\t\t<%d, %d, %d>, %d,\n", tris[i].id0(), tris[i].id1(), tris[i].id2(), parts[i]);
}
fprintf(fp, "\t}\n");
fclose(fp);
}

fprintf(fp, "#include \"texture_list.inc\"\n");
fprintf(fp, "#include \"face_index.inc\"\n");
}
*/

/*
static void texture_part(unsigned int part_num, color3 *part_colors, FILE *fp)
{
if (s_first) {
char fname[512];
strcpy(fname, s_path);
strcat(fname, "\\texture_list.inc");

FILE *fp = fopen(fname, "wt");
fprintf(fp, "\ttexture_list {\n");
fprintf(fp, "\t\t%d,\n", part_num);
for(unsigned int i=0; i<part_num; i++) {
fprintf(fp, "\t\ttexture{pigment{rgb <%lf, %lf, %lf>} finish{ Metal }}\n", part_colors[i]._rgbs[0]/255.f, part_colors[i]._rgbs[1]/255.f, part_colors[i]._rgbs[2]/255.f);
}
fprintf(fp, "\t}\n");
fclose(fp);
}

fprintf(fp, "#include \"texture_list.inc\"\n");
}
*/

void
mesh::povray(char *fname, bool first) //output to povray
{
	//FILE *fp = fopen(fname, "wt");
	//if (fp == NULL) {
	//	if (first) {
	//		printf("Cannot write pov files (at c:\\temp\\pov)\n");
	//	}
	//	return;
	//}
	//bool color = true;

	//s_first = first;
	//strcpy(s_path, fname);
	//char *idx = strrchr(s_path, '\\');
	//*idx = 0;
	//pr_head(fp);

	//mesh_head(fp);
	//vertex_part(_num_vtx, _vtxs, fp);

	///*	if (color) {
	//normal_part(_num_vtx, _nrms, fp);
	////texture_part(_num_parts, _part_colors, fp);
	//face_part(_num_tri, _tris, _parts, fp);
	//} else
	//{
	//face_part(_num_tri, _tris, fp);
	//}*/
	//normal_part(_num_vtx, _nrms, fp);
	//face_part(_num_tri, _tris, fp);

	//mesh_tail(fp, color);

	//pr_tail(fp);

	//fclose(fp);
}

//void
//mesh::objfile(char *fname)
//{
//	FILE *fp = fopen(fname, "wt");
//
//	for (unsigned int i=0; i<_num_vtx; i++) {
//		fprintf(fp, "v %lf %lf %lf\n", _vtxs[i].y, _vtxs[i].z, _vtxs[i].x);
//	}
//
//
//	fprintf(fp, "\n\n");
//
//	for (unsigned int i=0; i<_num_tri; i++) {
//		fprintf(fp, "f %d %d %d\n", _tris[i].id0()+1, _tris[i].id1()+1, _tris[i].id2()+1);
//	}
//
//	fclose(fp);
//}

void mesh::updateBoxes(bool ccd)
{
	if (ccd == false) {
		//for (unsigned int i = 0; i<_num_vtx; i++) {
		//	_vbxs[i] = _orig_vtxs[i];
		//	_vbxs[i].enlarge(_thickness);
		//}
		for (unsigned int i = 0; i<_num_tri; i++) {
			tri3f &t = _tris[i];

			_fbxs[i] = BOX(_orig_vtxs[t.id0()], _orig_vtxs[t.id1()]) + _orig_vtxs[t.id2()];
			//_fbxs[i].enlarge(_thickness);
		}
		//for (unsigned int i = 0; i<_num_edge; i++) {
		//	edge4f &e = _edges[i];

		//	_ebxs[i] = BOX(_orig_vtxs[e.vid0()], _orig_vtxs[e.vid1()]);
		//	_ebxs[i].enlarge(_thickness);
		//}
	}
	else {
		//for (unsigned int i = 0; i<_num_vtx; i++) {
		//	_vbxs[i] += _vtxs[i];
		//	_vbxs[i].enlarge(_thickness);
		//}

		for (unsigned int i = 0; i<_num_tri; i++) {
			tri3f &t = _tris[i];

			_fbxs[i] += BOX(_vtxs[t.id0()], _vtxs[t.id1()]) + _vtxs[t.id2()];
			//_fbxs[i].enlarge(_thickness);
		}

		//for (unsigned int i = 0; i<_num_edge; i++) {
		//	edge4f &e = _edges[i];

		//	_ebxs[i] += BOX(_vtxs[e.vid0()], _vtxs[e.vid1()]);
		//	_ebxs[i].enlarge(_thickness);
		//}
	}
}

mesh::mesh(unsigned int numVtx, unsigned int numTri, tri3f *tris, vec3f *vtxs, vec3f *texs, bool isCloth)
{
	_num_vtx = numVtx;
	_num_tri = numTri;

	_tris = tris;
	//_texs = texs;

	_vtxs = vtxs;
	_orig_vtxs = new vec3f[numVtx];
	_init_vtxs = new vec3f[numVtx];
	memcpy(_init_vtxs, _vtxs, sizeof(vec3f)*numVtx);
	memcpy(_orig_vtxs, _vtxs, sizeof(vec3f)*numVtx);

	//_vels = new vec3f[numVtx];
	//_orig_vels = new vec3f[numVtx];

	//_nrms = new vec3f[numVtx];

	buildEdges(false);
	buildMask();
	//_vbxs = new BOX[_num_vtx];
	_fbxs = new BOX[_num_tri];
	//_ebxs = new BOX[_num_edge];
	//_thickness = float(1e-2);
	//_friction = 5.f;
	//_edge_epsilon = _thickness*_thickness*0.1f;

	//_avg_vels = new vec3f[_num_vtx]; // optimization: do it once ...
	//_impls = new vec3f[_num_vtx];
	//_impl_nums = new int[_num_vtx];
	updateBoxes(false);
	//_bvh = new bvh(this);
}

mesh::~mesh()
{
	delete [] _tris;
	delete [] _vtxs;
	//if (_texs)
		//delete [] _texs;

	//if (_tri_edges)
	delete [] _tri_edges;

	delete [] _orig_vtxs;

	//if (_vtx_fids)
		//delete [] _vtx_fids;

	//delete [] _nrms;
	delete [] _edges;
	//delete [] _rest_lengths;
	//delete [] _eidx;

	delete [] _fbxs;
	//delete [] _vbxs;
	//delete [] _ebxs;

	//delete [] _avg_vels;
	//delete [] _impls;
	//delete [] _impl_nums;

	//delete _bvh;
	delete []_mask;
}

mesh::mesh(char *path)
{
	FILE *fp = fopen(path, "rb");

	fread(&_num_vtx, sizeof(unsigned int), 1, fp);
	fread(&_num_tri, sizeof(unsigned int), 1, fp);
	fread(&_num_edge, sizeof(unsigned int), 1, fp);

	_tris = new tri3f[_num_tri];
	//_tris1 = new tri3f1[_num_tri];
	fread(_tris, sizeof(tri3f), _num_tri, fp);
	_tri_edges = new tri3f[_num_tri];
	fread(_tri_edges, sizeof(tri3f), _num_tri, fp);
	_vtxs = new vec3f[_num_vtx];
	fread(_vtxs, sizeof(vec3f), _num_vtx, fp);

	_orig_vtxs = new vec3f[_num_vtx];
	_init_vtxs = new vec3f[_num_vtx];

	memcpy(_init_vtxs, _vtxs, sizeof(vec3f)*_num_vtx);
	memcpy(_orig_vtxs, _vtxs, sizeof(vec3f)*_num_vtx);

	_vels = new vec3f[_num_vtx];
	_orig_vels = new vec3f[_num_vtx];
	_nrms = new vec3f[_num_vtx];

	_edges = new edge4f[_num_edge];
	_rest_lengths = new float[_num_edge];
	_eidx = new unsigned int[_num_edge*2];
	fread(_edges, sizeof(edge4f), _num_edge, fp);
	fread(_rest_lengths, sizeof(float), _num_edge, fp);
	fread(_eidx, sizeof(unsigned int), _num_edge*2, fp);

	// build _vtx_fids
	_vtx_fids = new id_list[_num_vtx];
	for (unsigned int t=0; t<_num_tri; t++) {
		for (int i=0; i<3; i++) {
			_vtx_fids[_tris[t].id(i)].push_back(t);
		}
	}

	bool have_tex;
	fread(&have_tex, sizeof(bool), 1, fp);
	if (have_tex) {
		_texs = new vec3f[_num_vtx];
		fread(_texs, sizeof(vec3f), _num_vtx, fp);
	}

	_vbxs = new BOX[_num_vtx];
	_fbxs = new BOX[_num_tri];
	_ebxs = new BOX[_num_edge];
	fread(_vbxs, sizeof(BOX), _num_vtx, fp);
	fread(_fbxs, sizeof(BOX), _num_tri, fp);
	fread(_ebxs, sizeof(BOX), _num_edge, fp);

	_mask = new unsigned int[_num_tri];
	fread(_mask, sizeof(unsigned int), _num_tri, fp);

	_thickness = float(1e-2);
	_friction = 5.f;
	_edge_epsilon = _thickness*_thickness*0.1f;

	_avg_vels = new vec3f[_num_vtx]; // optimization: do it once ...
	_impls = new vec3f[_num_vtx];
	_impl_nums = new int[_num_vtx];

	_bvh = new bvh(this);

	fclose(fp);
}

//void mesh::save(char *path)
//{
//	FILE *fp = fopen(path, "wb");
//	fwrite(&_num_vtx, sizeof(unsigned int), 1, fp);
//	fwrite(&_num_tri, sizeof(unsigned int), 1, fp);
//	fwrite(&_num_edge, sizeof(unsigned int), 1, fp);
//
//	fwrite(_tris, sizeof(tri3f), _num_tri,  fp);
//	fwrite(_tri_edges, sizeof(tri3f), _num_tri, fp);
//	fwrite(_vtxs, sizeof(vec3f), _num_vtx, fp);
//
//	fwrite(_edges, sizeof(edge4f), _num_edge, fp);
//	fwrite(_rest_lengths, sizeof(float), _num_edge, fp);
//	fwrite(_eidx, sizeof(unsigned int), _num_edge*2, fp);
//
//	bool have_tex = (_texs != NULL);
//	fwrite(&have_tex, sizeof(bool), 1, fp);
//	if (have_tex)
//		fwrite(_texs, sizeof(vec3f), _num_vtx, fp);
//
//	fwrite(_vbxs, sizeof(BOX), _num_vtx, fp);
//	fwrite(_fbxs, sizeof(BOX), _num_tri, fp);
//	fwrite(_ebxs, sizeof(BOX), _num_edge, fp);
//
//	fwrite(_mask, sizeof(unsigned int), _num_tri, fp);
//	fclose(fp);
//}

void
mesh::update2GPU(bool isCloth)
{
//	pushMeshX(_vtxs, _num_vtx*sizeof(vec3f), isCloth);
//	pushMeshV(_vels, _num_vtx*sizeof(vec3f), isCloth);
}

void
mesh::updateFront(mesh *other)
{
/*	updateBoxes(false);
	_bvh->refit();

	front_list front;
	_bvh->collide(other->_bvh, front);

	unsigned int *frontA = new unsigned int [front.size()*4];
	int idx = 0;

	for (int i=0; i<front.size(); i++) {
		frontA[idx++] = front[i]._left - _bvh->_nodes;
		frontA[idx++] = front[i]._right - other->_bvh->_nodes;
		frontA[idx++] = 0;
		frontA[idx++] = 0;
	}
	printf("We are pushing new front (%ld)\n", front.size());
	reportMemory();
	pushNewFront(frontA, front.size());
	delete [] frontA;*/
}

bool
mesh::push2GPU(bool isCloth)
{
/*	initMesh(isCloth);
	reportMemory();
	pushMeshNum(_num_vtx, _num_tri, _num_edge, isCloth);
	pushMeshX(_vtxs, _num_vtx*sizeof(vec3f), isCloth);
	pushMeshV(_vels, _num_vtx*sizeof(vec3f), isCloth);
	pushMeshEdges(_edges, _num_edge*sizeof(edge4f), isCloth);
	pushMeshTris(_tris, _num_tri*sizeof(tri3f), isCloth);
	pushMeshEs(_tri_edges, _num_tri*sizeof(tri3f), isCloth);
	pushMeshMask(isCloth, _mask);
	pushMeshMisc(isCloth);
	reportMemory();

	_bvh->push2GPU(isCloth);
	reportMemory();

	if (isCloth) {
		// front for self-collisions
		front_list selfFront;
		_bvh->self_collide(selfFront);
		unsigned int *frontA = new unsigned int [selfFront.size()*4];
		int idx = 0;
		for (int i=0; i<selfFront.size(); i++) {
			frontA[idx++] = selfFront[i]._left-_bvh->_nodes;
			frontA[idx++] = selfFront[i]._right-_bvh->_nodes;
			frontA[idx++] = 0;
			frontA[idx++] = 0;
		}
		printf("We are pushing selffront (%ld)\n", selfFront.size());
		reportMemory();
		pushSelfFront(frontA, selfFront.size());
		delete [] frontA;

		// fid for vf/ee tests
		// build vtx_fids
		id_list *vtx_fids = new id_list[_num_vtx];
		for (unsigned t = 0; t < _num_tri; t++)
			for (int i=0; i<3; i++) {
				unsigned int vid = _tris[t].id(i);
				vtx_fids[vid].push_back(t);
			}

			unsigned int *fidIdx = new unsigned int[_num_vtx];
			unsigned int	fidNum = 0;
			for (unsigned int i=0; i<_num_vtx; i++) {
				fidIdx[i] = fidNum;
				fidNum += vtx_fids[i].size();
			}

			unsigned int *fidAll = new unsigned int[fidNum];
			for (unsigned int i=0; i<_num_vtx; i++) {
				unsigned int start = fidIdx[i];
				for (unsigned int j=0; j<vtx_fids[i].size(); j++)
					fidAll[start+j] = vtx_fids[i][j];
			}

			pushSelfFid(fidIdx, fidAll, fidNum);
			reportMemory();

			delete [] vtx_fids;
			delete [] fidAll;
			delete [] fidIdx;
	}*/

	return true;
}

#include <iostream>

bool
mesh::fetchGPU(bool isCloth)
{
//	fetchMeshX(_vtxs, _num_vtx*sizeof(vec3f), isCloth);
//	fetchMeshV(_vels, _num_vtx*sizeof(vec3f), isCloth);

	return true;
}

/*#include "mat3f.h"

extern float xmax, zmax;

static matrix3f trf = matrix3f::rotation(vec3f(0, 0, 1), (M_PI/180.f)/60.f);
static vec3f shift_vec(xmax*0.5, zmax*0.5, 0);

inline vec3f rotateVertex(vec3f &src)
{
	return (src - shift_vec) * trf + shift_vec;
}

void mesh::rotate()
{
	for (int i=0; i<_num_vtx-4; i++) {
		_orig_vtxs[i] = _vtxs[i];
		_vtxs[i] = rotateVertex(_vtxs[i]);
	}
}*/