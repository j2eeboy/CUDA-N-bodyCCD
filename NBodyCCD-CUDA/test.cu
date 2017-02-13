#include <iostream>
#include "ccdAPI.h"
#include "broad_phase_culling.h"
#include <cuda_profiler_api.h>
#include <cudaProfiler.h>
#include <fstream>
#include <map>
#include "mesh.h"
#include "bvh.h"

#include "Transporter.h"
#include "myfeature.h"
#include "result2b.h"

char DATA_PATH[512] = "ball/cloth_ball";
float world[] = { 2000, 2000, 2000 };
int dimention = 1;
int number_cell = 512;
int TOTAL_FRAMES = 1;
float centerPoint[3], radius = 0.f;
float width[3];

extern void BuildSession(char *fname, unsigned int num_frame, float ply_scale,
	unsigned int &vtx_num, vec3f_list &vtxs, tri_list &tris);
extern void init(mesh &mesh1, bvh &bvh1);
extern void free();


void entityUpdate(list<Cell *> &cell_list){
	for (list<Cell *>::iterator it = cell_list.begin(); it != cell_list.end(); it++)
		for (list<AABB *>::iterator it_aabb = (*it)->aabb_list.begin(); it_aabb != (*it)->aabb_list.end(); it_aabb++){
		    (*it_aabb)->updateCenter();
		}
}

int main(int argc, char *argv[]) {
	if (argc < 2)
	{
		std::cout << "Usage :" << argv[0] << " obj_num frame_num" << std::endl;
		std::cout << "Running with default " << number_cell << "objs and " << TOTAL_FRAMES << " frames" << std::endl;
	}
	else      
	{
		number_cell = atoi(argv[1]);
		TOTAL_FRAMES = atoi(argv[2]);
	}

	unsigned int vtx_num;
	vec3f_list vtxs;
	tri_list tris;
	float distance;
	list<Cell *> cell_list; //only one cell
	list<Result2B> result_list;

	width[0] = world[0];
	width[1] = world[0];
	width[2] = world[1];

	BuildSession(DATA_PATH, 1, 1.f, vtx_num, vtxs, tris);

	//InitDeformModel
	vec3f *vtxs1 = new vec3f[vtxs.size()];
	tri3f *tris1 = new tri3f[tris.size()];
	for (int i = 0; i < vtxs.size(); i++)
		vtxs1[i] = vtxs[i];
	for (int i = 0; i < tris.size(); i++)
		tris1[i] = tris[i];
	mesh mesh1(vtxs.size(), tris.size(), tris1, vtxs1);
	bvh bvh1(&mesh1);
	init(mesh1, bvh1);

	for (int j = 0; j < tris.size(); ++j) {
		centerPoint[0] = vtxs[tris[j].id0()][0] + vtxs[tris[j].id1()][0] + vtxs[tris[j].id2()][0];
		centerPoint[1] = vtxs[tris[j].id0()][1] + vtxs[tris[j].id1()][1] + vtxs[tris[j].id2()][1];
		centerPoint[2] = vtxs[tris[j].id0()][2] + vtxs[tris[j].id1()][2] + vtxs[tris[j].id2()][2];
	}
	centerPoint[0] = centerPoint[0] / (3 * tris.size());
	centerPoint[1] = centerPoint[1] / (3 * tris.size());
	centerPoint[2] = centerPoint[2] / (3 * tris.size());

	vec3f d;
	for (int k = 0; k < vtxs.size(); ++k) {
		d = vtxs[k] - vec3f(centerPoint[0], centerPoint[1], centerPoint[2]);
		distance = d.length();
		if (distance > radius) {
			radius = distance;
		}
	}

	srand(30000);
	for (int i = 0; i < dimention; i++){
		for (int j = 0; j < dimention; j++){
			Cell *temp = new Cell;
			temp->id[0] = 0;
			temp->id[1] = i*dimention + j;
			for (int k = 0; k < number_cell; k++){
				float start1 = width[1] * j;
				float start2 = width[2] * i;
				float center[] = { start1 + width[0] * ((float)rand()) / RAND_MAX, start2 + width[1] * ((float)rand()) / RAND_MAX, 
					width[2] * ((float)rand()) / RAND_MAX };
				float pos[] = { center[0] - centerPoint[0], center[1] - centerPoint[1], center[2] - centerPoint[2] };
				float vel[] = { ((float)rand()) / RAND_MAX*width[1] * 0.001f, 
					((float)rand()) / RAND_MAX*width[1] * 0.001f, ((float)rand()) / RAND_MAX*width[1] * 0.001f };
				float ori[] = { ((float)rand()) / RAND_MAX, ((float)rand()) / RAND_MAX, ((float)rand()) / RAND_MAX };
				float thelta[] = {0.0f,0.0f,0.0f};
				temp->aabb_list.push_back(new AABB(i *dimention*number_cell + j * number_cell + k, 
					centerPoint, pos, vel, ori, thelta, radius, temp));
			}
			cell_list.push_back(temp);
		}
	}


	cudaBroadPhaseCullingSession* cullingSession;	//Make it an array if we have more than 1 cell;
	for (int num_frame = 0; num_frame < TOTAL_FRAMES; num_frame++){
		entityUpdate(cell_list);
		cout << "In frame " << num_frame << ":" << endl;
		result_list.clear();
		for (list<Cell *>::iterator it = cell_list.begin(); it != cell_list.end(); it++) {
			if ((**it).aabb_list.size() == 0) continue;
			cullingSession = new cudaBroadPhaseCullingSession((**it).aabb_list);
			cullingSession->updateAABB((**it).aabb_list);
			result_list=cullingSession->BroadPhaseCulling(mesh1,bvh1);			

			delete cullingSession;
		}

		std::cout << "result list after init() " << num_frame << ":" << std::endl;
		for (list<Result2B>::iterator it = result_list.begin(); it != result_list.end();it++)
		{
			cout << (*it).fid(0) << ',' << (*it).fid(1) << endl;
		}
		cout << "-------------" << endl;

	}

	free();
	return 0;
}
