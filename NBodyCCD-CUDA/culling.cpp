#include"myfeature.h"
#include"result2b.h"
#include"stdio.h"
#include <cuda_runtime_api.h>
#include "mesh.h"
#include "bvh.h"
#include <thrust/device_vector.h>

extern vector<char> leave_list;
extern int thread;
/*static DeformModel *mdl1 = NULL;
static DeformModel *mdl2 = NULL;*/
//bvh bvh1_cuda, bvh2_cuda;

//extern void test(bvh &bvh1,bvh &bvh2,mesh &mesh_cuda);

extern void push2GPU(mesh &, bvh &);

void InitDeformModel(vec3f_list &vtxs, tri_list &tris){
	vec3f *vtxs1 = new vec3f[vtxs.size()];
	tri3f *tris1 = new tri3f[tris.size()];
	for (int i = 0; i < vtxs.size(); i++)
		vtxs1[i] = vtxs[i];
	for (int i = 0; i < tris.size(); i++)
		tris1[i] = tris[i];
	mesh mesh1(vtxs.size(), tris.size(), tris1, vtxs1, NULL);
	bvh bvh1(&mesh1);
	push2GPU(mesh1, bvh1);
	push2GPU(mesh1, bvh1);
	push2GPU(mesh1, bvh1);
	push2GPU(mesh1, bvh1);
	//bvh1.refit();
	//bvh bvh2(bvh1._mesh->getNbFaces() * 2 - 1);
	//bvh2.refit();

	//cudaMalloc((void**)&bvh1_cuda._nodes, sizeof(bvh_node)*bvh1._mesh->getNbFaces() * 2 - 1);
	//cudaMemcpy(bvh1_cuda._nodes, bvh1._nodes, sizeof(bvh_node)*bvh1._mesh->getNbFaces() * 2 - 1, cudaMemcpyHostToDevice);
	//cudaMalloc((void**)&bvh2_cuda._nodes, sizeof(bvh_node)*bvh1._mesh->getNbFaces() * 2 - 1);
	//cudaMemcpy(bvh2_cuda._nodes, bvh1._nodes, sizeof(bvh_node)*bvh1._mesh->getNbFaces() * 2 - 1, cudaMemcpyHostToDevice);
	//mesh mesh_cuda;
	//cudaMalloc((void**)&mesh_cuda._fbxs, sizeof(aabb)*mesh1._num_tri);
	//cudaMemcpy(mesh_cuda._fbxs, mesh1._fbxs, sizeof(aabb)*mesh1._num_tri, cudaMemcpyHostToDevice);
	
	//cudaMemcpy(bvh2._nodes, bvh1_cuda._nodes, sizeof(bvh_node)*bvh1._mesh->getNbFaces() * 2 - 1, cudaMemcpyDeviceToHost);
	//for (int i = 0; i < bvh1._mesh->getNbFaces() * 2 - 1; i++)
	//	cout << bvh2._nodes[i]._box._max.x << " " << bvh1._nodes[i]._box._max.x << endl;
	//bvh_list.push_back(bvh1_cuda);
	//bvh_list.push_back(bvh2_cuda);
	//test(bvh1_cuda,bvh2_cuda,mesh_cuda);
	//bvh bvh1_cuda(&mesh1);
	//cudaMalloc((void**)&bvh1_cuda._nodes, sizeof(bvh_node));
	//bvh_node *nodes;
	//cudaMalloc((void**)&nodes, sizeof(bvh_node));
	
	//bvh1.collide(bvh2);
	//bvh1.refit();
	//mdl1->BuildBVH(true);
	//mdl2->BuildBVH(true);
	//DeformModel *mdl3;
	//cudaMalloc((void**)&mdl3, sizeof(DeformModel));
	//cudaMemcpy(mdl3, mdl1, sizeof(DeformModel), cudaMemcpyHostToDevice);
}
//void clear(list<AABB*> &aabb_list, EndPoint *head[], EndPoint *curr[]){
//	for (int coord = 0; coord < 1; coord++){
//		head[coord] = curr[coord] = NULL;
//	}
//	for (list<AABB*>::iterator it = aabb_list.begin(); it != aabb_list.end(); it++) {
//		for (int coord = 0; coord < 1; coord++){
//			(*it)->lo->prev[coord] = NULL;
//			(*it)->lo->next[coord] = NULL;
//			(*it)->hi->prev[coord] = NULL;
//			(*it)->hi->next[coord] = NULL;
//		}
//	}
//}

// void init(list<AABB*> &aabb_list, list<Result2B> &result, EndPoint *head[], EndPoint *curr[]){
// 	//EndPoint *curr[3], *head[3];
// 	int flag = 0;
// 	for (list<AABB*>::iterator it = aabb_list.begin(); it != aabb_list.end(); it++) {
// 		for (int coord = 0; coord < 1; coord++){
// 			if (flag == 0){
// 				(*it)->lo->prev[coord] = NULL;
// 				(*it)->lo->next[coord] = (*it)->hi;
// 				(*it)->hi->prev[coord] = (*it)->lo;
// 				(*it)->hi->next[coord] = NULL;
// 				curr[coord] = (*it)->hi;
// 				head[coord] = (*it)->lo;
// 
// 				continue;
// 			}
// 			//min
// 			EndPoint *temp = curr[coord];
// 			while (temp != NULL && (*it)->lo->val[coord] < temp->val[coord]){//flag>0
// 				//printf("hello %d %f %f\n", coord, (*it)->lo->val[coord],temp->val[coord]);
// 				if (temp->minmax == MAX)
// 					if (temp->aabb->overlaps(**it)){
// 						result.push_back(Result2B((*it)->cell, (*it)->id, temp->aabb->id));
// 					}
// 				temp = temp->prev[coord];
// 			}
// 			if (temp == NULL){
// 				(*it)->lo->next[coord] = head[coord];
// 				(*it)->lo->prev[coord] = NULL;
// 				head[coord]->prev[coord] = (*it)->lo;
// 				head[coord] = (*it)->lo;
// 			}
// 			else{
// 				//printf("hello %d  %f=======\n", coord, (*it)->lo->val[coord]);
// 				if (temp != curr[coord]){
// 					EndPoint *temp1 = temp->next[coord];
// 					temp->next[coord] = (*it)->lo;
// 					(*it)->lo->next[coord] = temp1;
// 					temp1->prev[coord] = (*it)->lo;
// 					(*it)->lo->prev[coord] = temp;
// 
// 				}
// 				else{
// 					curr[coord] = (*it)->lo;
// 					temp->next[coord] = (*it)->lo;
// 					(*it)->lo->next[coord] = NULL;
// 					(*it)->lo->prev[coord] = temp;
// 					//printf("hello %d  %f=======\n", coord, curr[coord]->val[coord]);
// 				}
// 
// 			}
// 
// 			////max
// 			temp = curr[coord];
// 			//printf("hello %d %f %f=======\n", coord, temp->val[coord], (*it)->hi->val[coord]);
// 			while ((*it)->hi->val[coord] < temp->val[coord]){//flag>0
// 				temp = temp->prev[coord];
// 			}
// 
// 			if (temp != curr[coord]){
// 				EndPoint *temp1 = temp->next[coord];
// 				temp->next[coord] = (*it)->hi;
// 				(*it)->hi->next[coord] = temp1;
// 				temp1->prev[coord] = (*it)->hi;
// 				(*it)->hi->prev[coord] = temp;
// 			}
// 			else{
// 				temp->next[coord] = (*it)->hi;
// 				(*it)->hi->next[coord] = NULL;
// 				(*it)->hi->prev[coord] = temp;
// 				curr[coord] = (*it)->hi;
// 			}
// 		}
// 		flag++;
// 	}
// }
//void update(list<AABB*> &aabb_list, list<Result2B> &result, EndPoint *head[], EndPoint *curr[]){
//	//EndPoint *curr[3], *head[3];
//	for (int coord = 0; coord < 3; coord++){
//		EndPoint *temp = head[coord]->next[coord];
//		while (temp != NULL)
//		{
//			EndPoint *temp1 = temp->prev[coord];
//			while (temp1 != NULL && temp->val[coord] < temp1->val[coord]){
//				//if (temp1->minmax == MIN)
//				{
//					if (temp->aabb->overlaps(*temp1->aabb)){
//						result.push_back(Result2B(temp->aabb->cell, temp->aabb->id, temp1->aabb->id));
//						//printf("%f %f %f %f\n", (*it)->lo->val[0], (*it)->hi->val[0], temp->aabb->lo->val[0], temp->aabb->hi->val[0]);
//					}
//					else
//					{
//						result.remove(Result2B(temp->aabb->cell, temp->aabb->id, temp1->aabb->id));
//					}
//				}
//				temp1 = temp1->prev[coord];
//			}
//			if (temp1 != NULL){
//				if (temp->prev[coord] != temp1){
//					EndPoint *temp2 = temp1->next[coord];
//					temp1->next[coord] = temp;
//					temp2->prev[coord] = temp;
//
//					temp->prev[coord]->next[coord] = temp->next[coord];
//					if (temp->next[coord] != NULL)
//						temp->next[coord]->prev[coord] = temp->prev[coord];
//					else{
//						curr[coord] = temp->prev[coord];
//					}
//					temp->next[coord] = temp2;
//					temp->prev[coord] = temp1;
//				}
//			}
//			else{
//				temp->prev[coord]->next[coord] = temp->next[coord];
//				if (temp->next[coord] != NULL)
//					temp->next[coord]->prev[coord] = temp->prev[coord];
//				else
//					curr[coord] = temp->prev[coord];
//				temp->next[coord] = head[coord];
//				temp->prev[coord] = NULL;
//				head[coord]->prev[coord] = temp;
//				head[coord] = temp;
//
//			}
//			temp = temp->next[coord];
//		}
//
//	}
//}

/*void add(AABB* aabb, list<AABB *> &aabb_list, list<Result2B> &result, EndPoint *head[], EndPoint *curr[]){
	for (int coord = 0; coord < 3; coord++){
		if (head[coord] == NULL && curr[coord] == NULL){
			aabb->lo->prev[coord] = NULL;
			aabb->lo->next[coord] = aabb->hi;
			aabb->hi->prev[coord] = aabb->lo;
			aabb->hi->next[coord] = NULL;
			curr[coord] = aabb->hi;
			head[coord] = aabb->lo;

			continue;
		}
		//min
		EndPoint *temp = curr[coord];
		while (temp != NULL && aabb->lo->val[coord] < temp->val[coord]){//flag>0
			//printf("hello %d %f %f\n", coord, (*it)->lo->val[coord],temp->val[coord]);
			if (temp->minmax == ENDPOINT_MAX)
				if (temp->aabb->overlaps(*aabb)){
					result.push_back(Result2B(aabb->cell, aabb->id, temp->aabb->id));

				}
			temp = temp->prev[coord];
		}
		if (temp == NULL){
			aabb->lo->next[coord] = head[coord];
			aabb->lo->prev[coord] = NULL;
			head[coord]->prev[coord] = aabb->lo;
			head[coord] = aabb->lo;
		}
		else{
			//printf("hello %d  %f=======\n", coord, (*it)->lo->val[coord]);
			if (temp != curr[coord]){
				EndPoint *temp1 = temp->next[coord];
				temp->next[coord] = aabb->lo;
				aabb->lo->next[coord] = temp1;
				temp1->prev[coord] = aabb->lo;
				aabb->lo->prev[coord] = temp;

			}
			else{
				curr[coord] = aabb->lo;
				temp->next[coord] = aabb->lo;
				aabb->lo->next[coord] = NULL;
				aabb->lo->prev[coord] = temp;
				//printf("hello %d  %f=======\n", coord, curr[coord]->val[coord]);
			}

		}

		////max
		temp = curr[coord];
		//printf("hello %d %f %f=======\n", coord, temp->val[coord], (*it)->hi->val[coord]);
		while (aabb->hi->val[coord] < temp->val[coord]){//flag>0
			temp = temp->prev[coord];
		}

		if (temp != curr[coord]){
			EndPoint *temp1 = temp->next[coord];
			temp->next[coord] = aabb->hi;
			aabb->hi->next[coord] = temp1;
			temp1->prev[coord] = aabb->hi;
			aabb->hi->prev[coord] = temp;
		}
		else{
			temp->next[coord] = aabb->hi;
			aabb->hi->next[coord] = NULL;
			aabb->hi->prev[coord] = temp;
			curr[coord] = aabb->hi;
		}
	}
}*/

/*bool subdivide(list<Result2B> &result, list<Cell *> &cell_list, vec3f_list &vtxs, tri_list &tris){
	AABB *temp[2];
	for (list<Result2B>::iterator it = result.begin(); it != result.end();) {
		//if (it->cell->id[1] == 12){
		int flag = 0;
		for (int i = 1; i < leave_list.size(); i += 2)
			if (it->cell->id[0] == thread && it->cell->id[1] == leave_list[i]){
				goto next;
			}

		for (list<AABB *>::iterator it1 = it->cell->aabb_list.begin(); it1 != it->cell->aabb_list.end(); it1++) {
			if (it->fid(0) == (*it1)->id){
				temp[flag++] = *it1;
				if (flag == 2){
					if (temp[0]->recursiveOverlaps(temp[1]))
					{
						temp[0]->updatePos(vtxs, mdl1);
						temp[1]->updatePos(vtxs, mdl2);
						mdl1->RefitBVH(true);
						mdl2->RefitBVH(true);
						mdl1->Collide(mdl2);
						mdl1->_num_vf_true = mdl1->_num_ee_true = 0;
					}

					break;
				}
				else
					continue;
			}
			if (it->fid(1) == (*it1)->id){
				temp[flag++] = *it1;
				if (flag == 2){
					if (temp[0]->recursiveOverlaps(temp[1]))
					{
						temp[0]->updatePos(vtxs, mdl1);
						temp[1]->updatePos(vtxs, mdl2);
						mdl1->RefitBVH(true);
						mdl2->RefitBVH(true);
						mdl1->Collide(mdl2);
						mdl1->_num_vf_true = mdl1->_num_ee_true = 0;
					}
					break;
				}
				else
					continue;
			}
		}
		if (flag < 2){
			Result2B temp = *it;
			it++;
			result.remove(temp);
			continue;
			//_tag--;
		}
		else{ it++; continue; }
	next:   Result2B temp = *it;
		it++;
		result.remove(temp);
	}
	return false;
}
void EndSimulation()
{
	delete mdl1;
	delete mdl2;
}*/