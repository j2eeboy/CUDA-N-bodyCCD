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

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <assert.h>

#include "plyfile.h"
#pragma warning(disable: 4996)

#include <iostream>
using namespace std;

#include "vec3f.h"
#include "feature.h"

typedef struct PLYVertex{
    float coords[3];
    unsigned char color[3];
    void *other_props;
} PLYVertex;

typedef struct PLYFace{
    unsigned char nverts;
    int *verts;
    void *other_props;
} PLYFace;

PlyProperty vert_props[] = { /* list of property information for a vertex */
        {"x", PLY_FLOAT, PLY_FLOAT, 0, 0, 0, 0, 0},
        {"y", PLY_FLOAT, PLY_FLOAT, 4, 0, 0, 0, 0},
        {"z", PLY_FLOAT, PLY_FLOAT, 8, 0, 0, 0, 0},
        {"red", PLY_UCHAR, PLY_UCHAR, (int)offsetof(PLYVertex,color[0]), 0, 0, 0, 0},
        {"green", PLY_UCHAR, PLY_UCHAR, (int)offsetof(PLYVertex,color[1]), 0, 0, 0, 0},
        {"blue", PLY_UCHAR, PLY_UCHAR, (int)offsetof(PLYVertex,color[2]), 0, 0, 0, 0},
};

PlyProperty face_props[] = { /* list of property information for a vertex */
        {"vertex_indices", PLY_INT, PLY_INT, offsetof(PLYFace,verts), 1, PLY_UCHAR, PLY_UCHAR, offsetof(PLYFace,nverts)},
};

void BuildSession(char *fname, unsigned int num_frame, float ply_scale,
                  unsigned int &vtx_num, vec3f_list &vtxs, tri_list &tris)
{
    vtx_num = 0;
    //all_vtxs = NULL;

    char ply_fname[256];
    for (unsigned int cur_f = 0; cur_f < num_frame; cur_f++) {
        sprintf(ply_fname, "%s%d.ply", fname, cur_f);

        FILE *fp = fopen(ply_fname, "rb");
        assert(fp);

        // PLY object:
        PlyFile *ply;

        // PLY properties:
        char **elist;
        int nelems;

        // hand over the stream to the ply functions:
        ply = ply_read(fp, &nelems, &elist);
        assert(ply);

        int file_type;
        float version;
        ply_get_info(ply, &version, &file_type);

        for (int i=0; i<nelems; i++) {
            char *elem_name = elist[i];

            int num_elems, nprops;
            PlyProperty **plist = ply_get_element_description(ply, elem_name, &num_elems, &nprops);

            bool has_vertex_x = false, has_vertex_y = false, has_vertex_z = false, has_colors = false;
            unsigned char color_components = 0;

            // this is a vertex:
            if (equal_strings ("vertex", elem_name)) {
                for (int j=0; j<nprops; j++)
                {
                    if (equal_strings("x", plist[j]->name))
                    {
                        ply_get_property (ply, elem_name, &vert_props[0]);  /* x */
                        has_vertex_x = true;
                    }
                    else if (equal_strings("y", plist[j]->name))
                    {
                        ply_get_property (ply, elem_name, &vert_props[1]);  /* y */
                        has_vertex_y = true;
                    }
                    else if (equal_strings("z", plist[j]->name))
                    {
                        ply_get_property (ply, elem_name, &vert_props[2]);  /* z */
                        has_vertex_z = true;
                    }
                    else if (equal_strings("red", plist[j]->name))
                    {
                        ply_get_property (ply, elem_name, &vert_props[3]);  /* z */
                        color_components++;
                    }
                    else if (equal_strings("green", plist[j]->name))
                    {
                        ply_get_property (ply, elem_name, &vert_props[4]);  /* z */
                        color_components++;
                    }
                    else if (equal_strings("blue", plist[j]->name))
                    {
                        ply_get_property (ply, elem_name, &vert_props[5]);  /* z */
                        color_components++;
                    }
                }

                has_colors = color_components == 3;
                // test for necessary properties

                if ((!has_vertex_x) || (!has_vertex_y) || (!has_vertex_z))
                {
                    cout << "Warning: Vertex with less than 3 coordinated detected. Output will most likely be corrupt!" << endl;
                    continue;
                }

                // must be first frame, initialize structures:
                if (cur_f  == 0) {
                    vtx_num = num_elems;
                    //all_vtxs = new vec3f[num_elems*num_frame];
                }

                // grab all the vertex elements
                PLYVertex plyNewVertex;
				//printf("%d %d\n", nelems, num_elems);

                for (int j=0; j<num_elems; j++) {
                    ply_get_element(ply, (void *)&plyNewVertex);

                    //all_vtxs[cur_f*vtx_num+j] = vec3f(plyNewVertex.coords) * ply_scale;
                    if (cur_f == 0)
                        vtxs.push_back(vec3f(plyNewVertex.coords) * ply_scale);

                    if (j != 0 && j%1000000 == 0) {
                        cout << " - " << j << " of " << num_elems << " loaded." << endl;
                    }
                }
            }

                // this is a face (and, hopefully, a triangle):
            else if (equal_strings ("face", elem_name) && tris.empty()) {
                // I need this for..., otherwise error ...

                for (int j=0; j<nprops; j++)
                {
                    if (equal_strings("vertex_indices", plist[j]->name))
                    {
                        ply_get_property (ply, elem_name, &face_props[0]);  /* vertex_indices */
                    }
                }

                /* grab all the face elements */
                PLYFace plyFace;
                plyFace.other_props = NULL;
				//printf("%d %d\n", nelems, num_elems);
                for (int j = 0; j < num_elems; j++) {
                    ply_get_element(ply, (void *)&plyFace);
                    for (int fi = 0; fi < plyFace.nverts-2; fi++) {
                        //
                        // make a triangle in our format from PLY face + vertices
                        //
                        // copy vertex indices
                        unsigned int id0, id1, id2;

                        id0 = plyFace.verts[0];
                        id1 = plyFace.verts[fi+1];
                        id2 = plyFace.verts[fi+2];

                        tri3f tri(id0, id1, id2);

                        // insert triangle into list
                        tris.push_back(tri);
                    }
                    free(plyFace.verts);

                    if (j != 0 && j%500000 == 0) {
                        cout << " - " << j << " of " << num_elems << " loaded." << endl;
                    }
                }
            }

            else // otherwise: skip all further
                NULL;
        }

        // PLY parsing ended, clean up vertex buffer and close the file

        ply_close(ply);

        // fclose(fp);
    }

    assert(vtxs.size() == vtx_num);
}

