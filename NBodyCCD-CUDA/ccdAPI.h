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

#pragma once
#include "vec3f.h"
#include "feature.h"

typedef void ccdEETestCallback(unsigned int e1_v1, unsigned int e1_v2, unsigned int e2_v1, unsigned int e2_v2, float t);
typedef void ccdVFTestCallback(unsigned int vid, unsigned int fid, float t);

extern void ccdInitModel(vec3f_list &, tri_list &);
extern void ccdInitModel(vec3f_list &, tri_list &, vec3f_list &, tri_list &);
extern void ccdUpdateVtxs(vec3f_list &);
extern void ccdUpdateVtxs(vec3f_list &, vec3f_list &);
extern void ccdQuitModel();
extern void ccdChecking(bool);
extern void ccdReport();
extern void ccdSetEECallback(ccdEETestCallback *funcEE);
extern void ccdSetVFCallback(ccdVFTestCallback *funcVF);
