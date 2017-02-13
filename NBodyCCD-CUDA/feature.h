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

#include "forceline.h"
#define UINT_MAX      0xffffffff

class edge2f {
    unsigned int _vids[2];
    unsigned int _fids[2];

    FORCEINLINE void set(unsigned int id0, unsigned int id1) {
        if (id0 > id1) {
            _vids[0] = id0;
            _vids[1] = id1;
        } else {
            _vids[1] = id0;
            _vids[0] = id1;
        }
    }
public:
    FORCEINLINE edge2f() {
        _vids[0] = _vids[1] = UINT_MAX;
        _fids[0] = _fids[1] = UINT_MAX;
    }

    FORCEINLINE edge2f(unsigned id0, unsigned int id1, unsigned int fid) {
        set(id0, id1);
        _fids[0] = fid;
        _fids[1] = UINT_MAX;
    }

    FORCEINLINE void set_fid2(unsigned id) {
        _fids[1] = id;
    }

    FORCEINLINE unsigned int vid(int i) { return _vids[i]; }
    FORCEINLINE unsigned int fid(int i) { return _fids[i]; }

    FORCEINLINE bool operator == (const edge2f &other) const {
        return (_vids[0] == other._vids[0] && _vids[1] == other._vids[1]);
    }

    FORCEINLINE bool operator < (const edge2f &other) const {
        if (_vids[0] == other._vids[0])
            return _vids[1] < other._vids[1];
        else
            return _vids[0] < other._vids[0];
    }
};

class tri3e {
    unsigned int _ids[3];

public:
    FORCEINLINE tri3e() {
        _ids[0] = _ids[1] = _ids[2] = UINT_MAX;
    }

    FORCEINLINE tri3e(unsigned int id0, unsigned int id1, unsigned id2) {
        set(id0, id1, id2);
    }

    FORCEINLINE void set(unsigned int id0, unsigned int id1, unsigned int id2) {
        _ids[0] = id0;
        _ids[1] = id1;
        _ids[2] = id2;
    }

    FORCEINLINE unsigned int id(int i) { return _ids[i]; }
};

class tri3f {
public:
	unsigned int _ids[3];
	unsigned int _flag;

	FORCEINLINE tri3f() {
		_ids[0] = _ids[1] = _ids[2] = -1;
	}

	FORCEINLINE tri3f(unsigned int id0, unsigned int id1, unsigned int id2) {
		set(id0, id1, id2);
	}

	FORCEINLINE void set(unsigned int id0, unsigned int id1, unsigned int id2) {
		_ids[0] = id0;
		_ids[1] = id1;
		_ids[2] = id2;
	}
	FORCEINLINE void set(unsigned int flag){
		_flag = flag;
	}
	FORCEINLINE unsigned int id(int i) { return _ids[i]; }
	FORCEINLINE unsigned int id0() { return _ids[0]; }
	FORCEINLINE unsigned int id1() { return _ids[1]; }
	FORCEINLINE unsigned int id2() { return _ids[2]; }
	FORCEINLINE unsigned int flag() { return _flag; }
};

class tri3f1 {
public:
	unsigned int _ids[3];

	FORCEINLINE tri3f1() {
		_ids[0] = _ids[1] = _ids[2] = -1;
	}

	FORCEINLINE tri3f1(unsigned int id0, unsigned int id1, unsigned int id2) {
		set(id0, id1, id2);
	}

	FORCEINLINE void set(unsigned int id0, unsigned int id1, unsigned int id2) {
		_ids[0] = id0;
		_ids[1] = id1;
		_ids[2] = id2;
	}

	FORCEINLINE unsigned int id(int i) { return _ids[i]; }
	FORCEINLINE unsigned int id0() { return _ids[0]; }
	FORCEINLINE unsigned int id1() { return _ids[1]; }
	FORCEINLINE unsigned int id2() { return _ids[2]; }
};
#include <vector>
typedef std::vector<tri3f> tri_list;
