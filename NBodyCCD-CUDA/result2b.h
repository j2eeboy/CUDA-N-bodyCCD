#pragma once
#include"myfeature.h"

class Result2B{
private:
    unsigned int id[2];
public:
    Cell *cell;

	CUDA_CALLABLE_MEMBER Result2B(){}

    CUDA_CALLABLE_MEMBER Result2B(Cell *cell_father, unsigned int id0, unsigned int id1){
        cell = cell_father;
        if (id0 < id1) {
            id[0] = id0;
            id[1] = id1;
        }
        else {
            id[1] = id0;
            id[0] = id1;
        }
    }

	CUDA_CALLABLE_MEMBER int fid(int i){ return id[i]; }
	CUDA_CALLABLE_MEMBER bool operator != (const Result2B &other) const {
        return (id[0] != other.id[0] || id[1] != other.id[1]);
    }
	CUDA_CALLABLE_MEMBER bool operator == (const Result2B &other) const {
        return (id[0] == other.id[0] && id[1] == other.id[1]);
    }
	CUDA_CALLABLE_MEMBER bool operator < (const Result2B &other) const {
		return comp(*this, other);
    }

	CUDA_CALLABLE_MEMBER void set(unsigned int id0, unsigned int id1) {
		if (id0 < id1) {
			id[0] = id0;
			id[1] = id1;
		}
		else {
			id[1] = id0;
			id[0] = id1;
		}
	}


	CUDA_CALLABLE_MEMBER static bool comp(const Result2B &lhs, const Result2B& rhs)
	{
		if (lhs.id[0] == rhs.id[0])
			return lhs.id[1] < rhs.id[1];
		else
			return lhs.id[0] < rhs.id[0];
	}

};

struct Result2B_is_valid
{
	CUDA_CALLABLE_MEMBER
		bool operator()(Result2B x)
	{
		return (x.fid(0)) != UINT_MAX;
	}
};

// struct Result2BComp
// {
// 	CUDA_CALLABLE_MEMBER bool operator () (const Result2B &lhs, const Result2B& rhs) const {
// 		return Result2B::comp(lhs, rhs);
// 	}
// };