#ifndef FORDESCRIPTOR_H
#define FORDESCRIPTOR_H

#include <CL/cl.h>
typedef cl_float4 DescType;

#define NUM_VAR                 6
#define PT_DIM                  3
#define L_DIM                   (NUM_VAR+PT_DIM)
#define L_WIDTH                 (L_DIM+1)
#define L_INDEX(y,x)            ((y)*L_WIDTH+(x))
#define DESC_EQUATION_SIZE      L_DIM*L_WIDTH


#endif // FORDESCRIPTOR_H
