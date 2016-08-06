#ifndef TESTINDEXSORT_H
#define TESTINDEXSORT_H

#include "Share/project_common.h"
#include "Proto/indexsort.h"

namespace Test
{
inline void testBubbleSort()
{
    IdxVal<float> data[10];
    printf("TestSort data: ");
    for(int i=0; i<10; i++)
    {
        data[i].idx = i;
        if(i<5)
            data[i].val = 2.f*i+1.f;
        else
            data[i].val = 2.f*(i-4);
        printf("(%d,%.2f) ", data[i].idx, data[i].val);
    }

    BubbleSortDescending(data, 10);

    printf("TestSort result: ");
    for(int i=0; i<10; i++)
        printf("(%d,%.2f) ", data[i].idx, data[i].val);
}

}
#endif // TESTINDEXSORT_H
