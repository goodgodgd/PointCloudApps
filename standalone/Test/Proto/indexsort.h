#ifndef INDEXSORT_H
#define INDEXSORT_H

#include <stdio.h>

// header-only functions should be "inline"
// otherwise multiple definition errors occur
template <typename T>
struct IdxVal
{
    int idx;
    T val;
};

template <typename T>
void swap(IdxVal<T>* data, int i, int j)
{
    IdxVal<T> tmp = data[i];
    data[i] = data[j];
    data[j] = tmp;
}

template <typename T>
void BubbleSortDescending(IdxVal<T>* data, size_t length)
{
    for (int i = 0; i < length; ++i)
    {
        bool swapped = false;
        for (int j = 0; j < length - (i+1); ++j)
        {
            if (data[j].val < data[j+1].val)
            {
                swap(data, j, j+1);
                swapped = true;
            }
        }

        if (!swapped) break;
    }
}

#endif // INDEXSORT_H
