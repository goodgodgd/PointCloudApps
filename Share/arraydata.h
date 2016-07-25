#ifndef ARRAYDATA_H
#define ARRAYDATA_H

#include <stdio.h>
#include <string.h>
#include "Share/project_common.h"

template<typename T>
struct ArrayData
{
    ArrayData() : data(nullptr), size(0) {}
    ArrayData(int size_)
    {
        size = size_;
        data = new T[size];
    }
    ~ArrayData()
    {
        Destroy();
    }
    void Destroy()
    {
        if(data!=nullptr)
            delete[] data;
    }
    void Allocate(int size_)
    {
        Destroy();
        size = size_;
        data = new T[size];
    }
    T* GetArrayPtr()
    {
        return data;
    }
    int ArraySize()
    {
        return size;
    }
    int ByteSize()
    {
        return size*sizeof(T);
    }
    void SetZero()
    {
        memset(data, 0x00, ByteSize());
    }

private:
    T* data;
    int size;
};

#endif // ARRAYDATA_H
