#ifndef ARRAYDATA_H
#define ARRAYDATA_H

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

private:
    T* data;
    int size;
};

#endif // ARRAYDATA_H
