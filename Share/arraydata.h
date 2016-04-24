#ifndef ARRAYDATA_H
#define ARRAYDATA_H

template<typename T>
struct ArrayData
{
    ArrayData() : data(nullptr) {}
    ArrayData(int size) { data = new T[size]; }
    ~ArrayData() { Destroy(); }
    void Destroy()
    {
        if(data!=nullptr)
            delete[] data;
    }
    void Allocate(int size)
    {
        Destroy();
        data = new T[size];
    }
    T* GetArrayPtr()
    {
        return data;
    }

private:
    T* data;
};

#endif // ARRAYDATA_H
