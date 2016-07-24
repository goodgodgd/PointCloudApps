#ifndef ANNOTATION_H
#define ANNOTATION_H

#include <QString>
#include "range.h"

struct Annotation
{
    Annotation(const QString category_, int instance_, int xl_, int xh_, int yl_, int yh_)
    {
        category = category_;
        instanceID = instance_;
        imrect = ImRect(xl_, xh_, yl_, yh_);
    }

    QString category;
    int instanceID;
    ImRect imrect;
};

typedef std::vector<Annotation> vecAnnot;

#endif // ANNOTATION_H
