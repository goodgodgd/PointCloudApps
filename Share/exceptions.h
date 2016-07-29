#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include <QString>

struct TryFrameException
{
    TryFrameException(QString msg_) : msg(msg_) {}
    QString msg;
};

#endif // EXCEPTIONS_H
