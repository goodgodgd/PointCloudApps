#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

struct TryFrameException
{
    TryFrameException(QString msg_) : msg(msg_) {}
    QString msg;
};

#endif // EXCEPTIONS_H
