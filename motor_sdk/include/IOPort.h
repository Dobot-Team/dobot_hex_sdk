#ifndef IOPORT_H
#define IOPORT_H

#include <stdint.h>
#include <unistd.h>
#include <string>

enum class BlockYN{
    YES,
    NO
};


class IOPort{
public:
    IOPort(BlockYN blockYN, size_t recvLength, size_t timeOutUs){
        ResetIO(blockYN, recvLength, timeOutUs);
    }
    virtual ~IOPort(){}
    virtual size_t Send(uint8_t *sendMsg, size_t sendLength) = 0;
    virtual size_t Recv(uint8_t *recvMsg, size_t recvLength) = 0;
    virtual size_t Recv(uint8_t *recvMsg) = 0;
    virtual bool SendRecv(uint8_t *sendMsg, uint8_t *recvMsg, size_t sendLength) = 0;
    void ResetIO(BlockYN blockYN, size_t recvLength, size_t timeOutUs);
protected:
    BlockYN _blockYN = BlockYN::NO;
    size_t _recvLength;
    timeval _timeout;
    timeval _timeoutSaved;
};

#endif  // IOPORT_H