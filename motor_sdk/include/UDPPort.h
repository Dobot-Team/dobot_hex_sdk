#ifndef UDPPORT_H
#define UDPPORT_H

#include <arpa/inet.h>
#include <string>
#include <string.h>
#include <iostream>
#include <unistd.h>

#include "IOPort.h"

class UDPPort : public IOPort{
public:
    UDPPort(std::string toIP, uint toPort, uint ownPort, 
            size_t recvLength = 0,
            size_t timeOutUs = 20000,
            BlockYN blockYN = BlockYN::NO);
    ~UDPPort();
    size_t Send(uint8_t *sendMsg, size_t sendMsgLength);
    size_t Recv(uint8_t *recvMsg, size_t recvLength);
    size_t Recv(uint8_t *recvMsg);
    bool SendRecv(uint8_t *sendMsg, uint8_t *recvMsg, size_t sendLength);
private:
    size_t _recvBlock(uint8_t *recvMsg);
    size_t _recvUnBlock(uint8_t *recvMsg);
    sockaddr_in _ownAddr, _toAddr, _fromAddr;
    socklen_t _sockaddrSize;
    int _sockfd;
    int _on = 1;
    size_t _sentLength;

    fd_set _rSet;
    // timeval _timeout;
};


#endif  // UDPPORT_H
