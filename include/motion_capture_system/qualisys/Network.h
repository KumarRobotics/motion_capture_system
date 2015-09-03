#ifndef NETWORK_H
#define NETWORK_H

typedef unsigned int DWORD;

class COutput;

class CNetwork
{
public:
    CNetwork();
    bool  Connect(char* pServerAddr, int nPort);
    void  Disconnect();
    bool  Connected();
    bool  CreateUDPSocket(int nUDPPort, bool bBroadcast = false);
    int   Receive(char* rtDataBuff, int nDataBufSize, bool bHeader, int nTimeout, unsigned int *ipAddr = NULL);
    bool  Send(const char* pSendBuf, int nSize);
    bool  SendUDPBroadcast(const char* pSendBuf, int nSize, short nPort, unsigned int nFilterAddr = 0);
    char* GetErrorString();
    int   GetError();
    bool  IsLocalAddress(unsigned int nAddr);

private:
    bool InitWinsock();
    void SetErrorString();

private:
    COutput*   mpoOutput;
    //modified Socket to int (socket is windows)
    int     mhSocket;
    int     mhUDPSocket;
    int      mhUDPBroadcastSocket;
    //---------------------------------------
    char       maErrorStr[256];
    DWORD      mnLastError;
};


#endif
