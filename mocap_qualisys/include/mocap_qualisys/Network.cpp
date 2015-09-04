/* New operating system independend Socket class started 16.11.2012 Sebastian
 * Tauscher  Neu schreiben eines Platform unabhängigen SDKs?*/


#if defined(_WIN32) && !defined(__CYGWIN__)
  #include <windows.h>
  #include <winsock2.h>
  #include <iphlpapi.h>
#else //Linux includes
  #include <netinet/tcp.h>
  	#include <sys/time.h>
	#include <errno.h>
	#include <netdb.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <sys/ioctl.h>
	#include <unistd.h>
	#include <sys/select.h> /* this might not be needed*/
	#include <ifaddrs.h>
	#ifndef SOCKET
		#define  SOCKET int
	#endif
#endif

#include <stdio.h>
#include "Network.h"
#define INVALID_SOCKET -1;
#define SOCKET_ERROR -1;
// INVALID_Socket == -1

CNetwork::CNetwork()
{
    mhSocket             = -1;
    mhUDPSocket          = -1;
    mhUDPBroadcastSocket = -1;
    mnLastError          = 0;
    maErrorStr[0]        = 0;
}


bool CNetwork::InitWinsock()
{
	//if _win32 added----------------------------------------------------
	#if defined(_WIN32) && !defined(__CYGWIN__)
    WORD    wVersionRequested = MAKEWORD(2,2);
    WSADATA wsaData;

    // Initialize WinSock and check version

    if (WSAStartup(wVersionRequested, &wsaData) != 0)
    {
        SetErrorString();
        return false;
    }
    if (wsaData.wVersion != wVersionRequested)
    {
        SetErrorString();
        return false;
    }

	#endif
	//-------------------------------------------------------------------
    return true;
} // InitWinsock


bool CNetwork::Connect(char* pServerAddr, int nPort)
{
    mnLastError   = 0;
    maErrorStr[0] = 0;

	//Added INITWinSock just neccesarry when the OS is Windows-----------
	#if defined(_WIN32) && !defined(__CYGWIN__)
    if (InitWinsock() == false)
    {
        return false;
    }
	#endif
	//--------------------------------------------------------------------

    // Connect to QTM RT server.

    mhSocket = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in sAddr;

    // First check if the address is a dotted number "A.B.C.D"

    sAddr.sin_addr.s_addr = inet_addr(pServerAddr);
    if (sAddr.sin_addr.s_addr == INADDR_NONE)
    {
        // If it wasn't a dotted number lookup the server name
        hostent *psHost = gethostbyname(pServerAddr);
        if (!psHost)
        {
            close(mhSocket);
            return false;
        }
        sAddr.sin_addr = *((in_addr*)psHost->h_addr_list[0]);
    }

    sAddr.sin_port = htons(nPort);
    sAddr.sin_family = AF_INET;
	int check = connect(mhSocket,  reinterpret_cast<sockaddr*>(&sAddr), sizeof(sAddr));
    if (check == -1)
    {
        SetErrorString();
        close(mhSocket);
        return false;
    }

    // Disable Nagle's algorithm
    //Changed from char to int Tauscher ------------------------------------

    int bNoDelay = 1;
	//---------------------------------------------------------------------

    if (setsockopt(mhSocket, IPPROTO_TCP, TCP_NODELAY, (char*)&bNoDelay, sizeof(bNoDelay)))
    {
        SetErrorString();
        close(mhSocket);
        return false;
    }
    return true;
} // Connect


void CNetwork::Disconnect()
{
    // Try to shutdown gracefully

    shutdown(mhSocket, SHUT_WR);
    int nRecved = 1;
    char pData[500];
    while (nRecved > 0)
    {
        //
        // There shouldn't be anything left to receive now, but check just to make sure
        //
        nRecved = recv(mhSocket, pData, sizeof(pData), 0);
    }
    close(mhSocket);
    close(mhUDPSocket);
    close(mhUDPBroadcastSocket);
    mhSocket             = INVALID_SOCKET;
    mhUDPSocket          = INVALID_SOCKET;
    mhUDPBroadcastSocket = INVALID_SOCKET;
    #if defined(_WIN32) && !defined(__CYGWIN__)
    WSACleanup();
    #endif
} // Disconnect


bool CNetwork::Connected()
{
    return mhSocket != INVALID_SOCKET;
}

bool CNetwork::CreateUDPSocket(int nUDPPort, bool bBroadcast)
{

    if (nUDPPort > 1023)
    {
        int tempSocket = INVALID_SOCKET;

        // Create UDP socket for data streaming
        sockaddr_in RecvAddr;
        RecvAddr.sin_family = AF_INET;
        RecvAddr.sin_port = htons(nUDPPort);
        RecvAddr.sin_addr.s_addr = INADDR_ANY;

        tempSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (tempSocket != -1)
        {
            u_long argp = 1;
            // Make socket unblocking. windows ioctlsocket(...,..,...)---
            // Linux is ioctl---------------------------------------------
            if (ioctl(tempSocket, FIONBIO , &argp) == 0)
            //-------------------------------------------------------------
            {
                if (bind(tempSocket, (sockaddr *) &RecvAddr, sizeof(RecvAddr)) != -1)
                {
                    if (bBroadcast)
                    {
						//changed to int----------------------------
                        int nBroadcast = 1;
                        //--------------------------------------------
                        if (setsockopt(tempSocket, SOL_SOCKET, SO_BROADCAST, &nBroadcast,
                                       sizeof(nBroadcast)) == 0)
                        {
                            mhUDPBroadcastSocket = tempSocket;
                            return true;
                        }
                        else
                        {
                            snprintf(maErrorStr, sizeof(maErrorStr), "Failed to set socket options for UDP server socket.");
                        }
                    }
                    else
                    {
                        mhUDPSocket = tempSocket;
                        return true;
                    }
                }
                else
                {
                    snprintf(maErrorStr, sizeof(maErrorStr), "Failed to bind UDP server socket.");
                }
            }
            else
            {
                snprintf(maErrorStr, sizeof(maErrorStr), "Failed to make UDP server socket unblocking.");
            }
        }
        else
        {
            snprintf(maErrorStr, sizeof(maErrorStr), "Failed to create UDP server socket.");
        }
        close(tempSocket);
    }

    return false;
} // CreateUDPSocket




// Receive a data packet. Data is stored in a local static buffer
// Returns number of bytes in received message or -1 if there is an error.

int CNetwork::Receive(char* rtDataBuff, int nDataBufSize, bool bHeader, int nTimeout, unsigned int *ipAddr)
{
    int         nRecved = 0;
    //int         nRecvedTotal = 0;
    bool IsSOCKET=0, IsUDPSOCKET=0, IsUDPBROADCASTSOCKET= 0;
    sockaddr_in source_addr;
    socklen_t        fromlen = sizeof(source_addr);


    if (mhSocket != -1)
    {
        IsSOCKET = true;
    }
    if (mhUDPSocket != -1)
    {
        IsUDPSOCKET=true;
    }
    if (mhUDPBroadcastSocket != -1)
    {
        IsUDPBROADCASTSOCKET = true;
    }

   //timeval *pTimeout = NULL;
   //timeval  timeout;

    //if (nTimeout < 0)
    //{
    //    pTimeout = NULL;
    //}
    //else
    //{
    //    timeout.tv_sec  = 20000000 / 1000000;
    //    timeout.tv_usec = 20000000%1000000;//nTimeout % 1000000;
    //    pTimeout = &timeout;
    //}
    //-------------------------------------------------------------------
	//The FD_SET + Select stuff did'nt worked - so tried to work arround
	//but this doesn't works...no idea why ---S.Tauscher--------------------
	//--------------------------------------------------------------------


    // Wait for activity on the TCP and UDP sockets.
   // int test[nDataBufSize];
    //nRecved = recv(mhSocket, rtDataBuff, bHeader ? 8 : nDataBufSize, 0);

    //if ( timeout.tv_sec > 0 )
    //{

    //setsockopt(mhSocket, SOL_SOCKET, SO_RCVTIMEO,
      //         (char*)&(pTimeout), sizeof(pTimeout));
    //}


	if (IsSOCKET)
	{
		nRecved = recv(mhSocket, rtDataBuff, bHeader ? 8 : nDataBufSize, 0);
	}
	else if (IsUDPSOCKET)
	{
		nRecved = recvfrom(mhUDPSocket, rtDataBuff, nDataBufSize, 0, (sockaddr*)&source_addr, &fromlen);


	}
	else if (IsUDPBROADCASTSOCKET)
	{
		sockaddr_in source_addr;
		nRecved = recvfrom(mhUDPBroadcastSocket, rtDataBuff, nDataBufSize, 0, (sockaddr*)&source_addr, &fromlen);
		if (ipAddr)
		{
			*ipAddr = source_addr.sin_addr.s_addr;
		}
	}


    if (nRecved >= 0)
    {
        return nRecved;
    }

    SetErrorString();
    Disconnect();
    return -1;
} // RecvMessage


bool CNetwork::Send(const char* pSendBuf, int nSize)
{
    int         nSent      = 0;
    int         nTotSent   = 0;

    while (nTotSent < nSize)
    {
        nSent = send(mhSocket, pSendBuf + nTotSent, nSize - nTotSent, 0);
        if (nSent == -1)
        {
            SetErrorString();
            return false;
        }
        nTotSent += nSent;
    }

    return true;
} // Send





bool CNetwork::SendUDPBroadcast(const char* pSendBuf, int nSize, short nPort, unsigned int nFilterAddr /* = 0 */ )
{
    bool bBroadCastSent = false;

    if (mhUDPBroadcastSocket != -1)
    {
			#if defined(_WIN32) && !defined(__CYGWIN__)
			IP_ADAPTER_INFO* pAdptInfo  = NULL;
			IP_ADAPTER_INFO* pNextAd    = NULL;
			ULONG ulLen                 = 0;
			DWORD erradapt;

			// Find all network interfaces.
			erradapt = ::GetAdaptersInfo(pAdptInfo, &ulLen);
			if (erradapt == ERROR_BUFFER_OVERFLOW)
			{
				pAdptInfo = (IP_ADAPTER_INFO*)malloc(ulLen);
				erradapt = ::GetAdaptersInfo(pAdptInfo, &ulLen);
			}

			if (erradapt == ERROR_SUCCESS)
			{
				sockaddr_in recvAddr;
				recvAddr.sin_family      = AF_INET;
				recvAddr.sin_port        = htons(nPort);
				recvAddr.sin_addr.s_addr = 0xffffffff;

				// Send broadcast on all Ethernet interfaces.
				bool bWaitForResponse = false;
				pNextAd = pAdptInfo;
				while( pNextAd )
				{
					if (pNextAd->Type == MIB_IF_TYPE_ETHERNET)
					{
						unsigned int nIPaddr = inet_addr(pNextAd->IpAddressList.IpAddress.String);
						unsigned int nIPmask = inet_addr(pNextAd->IpAddressList.IpMask.String);
						unsigned int nMaskedLocalIp = nFilterAddr | (~nIPmask);
						recvAddr.sin_addr.s_addr = nIPaddr | (~nIPmask);
						if (recvAddr.sin_addr.s_addr != (nFilterAddr | (~nIPmask)))
						{
							if (sendto(mhUDPBroadcastSocket, pSendBuf, nSize, 0, (sockaddr*)&recvAddr, sizeof(recvAddr)) == nSize)
							{
								bBroadCastSent = true;
							}
						}
					}
					pNextAd = pNextAd->Next;
				}
			}
			free(pAdptInfo);

			//-----------Linux Version-S.Tauscher-------------------------
			#else
					struct ifaddrs *pAdptInfo, *pNextAd ;

					// Find all network interfaces.
					if (getifaddrs(&pAdptInfo) == -1) {
					   perror("getifaddrs");
				   }
				   else{
						sockaddr_in recvAddr;
						recvAddr.sin_family      = AF_INET;
						recvAddr.sin_port        = htons(nPort);
						recvAddr.sin_addr.s_addr = 0xffffffff;

						// Send broadcast on all Ethernet interfaces.
						//bool bWaitForResponse = false;
						pNextAd = pAdptInfo;
						while( pNextAd )
						{
							if (pNextAd->ifa_addr->sa_family == AF_INET)
							{
								unsigned int nIPaddr = inet_addr(pNextAd->ifa_addr->sa_data);
								unsigned int nIPmask = inet_addr(pNextAd->ifa_netmask->sa_data);
								//unsigned int nMaskedLocalIp = nFilterAddr | (~nIPmask);
								recvAddr.sin_addr.s_addr = nIPaddr | (~nIPmask);

									if (sendto(mhUDPBroadcastSocket, pSendBuf, nSize, 0, (sockaddr*)&recvAddr, sizeof(recvAddr)) == nSize)
									{
										bBroadCastSent = true;
									}

							}
							pNextAd = pNextAd->ifa_next;
						}
					}
				freeifaddrs(pAdptInfo);
       #endif
       //------------------------------------------------------------------
    }

    return bBroadCastSent;
} // SendUDPBroadcast


void CNetwork::SetErrorString()
{
    char *tError = NULL;
    mnLastError  = GetError();
    //DWORD nRet   = FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
                                // NULL, mnLastError, 0, reinterpret_cast<LPTSTR>(&tError), 0, NULL);

    printf(maErrorStr, sizeof(maErrorStr), "%s", tError);

    //LocalFree(tError);
}


char* CNetwork::GetErrorString()
{
    return maErrorStr;
}


int CNetwork::GetError()
{
    return mnLastError;
}


bool CNetwork::IsLocalAddress(unsigned int nAddr)
{
    #if defined(_WIN32) && !defined(__CYGWIN__)
		IP_ADAPTER_INFO* pAdptInfo  = NULL;
		IP_ADAPTER_INFO* pNextAd    = NULL;
		DWORD            erradapt;
		ULONG            ulLen      = 0;
		unsigned int     nAddrTmp   = 0;

		// Find all network interfaces.
		erradapt = ::GetAdaptersInfo(pAdptInfo, &ulLen);
		if (erradapt == ERROR_BUFFER_OVERFLOW)
		{
			pAdptInfo = (IP_ADAPTER_INFO*)malloc(ulLen);
			erradapt = ::GetAdaptersInfo(pAdptInfo, &ulLen);
		}

		if (erradapt == ERROR_SUCCESS)
		{
			pNextAd = pAdptInfo;
			while( pNextAd )
			{
				if (pNextAd->Type == MIB_IF_TYPE_ETHERNET)
				{
					// Check if it's a response from a local interface.
					if (inet_addr(pNextAd->IpAddressList.IpAddress.String) == nAddr)
					{
						return true;
					}
				}
				pNextAd = pNextAd->Next;
			}
		}
		free(pAdptInfo);
    #else
			struct ifaddrs *pAdptInfo, *pNextAd ;

			if (getifaddrs(&pAdptInfo) == -1) {
			   perror("getifaddrs");
		   }
		   else{

				pNextAd = pAdptInfo;
				while( pNextAd )
				{
					if (pNextAd->ifa_addr->sa_family== AF_INET)
					{
						 struct sockaddr_in *pNextAd_in = (struct sockaddr_in *)pNextAd->ifa_addr;


						if (pNextAd_in->sin_addr.s_addr == nAddr)
						{
							return true;
						}
					}
					pNextAd = pNextAd->ifa_next;
				}
			}
		freeifaddrs(pAdptInfo);
       #endif

    return false;
}
