#include "RTProtocol.h"
#include "NBC_Markup.h"
#include "Network.h"
#include "StdString.h"

#include <string.h>
#include <float.h>
#include "inet.h"

#define SOCKET_ERROR -1

CRTProtocol::CRTProtocol()
{
    mpoNetwork              = new CNetwork();
    mpoRTPacket             = NULL;
    meLastEvent             = CRTPacket::EventCaptureStopped;
    mnMajorVersion          = 1;
    mnMinorVersion          = 0;
    mbBigEndian             = false;
    maErrorStr[0]           = 0;
    bBroadcastSocketCreated = false;
    mpFileBuffer            = NULL;
} // CRTProtocol


CRTProtocol::~CRTProtocol()
{
    if (mpoNetwork)
    {
        delete mpoNetwork;
        mpoNetwork = NULL;
    }
    if (mpoRTPacket)
    {
        delete mpoRTPacket;
        mpoRTPacket = NULL;
    }
} // ~CRTProtocol


bool CRTProtocol::Connect(char* pServerAddr, int nPort, int nUDPServerPort,
                          int nMajorVersion, int nMinorVersion, bool bBigEndian)
{
    CRTPacket::EPacketType eType;
    char                   tTemp[100];
    char                   pResponseStr[256];

    mnMinorVersion = nMinorVersion;
    mnMajorVersion = nMajorVersion;
    mbBigEndian    = false;

    if ((mnMajorVersion != 1) || (mnMinorVersion != 0))
    {
        if (mbBigEndian)
        {
            nPort += 2;
        }
        else
        {
            nPort += 1;
        }
    }

    if (mpoRTPacket)
    {
        delete mpoRTPacket;
    }
    mpoRTPacket = new CRTPacket(nMajorVersion, nMinorVersion, bBigEndian);

    if (mpoNetwork->Connect(pServerAddr, nPort))
    {

        if (nUDPServerPort != 0)
        {
            if (mpoNetwork->CreateUDPSocket(nUDPServerPort) == false)
            {
        //----------------------------------------------------------------------------------------------------
                printf( "CreateUDPSocket failed. %s", mpoNetwork->GetErrorString());
                //----------------------------------------------------------------------------------------------------
                Disconnect();
                 printf("UDP fail...\n");
                return false;
            }
        }

    bool check=ReceiveRTPacket(eType, true);
        // Welcome message
        if (check)
        {
            if (eType == CRTPacket::PacketError)
            {

                Disconnect();
                printf("got error string...\n");
                return false;
            }
            if (eType == CRTPacket::PacketCommand)
            {

        printf("%s\n",mpoRTPacket->GetCommandString());
                if (strncmp("QTM RT Interface connected", mpoRTPacket->GetCommandString(), sizeof("QTM RT Interface connected")) == 0)
                {
                    // Set protocol version
                    if (SetVersion(mnMajorVersion, mnMinorVersion))
                    {
                        // Set byte order.
                        // Unless we use protocol version 1.0, we have set the byte order by selecting the correct port.

                        if ((mnMajorVersion == 1) && (mnMinorVersion == 0))
                        {
                            if (mbBigEndian)
                            {
                                sprintf(tTemp, "ByteOrder BigEndian");
                            }
                            else
                            {
                                sprintf(tTemp, "ByteOrder LittleEndian");
                            }

                            if (SendCommand(tTemp, pResponseStr, sizeof(pResponseStr)))
                            {
                                return true;
                            }
                            else
                            {
                //changed sprintf_s to snprintf for Linux -------------------
                 snprintf(maErrorStr, sizeof(maErrorStr), "Set byte order failed.");
                            }
                        }
                        else
                        {
                            GetState(meLastEvent, true);
                            return true;
                        }
                    }
                    Disconnect();
                    return false;
                }
            }
        }
        //changed sprintf_s to snprintf for Linux -------------------
        snprintf(maErrorStr, sizeof(maErrorStr), "Missing QTM server welcome message.");
        Disconnect();
    }
    else
    {
        if (mpoNetwork->GetError() == 10061)
        {
            snprintf(maErrorStr, sizeof(maErrorStr), "Check if QTM is running on target machine.");
        }
        else
        {
      //changed sprintf_s to snprintf for Linux -------------------
            snprintf(maErrorStr, sizeof(maErrorStr), "%s", mpoNetwork->GetErrorString());
            //-----------------------------------------------------------------------------
        }
    }
    return false;
} // Connect


void CRTProtocol::Disconnect()
{
    mpoNetwork->Disconnect();
    bBroadcastSocketCreated = false;
    if (mpoRTPacket)
    {
        delete mpoRTPacket;
        mpoRTPacket = NULL;
    }
} // Disconnect


bool CRTProtocol::Connected()
{
    return mpoNetwork->Connected();
}


bool CRTProtocol::SetVersion(int nMajorVersion, int nMinorVersion)
{
    char tTemp[256];
    char pResponseStr[256];

    sprintf(tTemp, "Version %u.%u", nMajorVersion, nMinorVersion);

    if (SendCommand(tTemp, pResponseStr, sizeof(pResponseStr)))
    {
        sprintf(tTemp, "Version set to %u.%u", mnMajorVersion, mnMinorVersion);
        printf ("%s\n", tTemp);

        if (strcmp(pResponseStr, tTemp) == 0)
        {
            return true;
        }

        if (strlen(pResponseStr))
        {
            snprintf(maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
        }
        else
        {
            snprintf(maErrorStr, sizeof(maErrorStr), "Set Version failed.");
        }
    }
    else
    {
        strncpy(tTemp, maErrorStr, sizeof(tTemp));
        snprintf(maErrorStr, sizeof(maErrorStr), "Send Version failed. %s.", tTemp);
    }
    return false;
}


bool CRTProtocol::GetVersion(int &nMajorVersion, int &nMinorVersion)
{
    char pResponseStr[256];

    if (SendCommand("Version", pResponseStr, sizeof(pResponseStr)))
    {
        if (sscanf(pResponseStr, "Version is %d.%d", &nMajorVersion, &nMinorVersion) == 2)
        {
            return true;
        }
    }
    snprintf(maErrorStr, sizeof(maErrorStr), "Get Version failed.");
    return false;
}


bool CRTProtocol::GetQTMVersion(char* pVersion, unsigned int nVersionLen)
{
    if (SendCommand("QTMVersion", pVersion, nVersionLen))
    {
        return true;
    }
    snprintf(maErrorStr, sizeof(maErrorStr), "Get QTM Version failed.");
    return false;
}


bool CRTProtocol::GetByteOrder(bool &bBigEndian)
{
    char pResponseStr[256];

    if (SendCommand("ByteOrder", pResponseStr, sizeof(pResponseStr)))
    {
        bBigEndian = (strcmp(pResponseStr, "Byte order is big endian") == 0);
        return true;
    }
    snprintf(maErrorStr, sizeof(maErrorStr), "Get Byte order failed.");
    return false;
}


bool CRTProtocol::CheckLicense(char* pLicenseCode)
{
    char tTemp[100];
    char pResponseStr[256];

    if (strlen(pLicenseCode) <= 85)
    {
        sprintf(tTemp, "CheckLicense %s", pLicenseCode);

        if (SendCommand(tTemp, pResponseStr, sizeof(pResponseStr)))
        {
            if (strcmp(pResponseStr, "License pass") == 0)
            {
                return true;
            }
            snprintf(maErrorStr, sizeof(maErrorStr), "Wrong license code.");
        }
        else
        {
            snprintf(maErrorStr, sizeof(maErrorStr), "CheckLicense failed.");
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "License code too long");
    }
    return false;
}


bool CRTProtocol::DiscoverRTServer(short nServerPort, bool bNoLocalResponses, short nDiscoverPort)
{
    //char pData[10];
    SDiscoverResponse sResponse;
    union PData{
      char data_char[10];
      struct DataInt{
        unsigned int first;
        unsigned int second;
        unsigned short third;
      }data_int;
    }pData;

    //*((unsigned int*)pData)         = (unsigned int)10;
    //*((unsigned int*)(pData + 4))   = (unsigned int)CRTPacket::PacketDiscover;
    //*((unsigned short*)(pData + 8)) = htons(nServerPort);
    pData.data_int.first  = (unsigned int)10;
    pData.data_int.second = (unsigned int)CRTPacket::PacketDiscover;
    pData.data_int.third  = htons(nServerPort);

    if (bBroadcastSocketCreated || mpoNetwork->CreateUDPSocket(nServerPort, true))
    {
        bBroadcastSocketCreated = true;
        //if (mpoNetwork->SendUDPBroadcast(pData, 10, nDiscoverPort))
        if (mpoNetwork->SendUDPBroadcast(pData.data_char, 10, nDiscoverPort))
        {
            int          nReceived;
            unsigned int nAddr;

            mvsDiscoverResponseList.clear();

            do
            {
                nReceived = mpoNetwork->Receive(maDataBuff, sizeof(maDataBuff), false, 100000, &nAddr);
                if (nReceived != -1 && nReceived > 8)
                {
                    char* pResponseStr;

                    if (CRTPacket::GetType(maDataBuff) == CRTPacket::PacketCommand)
                    {
                        pResponseStr        = CRTPacket::GetCommandString(maDataBuff);
                        sResponse.nAddr     = nAddr;
                        sResponse.nBasePort = CRTPacket::GetDiscoverResponseBasePort(maDataBuff);

                        if (pResponseStr && (!bNoLocalResponses || !mpoNetwork->IsLocalAddress(nAddr)))
                        {
                            strncpy(sResponse.pMessage, pResponseStr, sizeof(sResponse.pMessage));
                            mvsDiscoverResponseList.push_back(sResponse);
                        }
                    }
                }
            } while (nReceived != -1 && nReceived > 8); // Keep reading until no more responses.
            return true;
        }
    }
    return false;
}


int CRTProtocol::GetNumberOfDiscoverResponses()
{
    return mvsDiscoverResponseList.size();
}


bool CRTProtocol::GetDiscoverResponse(unsigned int nIndex, unsigned int &nAddr, unsigned short &nBasePort,
                                      bool &bLocalResponse, char* pMessage, int nMessageLen)
{
    if (nIndex < mvsDiscoverResponseList.size())
    {
        nAddr     = mvsDiscoverResponseList[nIndex].nAddr;
        nBasePort = mvsDiscoverResponseList[nIndex].nBasePort;
        strncpy(pMessage, mvsDiscoverResponseList[nIndex].pMessage, nMessageLen);
        return true;
    }
    return false;
}


bool CRTProtocol::GetCurrentFrame(EComponentType eComponentType)
{
    char pCommandStr[256];

    snprintf(pCommandStr, sizeof(pCommandStr), "GetCurrentFrame ");

    if (GetComponentString(pCommandStr + strlen(pCommandStr), sizeof(pCommandStr) - strlen(pCommandStr), eComponentType))
    {
        if (SendCommand(pCommandStr))
        {
            return true;
        }
        snprintf(maErrorStr, sizeof(maErrorStr), "GetCurrentFrame failed.");
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "DataComponent missing.");
    }
    return false;
}


bool CRTProtocol::StreamFrames(EStreamRate eRate, unsigned int nRateArg, unsigned short nUDPPort, const char* pUDPAddr,
                               EComponentType eComponentType)
{
    char pCommandStr[256];

    if (eRate == RateFrequencyDivisor)
    {
        snprintf(pCommandStr, sizeof(pCommandStr), "StreamFrames FrequencyDivisor:%d ", nRateArg);
    }
    else if (eRate == RateFrequency)
    {
        snprintf(pCommandStr, sizeof(pCommandStr), "StreamFrames Frequency:%d ", nRateArg);
    }
    else if (eRate == RateAllFrames)
    {
        snprintf(pCommandStr, sizeof(pCommandStr), "StreamFrames AllFrames ");
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "No valid rate.");
        return false;
    }
    if (nUDPPort > 0)
    {
        if (pUDPAddr != NULL && strlen(pUDPAddr) > 64)
        {
            snprintf(maErrorStr, sizeof(maErrorStr), "UDP address string too long.");
            return false;
        }
        snprintf(pCommandStr, sizeof(pCommandStr), "%s UDP%s%s:%d ",
                 pCommandStr, pUDPAddr != NULL ? ":" : "", pUDPAddr != NULL ? pUDPAddr : "", nUDPPort);
    }

    if (GetComponentString(pCommandStr + strlen(pCommandStr), sizeof(pCommandStr) - strlen(pCommandStr), eComponentType))
    {
        if (SendCommand(pCommandStr))
        {
            return true;
        }
        snprintf(maErrorStr, sizeof(maErrorStr), "StreamFrames failed.");
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "DataComponent missing.");
    }

    return false;
}


bool CRTProtocol::StreamFramesStop()
{
    if (SendCommand("StreamFrames Stop"))
    {
        return true;
    }
    snprintf(maErrorStr, sizeof(maErrorStr), "StreamFrames Stop failed.");
    return false;
}


bool CRTProtocol::GetState(CRTPacket::EEvent &eEvent, bool bUpdate)
{
    CRTPacket::EPacketType eType;

    if (bUpdate)
    {
        if (SendCommand("GetLastEvent"))
        {
            do
            {
                if (ReceiveRTPacket(eType, false) == false)
                {
                    break;
                }
                if (mpoRTPacket->GetEvent(eEvent))
                {
                    return true;
                }
            } while (1);
        }
        snprintf(maErrorStr, sizeof(maErrorStr), "GetLastEvent failed.");
    }
    else
    {
        eEvent = meLastEvent;
        return true;
    }
    return false;
}


bool CRTProtocol::GetCapture(char* pFileName, bool bC3D)
{
    CRTPacket::EPacketType eType;

    mpFileBuffer = fopen(pFileName, "wb");

    if (mpFileBuffer != NULL)
    {
        if (bC3D)
        {
            // C3D file
            if (SendCommand((mnMajorVersion > 1 || mnMinorVersion > 7) ? "GetCaptureC3D" : "GetCapture"))
            {
                if (ReceiveRTPacket(eType, true, 5000000)) // Wait for C3D file in 5 seconds.
                {
                    if (eType == CRTPacket::PacketC3DFile)
                    {
                        if (mpFileBuffer != NULL)
                        {
                            fclose(mpFileBuffer);
                            return true;
                        }
                        snprintf(maErrorStr, sizeof(maErrorStr), "Writing C3D file failed.");
                    }
                    else
                    {
                        snprintf(maErrorStr, sizeof(maErrorStr), "Wrong packet type received.");
                    }
                }
                else
                {
                    snprintf(maErrorStr, sizeof(maErrorStr), "No packet received.");
                }
            }
            else
            {
                snprintf(maErrorStr, sizeof(maErrorStr), "%s failed.",
                    (mnMajorVersion > 1 || mnMinorVersion > 7) ? "GetCaptureC3D" : "GetCapture");
            }
        }
        else
        {
            // QTM file
            if (SendCommand("GetCaptureQTM"))
            {
                if (ReceiveRTPacket(eType, true, 5000000)) // Wait for QTM file in 5 seconds.
                {
                    if (eType == CRTPacket::PacketQTMFile)
                    {
                        if (mpFileBuffer != NULL)
                        {
                            fclose(mpFileBuffer);
                            return true;
                        }
                        snprintf(maErrorStr, sizeof(maErrorStr), "Writing QTM file failed.");
                    }
                    else
                    {
                        snprintf(maErrorStr, sizeof(maErrorStr), "Wrong packet type received.");
                    }
                }
                else
                {
                    snprintf(maErrorStr, sizeof(maErrorStr), "No packet received. %s.", maErrorStr);
                }
            }
            else
            {
                snprintf(maErrorStr, sizeof(maErrorStr), "GetCaptureQTM failed.");
            }
        }
    }
    fclose(mpFileBuffer);

    return false;
}


bool CRTProtocol::SendTrig()
{
    char pResponseStr[256];

    if (SendCommand("Trig", pResponseStr, sizeof(pResponseStr)))
    {
        if (strcmp(pResponseStr, "Trig ok") == 0)
        {
            return true;
        }
    }
    if (strlen(pResponseStr))
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "Trig failed.");
    }
    return false;
}


bool CRTProtocol::SetQTMEvent(char* pLabel)
{
    char tTemp[100];
    char pResponseStr[256];

    if (strlen(pLabel) <= 92)
    {
        sprintf(tTemp, "%s %s",
                (mnMajorVersion > 1 || mnMinorVersion > 7) ? "SetQTMEvent" : "Event", pLabel);

        if (SendCommand(tTemp, pResponseStr, sizeof(pResponseStr)))
        {
            if (strcmp(pResponseStr, "Event set") == 0)
            {
                return true;
            }
        }
        if (strlen(pResponseStr))
        {
            snprintf (maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
        }
        else
        {
            snprintf(maErrorStr, sizeof(maErrorStr), "%s failed.",
                      (mnMajorVersion > 1 || mnMinorVersion > 7) ? "SetQTMEvent" : "Event");
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "Event label too long.");
    }
    return false;
}


bool CRTProtocol::TakeControl(char* pPassword)
{
    char pResponseStr[256];
    char pCmd[64];

    snprintf(pCmd, sizeof(pCmd), "TakeControl");
    if (pPassword != NULL)
    {
        // Add password
        if (pPassword[0] != 0)
        {
            strncat(pCmd, " ", sizeof(pCmd));
            //strncat(pCmd, pPassword, sizeof(pCmd));
            strncat(pCmd, pPassword, strlen(pPassword));
        }
    }
    if (SendCommand(pCmd, pResponseStr, sizeof(pResponseStr)))
    {
        if (strcmp("You are now master", pResponseStr)     == 0 ||
            strcmp("You are already master", pResponseStr) == 0)
        {
            return true;
        }
    }
    if (strlen(pResponseStr))
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
    }
    else
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "TakeControl failed.");
    }
    return false;
} // TakeControl


bool CRTProtocol::ReleaseControl()
{
    char pResponseStr[256];

    if (SendCommand("ReleaseControl", pResponseStr, sizeof(pResponseStr)))
    {
        if (strcmp("You are now a regular client", pResponseStr)     == 0 ||
            strcmp("You are already a regular client", pResponseStr) == 0)
        {
            return true;
        }
    }
    if (strlen(pResponseStr))
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
    }
    else
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "ReleaseControl failed.");
    }
    return false;
} // ReleaseControl


bool CRTProtocol::NewMeasurement()
{
    char pResponseStr[256];

    if (SendCommand("New", pResponseStr, sizeof(pResponseStr)))
    {
        if (strcmp(pResponseStr, "Creating new connection") == 0 ||
            strcmp(pResponseStr, "Already connected") == 0)
        {
            return true;
        }
    }
    if (strlen(pResponseStr))
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
    }
    else
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "New failed.");
    }
    return false;
}

bool CRTProtocol::CloseMeasurement()
{
    char pResponseStr[256];

    if (SendCommand("Close", pResponseStr, sizeof(pResponseStr)))
    {
        if (strcmp(pResponseStr, "Closing connection") == 0 ||
            strcmp(pResponseStr, "No connection to close") == 0)
        {
            return true;
        }
    }
    if (strlen(pResponseStr))
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
    }
    else
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "Close failed.");
    }
    return false;
}


bool CRTProtocol::StartCapture()
{
    char pResponseStr[256];

    if (SendCommand("Start", pResponseStr, sizeof(pResponseStr)))
    {
        if (strcmp(pResponseStr, "Starting measurement") == 0)
        {
            return true;
        }
    }
    if (strlen(pResponseStr))
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
    }
    else
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "Start failed.");
    }
    return false;
}


bool CRTProtocol::StopCapture()
{
    char pResponseStr[256];

    if (SendCommand("Stop", pResponseStr, sizeof(pResponseStr)))
    {
        if (strcmp(pResponseStr, "Stopping measurement") == 0)
        {
            return true;
        }
    }
    if (strlen(pResponseStr))
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
    }
    else
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "Stop failed.");
    }
    return false;
}


bool CRTProtocol::SaveCapture(char* pFileName, bool bOverwrite)
{
    char  tTemp[100];
    char pResponseStr[256];

    if (strlen(pFileName) <= 94)
    {
        sprintf(tTemp, "Save %s%s", pFileName, bOverwrite ? " Overwrite" : "");

        if (SendCommand(tTemp, pResponseStr, sizeof(pResponseStr)))
        {
            if (strcmp(pResponseStr, "Measurement saved") == 0)
            {
                return true;
            }
        }
        if (strlen(pResponseStr))
        {
            snprintf (maErrorStr, sizeof(maErrorStr), "%s.", pResponseStr);
        }
        else
        {
            snprintf (maErrorStr, sizeof(maErrorStr), "Save failed.");
        }
    }
    else
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "File name too long.");
    }
    return false;
}


CRTPacket* CRTProtocol::ReceiveRTPacket(CRTPacket::EPacketType &eType, bool bSkipEvents, int nTimeout)
{
    int          nRecved      = 0;
    unsigned int nRecvedTotal = 0;
    unsigned int nFrameSize;

    eType = CRTPacket::PacketNone;

    do
    {
        nRecved      = 0;
        nRecvedTotal = 0;

        nRecved = mpoNetwork->Receive(maDataBuff, sizeof(maDataBuff), true, nTimeout);
        //printf("%d",nRecved);
        if (nRecved < static_cast<int>(sizeof(int) * 2))
        {
            // QTM header not received.
            printf ("Couldn't read header bytes.");
            return NULL;
        }
        if (nRecved == -1)
        {
            if (mpoNetwork->Connected() == false)
            {
                printf("Disconnected from server.");
            }
            else
            {
                printf ("Socket Error.");
            }
            return NULL;
        }
        nRecvedTotal += nRecved;

        nFrameSize = mpoRTPacket->GetSize(maDataBuff);
        eType      = mpoRTPacket->GetType(maDataBuff);
        //printf ("%s", maDataBuff);
        unsigned int nReadSize;
    //printf("eType: %d\n", eType);
        if (eType == CRTPacket::PacketC3DFile || eType == CRTPacket::PacketQTMFile)
        {
            if (mpFileBuffer != NULL)
            {
                rewind(mpFileBuffer); // Start from the beginning
                if (fwrite(maDataBuff + sizeof(int) * 2, 1, nRecvedTotal - sizeof(int) * 2, mpFileBuffer) !=
                    nRecvedTotal - sizeof(int) * 2)
                {
                    printf ("Failed to write file to disk.");
                    fclose(mpFileBuffer);
                    mpFileBuffer = NULL;
                    return NULL;
                }
                // Receive more data until we have read the whole packet
                while (nRecvedTotal < nFrameSize)
                {
                    nReadSize = nFrameSize - nRecvedTotal;
                    if (nFrameSize > sizeof(maDataBuff))
                    {
                        nReadSize = sizeof(maDataBuff);
                    }
                    // As long as we haven't received enough data, wait for more
                    nRecved = mpoNetwork->Receive(&(maDataBuff[sizeof(int) * 2]), nReadSize, false, nTimeout);
                    if (nRecved == -1 || nRecved == 0)
                    {
                        if (mpoNetwork->Connected() == false)
                        {
                            printf("Disconnected from server.");
                        }
                        else
                        {
                            printf ("Socket Error.");
                        }
                        fclose(mpFileBuffer);
                        mpFileBuffer = NULL;
                        return NULL;
                    }
                    if (static_cast<int>(fwrite(maDataBuff + sizeof(int) * 2, 1, nRecved, mpFileBuffer)) != nRecved)
                    {
                        printf ("Failed to write file to disk.");
                        fclose(mpFileBuffer);
                        mpFileBuffer = NULL;
                        return NULL;
                    }
                    nRecvedTotal += nRecved;
                }
            }
            else
            {
                printf ( "Receive file buffer not opened.");
                fclose(mpFileBuffer);
                mpFileBuffer = NULL;
                return NULL;
            }
        }
        else
        {
            if (nFrameSize > sizeof(maDataBuff))
            {
                printf ("Receive buffer overflow. %d= %d > %d ", nFrameSize,nRecvedTotal,  static_cast<int>(sizeof(maDataBuff)));
                return NULL;
            }

            // Receive more data until we have read the whole packet
            while (nRecvedTotal < nFrameSize)
            {
                // As long as we haven't received enough data, wait for more
               // printf("second recieve\n");
                 //nRecved = mpoNetwork->Receive(maDataBuff, sizeof(maDataBuff), true, nTimeout);
                 //printf(" fsize %d recvd %d ",nFrameSize, nRecvedTotal); //(&(maDataBuff[nRecvedTotal])
                nRecved = mpoNetwork->Receive(&(maDataBuff[nRecvedTotal]), nFrameSize - nRecvedTotal, false, nTimeout);
                //printf ("bla %s", maDataBuff);
                if (nRecved == -1 || nRecved == 0)
                {
                    if (mpoNetwork->Connected() == false)
                    {
                        printf("Disconnected from server.");
                    }
                    else
                    {
                        printf ("Socket Error.");
                    }
                    return NULL;
                }
                nRecvedTotal += nRecved;
            }
        }
        //printf ("%s", maDataBuff);
    //printf ("SetData");
        mpoRTPacket->SetData(maDataBuff);
        mpoRTPacket->GetEvent(meLastEvent); // Update last event if there is an event
        //printf ("SetData done \n");

    } while (bSkipEvents && eType == CRTPacket::PacketEvent);

    if (nRecvedTotal == nFrameSize)
    {
        return mpoRTPacket;
    }
    printf( "Packet truncated.");

    return NULL;
} // ReceiveRTPacket


CRTPacket* CRTProtocol::GetRTPacket()
{
    return mpoRTPacket;
};


bool CRTProtocol::ReadGeneralSettings()
{
    CRTPacket::EPacketType  eType;
    CMarkup             oXML;
    bool                    bReturn = false;
    CStdStringA             tStr;

    msGeneralSettings.vsCameras.clear();

    if (SendCommand("GetParameters General"))
    {
        if (ReceiveRTPacket(eType, true))
        {
            if (eType == CRTPacket::PacketXML)
            {
                oXML.SetDoc(mpoRTPacket->GetXMLString());

                if (oXML.FindChildElem("General") && oXML.IntoElem())
                {
                    if (oXML.FindChildElem("Frequency"))
                    {
            msGeneralSettings.nCaptureFrequency = atoi(oXML.GetChildData().c_str());
                        if (oXML.FindChildElem("Capture_Time"))
                        {
                            msGeneralSettings.fCaptureTime = (float)atof(oXML.GetChildData().c_str());
                            if (oXML.FindChildElem("Start_On_External_Trigger"))
                            {
                                msGeneralSettings.bStartOnExternalTrigger = (strncmp (oXML.GetChildData().c_str(),"true",4) == 0);
                                bReturn = true;
                           }
                        }
                    }

                    if (bReturn && oXML.FindChildElem("External_Time_Base") && oXML.IntoElem())
                    {
                        if (oXML.FindChildElem("Enabled"))
                        {
                            msGeneralSettings.sExternalTimebase.bEnabled = (strncmp (oXML.GetChildData().c_str(),"true",4) == 0);
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Signal_Source"))
                        {
                            tStr = oXML.GetChildData();
                            if (tStr.CompareNoCase("control port") == 0)
                            {
                                msGeneralSettings.sExternalTimebase.eSignalSource = SourceControlPort;
                            }
                            else if (tStr.CompareNoCase("ir receiver") == 0)
                            {
                                msGeneralSettings.sExternalTimebase.eSignalSource = SourceIRReceiver;
                            }
                            else if (tStr.CompareNoCase("smpte") == 0)
                            {
                                msGeneralSettings.sExternalTimebase.eSignalSource = SourceSMPTE;
                            }
                            else if (tStr.CompareNoCase("video sync") == 0)
                            {
                                msGeneralSettings.sExternalTimebase.eSignalSource = SourceVideoSync;
                            }
                            else
                            {
                                bReturn = false;
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Signal_Mode"))
                        {
                            tStr = oXML.GetChildData();
                            if (tStr.CompareNoCase("periodic") == 0)
                            {
                                msGeneralSettings.sExternalTimebase.bSignalModePeriodic = true;
                            }
                            else if (tStr.CompareNoCase("non-periodic") == 0)
                            {
                                msGeneralSettings.sExternalTimebase.bSignalModePeriodic = false;
                            }
                            else
                            {
                                bReturn = false;
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Frequency_Multiplier"))
                        {
                            unsigned int nMultiplier;
                            tStr = oXML.GetChildData();
                            if (sscanf(tStr.GetBuffer(), "%d", &nMultiplier) == 1)
                            {
                                msGeneralSettings.sExternalTimebase.nFreqMultiplier = nMultiplier;
                            }
                            else
                            {
                                bReturn = false;
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Frequency_Divisor"))
                        {
                            unsigned int nDivisor;
                            tStr = oXML.GetChildData();
                            if (sscanf(tStr.GetBuffer(), "%d", &nDivisor) == 1)
                            {
                                msGeneralSettings.sExternalTimebase.nFreqDivisor = nDivisor;
                            }
                            else
                            {
                                bReturn = false;
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Frequency_Tolerance"))
                        {
                            unsigned int nTolerance;
                            tStr = oXML.GetChildData();
                            if (sscanf(tStr.GetBuffer(), "%d", &nTolerance) == 1)
                            {
                                msGeneralSettings.sExternalTimebase.nFreqTolerance = nTolerance;
                            }
                            else
                            {
                                bReturn = false;
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Nominal_Frequency"))
                        {
                            tStr = oXML.GetChildData();
                            if (tStr.CompareNoCase("none") == 0)
                            {
                                msGeneralSettings.sExternalTimebase.fNominalFrequency = -1; // -1 = disabled
                            }
                            else
                            {
                                float fFrequency;
                                if (sscanf(tStr.GetBuffer(), "%f", &fFrequency) == 1)
                                {
                                    msGeneralSettings.sExternalTimebase.fNominalFrequency = fFrequency;
                                }
                                else
                                {
                                    bReturn = false;
                                }
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Signal_Edge"))
                        {
                            tStr = oXML.GetChildData();
                            if (tStr.CompareNoCase("negative") == 0)
                            {
                                msGeneralSettings.sExternalTimebase.bNegativeEdge = true;
                            }
                            else if (tStr.CompareNoCase("positive") == 0)
                            {
                                msGeneralSettings.sExternalTimebase.bNegativeEdge = false;
                            }
                            else
                            {
                                bReturn = false;
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Signal_Shutter_Delay"))
                        {
                            unsigned int nDelay;
                            tStr = oXML.GetChildData();
                            if (sscanf(tStr.GetBuffer(), "%d", &nDelay) == 1)
                            {
                                msGeneralSettings.sExternalTimebase.nSignalShutterDelay = nDelay;
                            }
                            else
                            {
                                bReturn = false;
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Non_Periodic_Timeout"))
                        {
                            float fTimeout;
                            tStr = oXML.GetChildData();
                            if (sscanf(tStr.GetBuffer(), "%f", &fTimeout) == 1)
                            {
                                msGeneralSettings.sExternalTimebase.fNonPeriodicTimeout = fTimeout;
                            }
                            else
                            {
                                bReturn = false;
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        oXML.OutOfElem(); // External_Time_Base
                    }

                    if (bReturn && oXML.FindChildElem("Processing_Actions") && oXML.IntoElem())
                    {
                        msGeneralSettings.eProcessingActions = ProcessingNone;

                        if (oXML.FindChildElem("Tracking"))
                        {
                            tStr = oXML.GetChildData();
                            if (tStr.CompareNoCase("3d") == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingTracking3D);
                            }
                            else if (tStr.CompareNoCase("2d") == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingTracking2D);
                            }
                            else if (tStr.CompareNoCase("false") != 0)
                            {
                                bReturn = false;
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("TwinSystemMerge"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingTwinSystemMerge);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("SplineFill"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingSplineFill);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("AIM"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingAIM);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Track6DOF"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + Processing6DOFTracking);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("ForceData"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingForceData);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("ExportTSV"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingExportTSV);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("ExportC3D"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingExportC3D);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("ExportDiff"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingExportDiff);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("ExportMatlabDirect"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingExportMatlabDirect);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("ExportMatlabFile"))
                        {
                            if (strncmp (oXML.GetChildData().c_str(),"true",4) == 0)
                            {
                                msGeneralSettings.eProcessingActions = (EProcessingActions)(msGeneralSettings.eProcessingActions + ProcessingExportMatlabFile);
                            }
                        }
                        else
                        {
                            bReturn = false;
                        }

                        oXML.OutOfElem(); // Processing_Actions
                    }

                    SSettingsGeneralCamera sCameraSettings;

                    while (bReturn && oXML.FindChildElem("Camera") && oXML.IntoElem())
                    {
                        if (oXML.FindChildElem("ID"))
                        {
                            sCameraSettings.nID = atoi(oXML.GetChildData().c_str());
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Model"))
                        {
                            tStr = oXML.GetChildData();

                            if (tStr.CompareNoCase("MacReflex") == 0)
                            {
                                sCameraSettings.eModel = ModelMacReflex;
                            }
                            else if (tStr.CompareNoCase("ProReflex 120") == 0)
                            {
                                sCameraSettings.eModel = ModelProReflex120;
                            }
                            else if (tStr.CompareNoCase("ProReflex 240") == 0)
                            {
                                sCameraSettings.eModel = ModelProReflex240;
                            }
                            else if (tStr.CompareNoCase("ProReflex 500") == 0)
                            {
                                sCameraSettings.eModel = ModelProReflex500;
                            }
                            else if (tStr.CompareNoCase("ProReflex 1000") == 0)
                            {
                                sCameraSettings.eModel = ModelProReflex1000;
                            }
                            else if (tStr.CompareNoCase("Oqus 100") == 0)
                            {
                                sCameraSettings.eModel = ModelOqus100;
                            }
                            else if (tStr.CompareNoCase("Oqus 200") == 0)
                            {
                                sCameraSettings.eModel = ModelOqus200;
                            }
                            else if (tStr.CompareNoCase("Oqus 300") == 0)
                            {
                                sCameraSettings.eModel = ModelOqus300;
                            }
                            else if (tStr.CompareNoCase("Oqus 300 Plus") == 0)
                            {
                                sCameraSettings.eModel = ModelOqus300Plus;
                            }
                            else if (tStr.CompareNoCase("Oqus 400") == 0)
                            {
                                sCameraSettings.eModel = ModelOqus400;
                            }
                            else if (tStr.CompareNoCase("Oqus 500") == 0)
                            {
                                sCameraSettings.eModel = ModelOqus500;
                            }
                            else if (tStr.CompareNoCase("Oqus 500 Plus") == 0)
                            {
                                sCameraSettings.eModel = ModelOqus500Plus;
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        // Only available from protocol version 1.10 and later.
                        if (oXML.FindChildElem("Underwater"))
                        {
                            sCameraSettings.bUnderwater = (strncmp (oXML.GetChildData().c_str(),"true",4) == 0);
                        }

                        if (oXML.FindChildElem("Serial"))
                        {
                            sCameraSettings.nSerial = atoi(oXML.GetChildData().c_str());
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Mode"))
                        {
                            tStr = oXML.GetChildData();

                            if (tStr.CompareNoCase("Marker") == 0)
                            {
                                sCameraSettings.eMode = ModeMarker;
                            }
                            else if (tStr.CompareNoCase("Marker Intensity") == 0)
                            {
                                sCameraSettings.eMode = ModeMarkerIntensity;
                            }
                            else if (tStr.CompareNoCase("Video") == 0)
                            {
                                sCameraSettings.eMode = ModeVideo;
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Video_Exposure") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("Current"))
                            {
                                sCameraSettings.nVideoExposure = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Min"))
                            {
                                sCameraSettings.nVideoExposureMin = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Max"))
                            {
                                sCameraSettings.nVideoExposureMax = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }
                            oXML.OutOfElem(); // Video_Exposure
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Video_Flash_Time") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("Current"))
                            {
                                sCameraSettings.nVideoFlashTime = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Min"))
                            {
                                sCameraSettings.nVideoFlashTimeMin = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Max"))
                            {
                                sCameraSettings.nVideoFlashTimeMax = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }
                            oXML.OutOfElem(); // Video_Flash_Time
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Marker_Exposure") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("Current"))
                            {
                                sCameraSettings.nMarkerExposure = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Min"))
                            {
                                sCameraSettings.nMarkerExposureMin = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Max"))
                            {
                                sCameraSettings.nMarkerExposureMax = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }
                            oXML.OutOfElem(); // Marker_Exposure
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Marker_Threshold") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("Current"))
                            {
                                sCameraSettings.nMarkerThreshold = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Min"))
                            {
                                sCameraSettings.nMarkerThresholdMin = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Max"))
                            {
                                sCameraSettings.nMarkerThresholdMax = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }
                            oXML.OutOfElem(); // Marker_Threshold
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Position") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("X"))
                            {
                                sCameraSettings.fPositionX = (float)atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Y"))
                            {
                                sCameraSettings.fPositionY = (float)atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Z"))
                            {
                                sCameraSettings.fPositionZ = (float)atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Rot_1_1"))
                            {
                                sCameraSettings.fPositionRotMatrix[0][0] = (float)atof(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Rot_2_1"))
                            {
                                sCameraSettings.fPositionRotMatrix[1][0] = (float)atof(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Rot_3_1"))
                            {
                                sCameraSettings.fPositionRotMatrix[2][0] = (float)atof(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Rot_1_2"))
                            {
                                sCameraSettings.fPositionRotMatrix[0][1] = (float)atof(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Rot_2_2"))
                            {
                                sCameraSettings.fPositionRotMatrix[1][1] = (float)atof(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Rot_3_2"))
                            {
                                sCameraSettings.fPositionRotMatrix[2][1] = (float)atof(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Rot_1_3"))
                            {
                                sCameraSettings.fPositionRotMatrix[0][2] = (float)atof(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Rot_2_3"))
                            {
                                sCameraSettings.fPositionRotMatrix[1][2] = (float)atof(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Rot_3_3"))
                            {
                                sCameraSettings.fPositionRotMatrix[2][2] = (float)atof(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            oXML.OutOfElem(); // Position
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Orientation"))
                        {
                            sCameraSettings.nOrientation = atoi(oXML.GetChildData().c_str());
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Marker_Res") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("Width"))
                            {
                                sCameraSettings.nMarkerResolutionWidth = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Height"))
                            {
                                sCameraSettings.nMarkerResolutionHeight = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            oXML.OutOfElem(); // Marker_Res
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Video_Res") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("Width"))
                            {
                                sCameraSettings.nVideoResolutionWidth = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Height"))
                            {
                                sCameraSettings.nVideoResolutionHeight = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            oXML.OutOfElem(); // Video_Res
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Marker_FOV") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("Left"))
                            {
                                sCameraSettings.nMarkerFOVLeft = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Top"))
                            {
                                sCameraSettings.nMarkerFOVTop = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Right"))
                            {
                                sCameraSettings.nMarkerFOVRight = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Bottom"))
                            {
                                sCameraSettings.nMarkerFOVBottom = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            oXML.OutOfElem(); // Marker_FOV
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        if (oXML.FindChildElem("Video_FOV") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("Left"))
                            {
                                sCameraSettings.nVideoFOVLeft = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Top"))
                            {
                                sCameraSettings.nVideoFOVTop = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Right"))
                            {
                                sCameraSettings.nVideoFOVRight = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (oXML.FindChildElem("Bottom"))
                            {
                                sCameraSettings.nVideoFOVBottom = atoi(oXML.GetChildData().c_str());
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            oXML.OutOfElem(); // Video_FOV
                        }
                        else
                        {
                            bReturn = false;
                            break;
                        }

                        // Only available from protocol version 1.10 and later.
                        if (oXML.FindChildElem("Sync_Out") && oXML.IntoElem())
                        {
                            if (oXML.FindChildElem("Mode"))
                            {
                                tStr = oXML.GetChildData();

                                if (tStr.CompareNoCase("Shutter out") == 0)
                                {
                                    sCameraSettings.eSyncOutMode = ModeShutterOut;
                                }
                                else if (tStr.CompareNoCase("Multiplier") == 0)
                                {
                                    sCameraSettings.eSyncOutMode = ModeMultiplier;
                                }
                                else if (tStr.CompareNoCase("Divisor") == 0)
                                {
                                    sCameraSettings.eSyncOutMode = ModeDivisor;
                                }
                                else if (tStr.CompareNoCase("Camera independent") == 0)
                                {
                                    sCameraSettings.eSyncOutMode = ModeActualFreq;
                                }
                                else if (tStr.CompareNoCase("Measurement time") == 0)
                                {
                                    sCameraSettings.eSyncOutMode = ModeMeasurementTime;
                                }
                                else if (tStr.CompareNoCase("SRAM wired") == 0)
                                {
                                    sCameraSettings.eSyncOutMode = ModeSRAMWireSync;
                                }
                                else if (tStr.CompareNoCase("Continuous 100Hz") == 0)
                                {
                                    sCameraSettings.eSyncOutMode = ModeFixed100Hz;
                                }
                                else
                                {
                                    bReturn = false;
                                    break;
                                }
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }

                            if (sCameraSettings.eSyncOutMode == ModeMultiplier ||
                                sCameraSettings.eSyncOutMode == ModeDivisor    ||
                                sCameraSettings.eSyncOutMode == ModeActualFreq)
                            {
                                if (oXML.FindChildElem("Value"))
                                {
                                    sCameraSettings.nSyncOutValue = atoi(oXML.GetChildData().c_str());
                                }
                                else
                                {
                                    bReturn = false;
                                    break;
                                }
                                if (oXML.FindChildElem("Duty_Cycle"))
                                {
                                    sCameraSettings.fSyncOutDutyCycle = (float)atof(oXML.GetChildData().c_str());
                                }
                                else
                                {
                                    bReturn = false;
                                    break;
                                }
                            }
                            else
                            {
                                sCameraSettings.nSyncOutValue     = 0;
                                sCameraSettings.fSyncOutDutyCycle = 0;
                            }

                            if (sCameraSettings.eSyncOutMode != ModeSRAMWireSync &&
                                sCameraSettings.eSyncOutMode != ModeFixed100Hz)
                            {
                                if (oXML.FindChildElem("Signal_Polarity"))
                                {
                                    sCameraSettings.bSyncOutNegativePolarity =
                                        (strncmp (oXML.GetChildData().c_str(),"negative",8) == 0);
                                }
                                else
                                {
                                    bReturn = false;
                                    break;
                                }
                            }
                            oXML.OutOfElem(); // Sync_Out
                        }
                        else
                        {
                            sCameraSettings.eSyncOutMode             = ModeActualFreq;
                            sCameraSettings.nSyncOutValue            = 0;
                            sCameraSettings.fSyncOutDutyCycle        = 0;
                            sCameraSettings.bSyncOutNegativePolarity = false;
                        }
                        oXML.OutOfElem(); // Camera

                        msGeneralSettings.vsCameras.push_back(sCameraSettings);
                    }
                }
            }
            else if (eType == CRTPacket::PacketError)
            {
                snprintf(maErrorStr, sizeof(maErrorStr), "%s.", mpoRTPacket->GetErrorString());
            }
            else
            {
                snprintf(maErrorStr, sizeof(maErrorStr),
                    "GetParameters General returned wrong packet type. Got type %d expected type 2.", eType);
            }
        }
        else
        {
            char pTmpStr[256];
            strncpy(pTmpStr, maErrorStr, sizeof(pTmpStr));
            snprintf(maErrorStr, sizeof(maErrorStr), "No response received. Expected XML packet. %s", pTmpStr);
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "GetParameters General failed");
    }

    return bReturn;
} // ReadGeneralSettings

bool CRTProtocol::Read3DSettings()
{
    CRTPacket::EPacketType  eType;
    CMarkup             oXML;
    bool                    bReturn = false;
    CStdStringA             oTmpStr;

    ms3DSettings.s3DLabels.clear();
    ms3DSettings.pCalibrationTime[0] = 0;

    if (SendCommand("GetParameters 3D"))
    {
        if (ReceiveRTPacket(eType, true))
        {
            if (eType == CRTPacket::PacketXML)
            {
                oXML.SetDoc(mpoRTPacket->GetXMLString());

                if (oXML.FindChildElem("The_3D") && oXML.IntoElem())
                {
                    bReturn = true;

                    if (oXML.FindChildElem("AxisUpwards"))
                    {
                        oTmpStr = oXML.GetChildData();

                        if (oTmpStr.CompareNoCase("+X") == 0)
                        {
                            ms3DSettings.eAxisUpwards = XPos;
                        }
                        else if (oTmpStr.CompareNoCase("-X") == 0)
                        {
                            ms3DSettings.eAxisUpwards = XNeg;
                        }
                        else if (oTmpStr.CompareNoCase("+Y") == 0)
                        {
                            ms3DSettings.eAxisUpwards = YPos;
                        }
                        else if (oTmpStr.CompareNoCase("-Y") == 0)
                        {
                            ms3DSettings.eAxisUpwards = YNeg;
                        }
                        else if (oTmpStr.CompareNoCase("+Z") == 0)
                        {
                            ms3DSettings.eAxisUpwards = ZPos;
                        }
                        else if (oTmpStr.CompareNoCase("-Z") == 0)
                        {
                            ms3DSettings.eAxisUpwards = ZNeg;
                        }
                        else
                        {
                            bReturn = false;
                        }
                    }

                    if (oXML.FindChildElem("CalibrationTime"))
                    {
                        oTmpStr = oXML.GetChildData();
                        strncpy(ms3DSettings.pCalibrationTime,
                                 oTmpStr.GetBuffer(), sizeof(ms3DSettings.pCalibrationTime));
                    }

                    if (oXML.FindChildElem("Labels"))
                    {
                        unsigned int nNumberOfLabels = atoi(oXML.GetChildData().c_str());

                        ms3DSettings.s3DLabels.resize(nNumberOfLabels);
                        SSettings3DLabel sLabel;

                        for (unsigned int iLabel = 0; iLabel < nNumberOfLabels; iLabel++)
                        {
                            if (oXML.FindChildElem("Label"))
                            {
                                oXML.IntoElem();
                                if (oXML.FindChildElem("Name"))
                                {
                                    sLabel.oName = oXML.GetChildData();
                                    if (oXML.FindChildElem("RGBColor"))
                                    {
                                        sLabel.nRGBColor = atoi(oXML.GetChildData().c_str());
                                    }
                                    ms3DSettings.s3DLabels[iLabel] = sLabel;
                                }
                                oXML.OutOfElem();
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                if (eType == CRTPacket::PacketError)
                {
                    snprintf(maErrorStr, sizeof(maErrorStr), "%s.", mpoRTPacket->GetErrorString());
                }
                else
                {
                    snprintf(maErrorStr, sizeof(maErrorStr),
                        "GetParameters 3D returned wrong packet type. Got type %d expected type 2.", eType);
                }
            }
        }
        else
        {
            char pTmpStr[256];
            strncpy(pTmpStr, maErrorStr, sizeof(pTmpStr));
            snprintf(maErrorStr, sizeof(maErrorStr), "No response received. Expected XML packet. %s", pTmpStr);
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "GetParameters 3D failed");
    }
    return bReturn;
} // Read3DSettings

bool CRTProtocol::Read6DOFSettings()
{
    CRTPacket::EPacketType  eType;
    CMarkup             oXML;
    bool                    bReturn = false;

    mvs6DOFBodySettings.clear();
    if (SendCommand("GetParameters 6D"))
    {
        if (ReceiveRTPacket(eType, true))
        {
            if (eType == CRTPacket::PacketXML)
            {
                oXML.SetDoc(mpoRTPacket->GetXMLString());
                //
                // Read 6DOF bodies
                //
                if (oXML.FindChildElem("The_6D") && oXML.IntoElem())
                {
                    bReturn = true;

                    if (oXML.FindChildElem("Bodies"))
                    {
                        int nBodies = atoi(oXML.GetChildData().c_str());
                        SSettings6DOF s6DBodySettings;
                        SPoint        sPoint;
                        s6DBodySettings.vsPoints.clear();

                        for (int iBody = 0; iBody < nBodies; iBody++)
                        {
                            if (oXML.FindChildElem("Body"))
                            {
                                CStdString sString;
                                oXML.IntoElem();
                                if (oXML.FindChildElem("Name"))
                                {
                                    s6DBodySettings.oName = oXML.GetChildData();
                                    if (oXML.FindChildElem("RGBColor"))
                                    {
                                        s6DBodySettings.vsPoints.clear();
                                        s6DBodySettings.nRGBColor = atoi(oXML.GetChildData().c_str());
                                        while (oXML.FindChildElem("Point"))
                                        {
                                            oXML.IntoElem();
                                            if (oXML.FindChildElem("X"))
                                            {

                                                sPoint.fX = (float)atof(oXML.GetChildData().c_str());
                                            }
                                            else
                                            {
                                                bReturn = false;
                                            }
                                            if (oXML.FindChildElem("Y"))
                                            {
                                                sPoint.fY = (float)atof(oXML.GetChildData().c_str());
                                            }
                                            else
                                            {
                                                bReturn = false;
                                            }
                                            if (oXML.FindChildElem("Z"))
                                            {
                                                sPoint.fZ = (float)atof(oXML.GetChildData().c_str());
                                            }
                                            else
                                            {
                                                bReturn = false;
                                            }
                                            oXML.OutOfElem(); // Point
                                            s6DBodySettings.vsPoints.push_back(sPoint);
                                        }
                                        mvs6DOFBodySettings.push_back(s6DBodySettings);
                                    }
                                    else
                                    {
                                        bReturn = false;
                                        break;
                                    }
                                }
                                else
                                {
                                    bReturn = false;
                                    break;
                                }
                                oXML.OutOfElem(); // Body
                            }
                            else
                            {
                                bReturn = false;
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                if (eType == CRTPacket::PacketError)
                {
                    snprintf(maErrorStr, sizeof(maErrorStr), "%s.", mpoRTPacket->GetErrorString());
                    printf("%s.", mpoRTPacket->GetErrorString());
                }
                else
                {
                    snprintf(maErrorStr, sizeof(maErrorStr),
                        "GetParameters 6DOF returned wrong packet type. Got type %d expected type 2.", eType);
                             printf("GetParameters 6DOF returned wrong packet type. Got type %d expected type 2.", eType);
                }
            }
        }
        else
        {
            char pTmpStr[256];
            strncpy(pTmpStr, maErrorStr, sizeof(pTmpStr));
            snprintf(maErrorStr, sizeof(maErrorStr), "No response received. Expected XML packet. %s", pTmpStr);
            printf( "No response received. Expected XML packet. %s", pTmpStr);
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "GetParameters 6D failed");
         printf( "GetParameters 6D failed");
    }
    //printf( "return %d\n", bReturn);
    return bReturn;
} // Read6DOFSettings

bool CRTProtocol::ReadAnalogSettings()
{
    CRTPacket::EPacketType  eType;
    CMarkup             oXML;
    bool                    bReturn = false;

    mvsAnalogDeviceSettings.clear();

    if (SendCommand("GetParameters Analog"))
    {
        if (ReceiveRTPacket(eType, true))
        {
            if (eType == CRTPacket::PacketXML)
            {
                oXML.SetDoc(mpoRTPacket->GetXMLString());

                if (oXML.FindChildElem("Analog"))
                {
                    SAnalogDevice sAnalogDevice;

                    oXML.IntoElem();
                    if (mnMajorVersion == 1 && mnMinorVersion == 0)
                    {
                        sAnalogDevice.nDeviceID = 1;   // Always channel 1
                        sAnalogDevice.oName     = "AnalogDevice";
                        if (oXML.FindChildElem("Channels"))
                        {
                            sAnalogDevice.nChannels  = atoi(oXML.GetChildData().c_str());
                            if (oXML.FindChildElem("Frequency"))
                            {
                                sAnalogDevice.nFrequency = atoi(oXML.GetChildData().c_str());
                                if (oXML.FindChildElem("Unit"))
                                {
                                    sAnalogDevice.oUnit = oXML.GetChildData();
                                    if (oXML.FindChildElem("Range"))
                                    {
                                        oXML.IntoElem();
                                        if (oXML.FindChildElem("Min"))
                                        {
                                            sAnalogDevice.nMinRange = atoi(oXML.GetChildData().c_str());
                                            if (oXML.FindChildElem("Max"))
                                            {
                                                sAnalogDevice.nMaxRange = atoi(oXML.GetChildData().c_str());
                                                mvsAnalogDeviceSettings.push_back(sAnalogDevice);
                                                bReturn = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        bReturn = true;
                        while (oXML.FindChildElem("Device"))
                        {
                            sAnalogDevice.voLabels.clear();
                            oXML.IntoElem();
                            if (oXML.FindChildElem("Device_ID"))
                            {
                                sAnalogDevice.nDeviceID = atoi(oXML.GetChildData().c_str());

                                if (oXML.FindChildElem("Device_Name"))
                                {
                                    sAnalogDevice.oName = oXML.GetChildData();

                                    if (oXML.FindChildElem("Channels"))
                                    {
                                        sAnalogDevice.nChannels = atoi(oXML.GetChildData().c_str());

                                        if (oXML.FindChildElem("Frequency"))
                                        {
                                            sAnalogDevice.nFrequency = atoi(oXML.GetChildData().c_str());

                                            if (oXML.FindChildElem("Unit"))
                                            {
                                                sAnalogDevice.oUnit      = oXML.GetChildData();

                                                if (oXML.FindChildElem("Range"))
                                                {
                                                    oXML.IntoElem();
                                                    if (oXML.FindChildElem("Min"))
                                                    {
                                                        sAnalogDevice.nMinRange = atoi(oXML.GetChildData().c_str());
                                                        if (oXML.FindChildElem("Max"))
                                                        {
                                                            sAnalogDevice.nMaxRange = atoi(oXML.GetChildData().c_str());

                                                            for (unsigned int i = 0; i < sAnalogDevice.nChannels; i++)
                                                            {
                                                                if (oXML.FindChildElem("Label"))
                                                                {
                                                                    sAnalogDevice.voLabels.push_back(oXML.GetChildData());
                                                                }
                                                            }
                                                            if (sAnalogDevice.voLabels.size() != sAnalogDevice.nChannels)
                                                            {
                                                                bReturn = false;
                                                            }
                                                        }
                                                    }
                                                    oXML.OutOfElem(); // Range
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            oXML.OutOfElem(); // Device
                            mvsAnalogDeviceSettings.push_back(sAnalogDevice);
                        }
                    }
                }
            }
            else
            {
                if (eType == CRTPacket::PacketError)
                {
                    snprintf(maErrorStr, sizeof(maErrorStr), "%s.", mpoRTPacket->GetErrorString());
                }
                else
                {
                    snprintf(maErrorStr, sizeof(maErrorStr),
                        "GetParameters Analog returned wrong packet type. Got type %d expected type 2.", eType);
                }
            }
        }
        else
        {
            char pTmpStr[256];
            strncpy(pTmpStr, maErrorStr, sizeof(pTmpStr));
            snprintf(maErrorStr, sizeof(maErrorStr), "No response received. Expected XML packet. %s", pTmpStr);
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "GetParameters Analog failed");
    }
    return bReturn;
} // ReadAnalogSettings

bool CRTProtocol::ReadForceSettings()
{
    CRTPacket::EPacketType  eType;
    CMarkup             oXML;
    bool                    bReturn = false;

    msForceSettings.vsForcePlates.clear();

    if (SendCommand("GetParameters Force"))
    {
        if (ReceiveRTPacket(eType, true))
        {
            if (eType == CRTPacket::PacketXML)
            {
                oXML.SetDoc(mpoRTPacket->GetXMLString());
                //
                // Read some force plate parameters
                //
                if (oXML.FindChildElem("Force") && oXML.IntoElem())
                {
                    SForcePlate sForcePlate;
                    sForcePlate.bValidCalibrationMatrix = false;

                    if (oXML.FindChildElem("Unit_Length"))
                    {
                        msForceSettings.oUnitLength = oXML.GetChildData();
                        bReturn = true;
                    }
                    if (oXML.FindChildElem("Unit_Force"))
                    {
                        msForceSettings.oUnitForce = oXML.GetChildData();
                        bReturn = true;
                    }

                    int  iPlate = 1;
                    bool bPlateExists = false;
                    while (oXML.FindChildElem("Plate"))
                    {
                        bReturn      = true;
                        bPlateExists = true;
                        //
                        // Get name and type of the plates
                        //
                        oXML.IntoElem(); // "Plate"
                        if (oXML.FindChildElem("Force_Plate_Index")) // Version 1.7 and earlier.
                        {
                            sForcePlate.nID = atoi(oXML.GetChildData().c_str());
                        }
                        else if (oXML.FindChildElem("Plate_ID")) // Version 1.8 and later.
                        {
                            sForcePlate.nID = atoi(oXML.GetChildData().c_str());
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Analog_Device_ID"))
                        {
                            sForcePlate.nAnalogDeviceID = atoi(oXML.GetChildData().c_str());
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Frequency"))
                        {
                            sForcePlate.nFrequency = atoi(oXML.GetChildData().c_str());
                        }
                        else
                        {
                            bReturn = false;
                        }

                        if (oXML.FindChildElem("Type"))
                        {
                            sForcePlate.oType = oXML.GetChildData();
                        }
                        else
                        {
                            sForcePlate.oType = "unknown";
                        }

                        if (oXML.FindChildElem("Name"))
                        {
                            sForcePlate.oName = oXML.GetChildData();
                        }
                        else
                        {
                            sForcePlate.oName.Format("#%d", iPlate);
                        }

                        if (oXML.FindChildElem("Length"))
                        {
                            sForcePlate.fLength = (float)atof(oXML.GetChildData().c_str());
                        }
                        if (oXML.FindChildElem("Width"))
                        {
                            sForcePlate.fWidth  = (float)atof(oXML.GetChildData().c_str());
                        }

                        if (oXML.FindChildElem("Location"))
                        {
                            oXML.IntoElem();
                            if (oXML.FindChildElem("Corner1"))
                            {
                                oXML.IntoElem();
                                if (oXML.FindChildElem("X"))
                                {
                                    sForcePlate.asCorner[0].fX = (float)atof(oXML.GetChildData().c_str());
                                }
                                if (oXML.FindChildElem("Y"))
                                {
                                    sForcePlate.asCorner[0].fY = (float)atof(oXML.GetChildData().c_str());
                                }
                                if (oXML.FindChildElem("Z"))
                                {
                                    sForcePlate.asCorner[0].fZ = (float)atof(oXML.GetChildData().c_str());
                                }
                                oXML.OutOfElem();
                            }
                            if (oXML.FindChildElem("Corner2"))
                            {
                                oXML.IntoElem();
                                if (oXML.FindChildElem("X"))
                                {
                                    sForcePlate.asCorner[1].fX = (float)atof(oXML.GetChildData().c_str());
                                }
                                if (oXML.FindChildElem("Y"))
                                {
                                    sForcePlate.asCorner[1].fY = (float)atof(oXML.GetChildData().c_str());
                                }
                                if (oXML.FindChildElem("Z"))
                                {
                                    sForcePlate.asCorner[1].fZ = (float)atof(oXML.GetChildData().c_str());
                                }
                                oXML.OutOfElem();
                            }
                            if (oXML.FindChildElem("Corner3"))
                            {
                                oXML.IntoElem();
                                if (oXML.FindChildElem("X"))
                                {
                                    sForcePlate.asCorner[2].fX = (float)atof(oXML.GetChildData().c_str());
                                }
                                if (oXML.FindChildElem("Y"))
                                {
                                    sForcePlate.asCorner[2].fY = (float)atof(oXML.GetChildData().c_str());
                                }
                                if (oXML.FindChildElem("Z"))
                                {
                                    sForcePlate.asCorner[2].fZ = (float)atof(oXML.GetChildData().c_str());
                                }
                                oXML.OutOfElem();
                            }
                            if (oXML.FindChildElem("Corner4"))
                            {
                                oXML.IntoElem();
                                if (oXML.FindChildElem("X"))
                                {
                                    sForcePlate.asCorner[3].fX = (float)atof(oXML.GetChildData().c_str());
                                }
                                if (oXML.FindChildElem("Y"))
                                {
                                    sForcePlate.asCorner[3].fY = (float)atof(oXML.GetChildData().c_str());
                                }
                                if (oXML.FindChildElem("Z"))
                                {
                                    sForcePlate.asCorner[3].fZ = (float)atof(oXML.GetChildData().c_str());
                                }
                                oXML.OutOfElem();
                            }
                            oXML.OutOfElem();
                        }
                        if (oXML.FindChildElem("Origin"))
                        {
                            oXML.IntoElem();
                            if (oXML.FindChildElem("X"))
                            {
                                sForcePlate.sOrigin.fX = (float)atof(oXML.GetChildData().c_str());
                            }
                            if (oXML.FindChildElem("Y"))
                            {
                                sForcePlate.sOrigin.fY = (float)atof(oXML.GetChildData().c_str());
                            }
                            if (oXML.FindChildElem("Z"))
                            {
                                sForcePlate.sOrigin.fZ = (float)atof(oXML.GetChildData().c_str());
                            }
                            oXML.OutOfElem();
                        }
                        if (oXML.FindChildElem("Channels"))
                        {
                            oXML.IntoElem();
                            sForcePlate.vChannels.clear();
                            SForceChannel sForceChannel;
                            while (oXML.FindChildElem("Channel"))
                            {
                                oXML.IntoElem();
                                if (oXML.FindChildElem("Channel_No"))
                                {
                                    sForceChannel.nChannelNumber = atoi(oXML.GetChildData().c_str());
                                }
                                if (oXML.FindChildElem("ConversionFactor"))
                                {
                                    sForceChannel.fConversionFactor = (float)atof(oXML.GetChildData().c_str());
                                }
                                sForcePlate.vChannels.push_back(sForceChannel);
                                oXML.OutOfElem();
                            }
                            oXML.OutOfElem();
                        }
                        if (oXML.FindChildElem("Calibration_Matrix"))
                        {
                            oXML.IntoElem();
                            int nRow = 0;
                            char strRow[16];
                            char strCol[16];
                            sprintf(strRow, "Row%d", nRow + 1);
                            while (oXML.FindChildElem(strRow))
                            {
                                oXML.IntoElem();
                                int nCol = 0;
                                sprintf(strCol, "Col%d", nCol + 1);
                                while (oXML.FindChildElem(strCol))
                                {
                                    sForcePlate.afCalibrationMatrix[nRow][nCol] = (float)atof(oXML.GetChildData().c_str());
                                    nCol++;
                                    sprintf(strCol, "Col%d", nCol + 1);
                                }
                                nRow++;
                                sprintf(strRow, "Row%d", nRow + 1);
                                oXML.OutOfElem(); // RowX
                            }
                            sForcePlate.bValidCalibrationMatrix = true;

                            oXML.OutOfElem(); // "Plate"
                        }
                        oXML.OutOfElem(); // "Plate"
                    }
                    if (bPlateExists)
                    {
                        msForceSettings.vsForcePlates.push_back(sForcePlate);
                    }
                }
            }
            else
            {
                if (eType == CRTPacket::PacketError)
                {
                    snprintf(maErrorStr, sizeof(maErrorStr), "%s.", mpoRTPacket->GetErrorString());
                }
                else
                {
                    snprintf(maErrorStr, sizeof(maErrorStr),
                        "GetParameters Force returned wrong packet type. Got type %d expected type 2.", eType);
                }
            }
        }
        else
        {
            char pTmpStr[256];
            strncpy(pTmpStr, maErrorStr, sizeof(pTmpStr));
            snprintf(maErrorStr, sizeof(maErrorStr), "No response received. Expected XML packet. %s", pTmpStr);
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "GetParameters Force failed");
    }
    return bReturn;
} // Read force settings

bool CRTProtocol::ReadImageSettings()
{
    CRTPacket::EPacketType  eType;
    CMarkup             oXML;
    bool                    bReturn = false;

    mvsImageSettings.clear();

    if (SendCommand("GetParameters Image"))
    {
        if (ReceiveRTPacket(eType, true))
        {
            if (eType == CRTPacket::PacketXML)
            {
                oXML.SetDoc(mpoRTPacket->GetXMLString());
                //
                // Read some Image parameters
                //
                if (oXML.FindChildElem("Image") && oXML.IntoElem())
                {
                    while (oXML.FindChildElem("Camera"))
                    {
                        oXML.IntoElem();

                        SImageCamera sImageCamera;

                        if (oXML.FindChildElem("ID"))
                        {
                            sImageCamera.nID = atoi(oXML.GetChildData().c_str());

                            if (oXML.FindChildElem("Enabled"))
                            {
                                CStdStringA oStr;
                                oStr = oXML.GetChildData();

                                if (oStr.CompareNoCase("True") == 0)
                                {
                                    sImageCamera.bEnabled = true;
                                }
                                else
                                {
                                    sImageCamera.bEnabled = false;
                                }

                                if (oXML.FindChildElem("Format"))
                                {
                                    CStdStringA oFormat;
                                    oFormat = oXML.GetChildData();

                                    if (oFormat.CompareNoCase("RAWGrayscale") == 0)
                                    {
                                        sImageCamera.eFormat = CRTPacket::FormatRawGrayscale;
                                    }
                                    if (oFormat.CompareNoCase("RAWBGR") == 0)
                                    {
                                        sImageCamera.eFormat = CRTPacket::FormatRawBGR;
                                    }
                                    if (oFormat.CompareNoCase("JPG") == 0)
                                    {
                                        sImageCamera.eFormat = CRTPacket::FormatJPG;
                                    }
                                    if (oFormat.CompareNoCase("PNG") == 0)
                                    {
                                        sImageCamera.eFormat = CRTPacket::FormatPNG;
                                    }
                                    //sImageCamera.nID = atoi(oXML.GetChildData().c_str());

                                    if (oXML.FindChildElem("Width"))
                                    {
                                        sImageCamera.nWidth = atoi(oXML.GetChildData().c_str());

                                        if (oXML.FindChildElem("Height"))
                                        {
                                            sImageCamera.nHeight = atoi(oXML.GetChildData().c_str());

                                            if (oXML.FindChildElem("Left_Crop"))
                                            {
                                                sImageCamera.fCropLeft = (float)atof(oXML.GetChildData().c_str());

                                                if (oXML.FindChildElem("Top_Crop"))
                                                {
                                                    sImageCamera.fCropTop = (float)atof(oXML.GetChildData().c_str());

                                                    if (oXML.FindChildElem("Right_Crop"))
                                                    {
                                                        sImageCamera.fCropRight = (float)atof(oXML.GetChildData().c_str());

                                                        if (oXML.FindChildElem("Bottom_Crop"))
                                                        {
                                                            sImageCamera.fCropBottom = (float)atof(oXML.GetChildData().c_str());
                                                            mvsImageSettings.push_back(sImageCamera);
                                                            bReturn = true;
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                if (eType == CRTPacket::PacketError)
                {
                    snprintf(maErrorStr, sizeof(maErrorStr), "%s.", mpoRTPacket->GetErrorString());
                }
                else
                {
                    snprintf(maErrorStr, sizeof(maErrorStr),
                        "GetParameters Image returned wrong packet type. Got type %d expected type 2.", eType);
                }
            }
        }
        else
        {
            char pTmpStr[256];
            strncpy(pTmpStr, maErrorStr, sizeof(pTmpStr));
            snprintf(maErrorStr, sizeof(maErrorStr), "No response received. Expected XML packet. %s", pTmpStr);
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "GetParameters 6D failed");
    }
    return bReturn;
} // ReadImageSettings


void CRTProtocol::GetGeneral(unsigned int       &nCaptureFrequency, float &fCaptureTime, bool &bStartOnExtTrig,
                             EProcessingActions &eProcessingActions)
{
    nCaptureFrequency  = msGeneralSettings.nCaptureFrequency;
    fCaptureTime       = msGeneralSettings.fCaptureTime;
    bStartOnExtTrig    = msGeneralSettings.bStartOnExternalTrigger;
    eProcessingActions = msGeneralSettings.eProcessingActions;
}


// External time base settings only available in version 1.10 of the rt protocol and later
void CRTProtocol::GetGeneralExtTimeBase(bool &bEnabled,                    ESignalSource &eSignalSource,
                                        bool &bSignalModePeriodic,         unsigned int &nFreqMultiplier,
                                        unsigned int &nFreqDivisor,        unsigned int &nFreqTolerance,
                                        float &fNominalFrequency,          bool &bNegativeEdge,
                                        unsigned int &nSignalShutterDelay, float &fNonPeriodicTimeout)
{
    bEnabled            = msGeneralSettings.sExternalTimebase.bEnabled;
    eSignalSource       = msGeneralSettings.sExternalTimebase.eSignalSource;
    bSignalModePeriodic = msGeneralSettings.sExternalTimebase.bSignalModePeriodic;
    nFreqMultiplier     = msGeneralSettings.sExternalTimebase.nFreqMultiplier;
    nFreqDivisor        = msGeneralSettings.sExternalTimebase.nFreqDivisor;
    nFreqTolerance      = msGeneralSettings.sExternalTimebase.nFreqTolerance;
    fNominalFrequency   = msGeneralSettings.sExternalTimebase.fNominalFrequency;
    bNegativeEdge       = msGeneralSettings.sExternalTimebase.bNegativeEdge;
    nSignalShutterDelay = msGeneralSettings.sExternalTimebase.nSignalShutterDelay;
    fNonPeriodicTimeout = msGeneralSettings.sExternalTimebase.fNonPeriodicTimeout;
}


unsigned int CRTProtocol::GetGeneralCameraCount()
{
    return msGeneralSettings.vsCameras.size();
}


bool CRTProtocol::GetGeneralCamera(unsigned int nCameraIndex, unsigned int &nID,     ECameraModel &eModel,
                                   bool         &bUnderwater, unsigned int &nSerial, ECameraMode  &eMode)
{
    if (nCameraIndex < msGeneralSettings.vsCameras.size())
    {
        nID                      = msGeneralSettings.vsCameras[nCameraIndex].nID;
        eModel                   = msGeneralSettings.vsCameras[nCameraIndex].eModel;
        bUnderwater              = msGeneralSettings.vsCameras[nCameraIndex].bUnderwater;
        nSerial                  = msGeneralSettings.vsCameras[nCameraIndex].nSerial;
        eMode                    = msGeneralSettings.vsCameras[nCameraIndex].eMode;
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


bool CRTProtocol::GetGeneralCameraMarker(unsigned int nCameraIndex, unsigned int &nCurrentExposure, unsigned int &nMinExposure,
                                         unsigned int &nMaxExposure, unsigned int &nCurrentThreshold,
                                         unsigned int &nMinThreshold, unsigned int &nMaxThreshold)
{
    if (nCameraIndex < msGeneralSettings.vsCameras.size())
    {
        nCurrentExposure  = msGeneralSettings.vsCameras[nCameraIndex].nMarkerExposure;
        nMinExposure      = msGeneralSettings.vsCameras[nCameraIndex].nMarkerExposureMin;
        nMaxExposure      = msGeneralSettings.vsCameras[nCameraIndex].nMarkerExposureMax;
        nCurrentThreshold = msGeneralSettings.vsCameras[nCameraIndex].nMarkerThreshold;
        nMinThreshold     = msGeneralSettings.vsCameras[nCameraIndex].nMarkerThresholdMin;
        nMaxThreshold     = msGeneralSettings.vsCameras[nCameraIndex].nMarkerThresholdMax;
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


bool CRTProtocol::GetGeneralCameraVideo(unsigned int nCameraIndex, unsigned int &nCurrentExposure, unsigned int &nMinExposure,
                                        unsigned int &nMaxExposure, unsigned int &nCurrentFlashTime,
                                        unsigned int &nMinFlashTime, unsigned int &nMaxFlashTime)
{
    if (nCameraIndex < msGeneralSettings.vsCameras.size())
    {
        nCurrentExposure  = msGeneralSettings.vsCameras[nCameraIndex].nVideoExposure;
        nMinExposure      = msGeneralSettings.vsCameras[nCameraIndex].nVideoExposureMin;
        nMaxExposure      = msGeneralSettings.vsCameras[nCameraIndex].nVideoExposureMax;
        nCurrentFlashTime = msGeneralSettings.vsCameras[nCameraIndex].nVideoFlashTime;
        nMinFlashTime     = msGeneralSettings.vsCameras[nCameraIndex].nVideoFlashTimeMin;
        nMaxFlashTime     = msGeneralSettings.vsCameras[nCameraIndex].nVideoFlashTimeMax;
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


bool CRTProtocol::GetGeneralCameraPosition(unsigned int nCameraIndex, SPoint &sPoint, float fvRotationMatrix[3][3])
{
    if (nCameraIndex < msGeneralSettings.vsCameras.size())
    {
        sPoint.fX = msGeneralSettings.vsCameras[nCameraIndex].fPositionX;
        sPoint.fY = msGeneralSettings.vsCameras[nCameraIndex].fPositionY;
        sPoint.fZ = msGeneralSettings.vsCameras[nCameraIndex].fPositionZ;
        memcpy(fvRotationMatrix, msGeneralSettings.vsCameras[nCameraIndex].fPositionRotMatrix,
                 9 * sizeof(float));
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


bool CRTProtocol::GetGeneralCameraOrientation(unsigned int nCameraIndex, int &nOrientation)
{
    if (nCameraIndex < msGeneralSettings.vsCameras.size())
    {
        nOrientation = msGeneralSettings.vsCameras[nCameraIndex].nOrientation;
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}

bool CRTProtocol::GetGeneralCameraResolution(unsigned int nCameraIndex, unsigned int &nMarkerWidth, unsigned int &nMarkerHeight,
                                             unsigned int &nVideoWidth, unsigned int &nVideoHeight)
{
    if (nCameraIndex < msGeneralSettings.vsCameras.size())
    {
        nMarkerWidth  = msGeneralSettings.vsCameras[nCameraIndex].nMarkerResolutionWidth;
        nMarkerHeight = msGeneralSettings.vsCameras[nCameraIndex].nMarkerResolutionHeight;
        nVideoWidth   = msGeneralSettings.vsCameras[nCameraIndex].nVideoResolutionWidth;
        nVideoHeight  = msGeneralSettings.vsCameras[nCameraIndex].nVideoResolutionHeight;
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}

bool CRTProtocol::GetGeneralCameraFOV(unsigned int nCameraIndex,  unsigned int &nMarkerLeft,  unsigned int &nMarkerTop,
                                      unsigned int &nMarkerRight, unsigned int &nMarkerBottom,
                                      unsigned int &nVideoLeft,   unsigned int &nVideoTop,
                                      unsigned int &nVideoRight,  unsigned int &nVideoBottom)
{
    if (nCameraIndex < msGeneralSettings.vsCameras.size())
    {
        nMarkerLeft   = msGeneralSettings.vsCameras[nCameraIndex].nMarkerFOVLeft;
        nMarkerTop    = msGeneralSettings.vsCameras[nCameraIndex].nMarkerFOVTop;
        nMarkerRight  = msGeneralSettings.vsCameras[nCameraIndex].nMarkerFOVRight;
        nMarkerBottom = msGeneralSettings.vsCameras[nCameraIndex].nMarkerFOVBottom;
        nVideoLeft    = msGeneralSettings.vsCameras[nCameraIndex].nVideoFOVLeft;
        nVideoTop     = msGeneralSettings.vsCameras[nCameraIndex].nVideoFOVTop;
        nVideoRight   = msGeneralSettings.vsCameras[nCameraIndex].nVideoFOVRight;
        nVideoBottom  = msGeneralSettings.vsCameras[nCameraIndex].nVideoFOVBottom;
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


bool CRTProtocol::GetGeneralCameraSyncOut(unsigned int nCameraIndex,   ESyncOutFreqMode &eSyncOutMode,
                                          unsigned int &nSyncOutValue, float            &fSyncOutDutyCycle,
                                          bool         &bSyncOutNegativePolarity)
{
    if (nCameraIndex < msGeneralSettings.vsCameras.size())
    {
        eSyncOutMode             = msGeneralSettings.vsCameras[nCameraIndex].eSyncOutMode;
        nSyncOutValue            = msGeneralSettings.vsCameras[nCameraIndex].nSyncOutValue;
        fSyncOutDutyCycle        = msGeneralSettings.vsCameras[nCameraIndex].fSyncOutDutyCycle;
        bSyncOutNegativePolarity = msGeneralSettings.vsCameras[nCameraIndex].bSyncOutNegativePolarity;
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


CRTProtocol::EAxis CRTProtocol::Get3DUpwardAxis()
{
    return ms3DSettings.eAxisUpwards;
}


char* CRTProtocol::Get3DCalibrated()
{
    return ms3DSettings.pCalibrationTime;
}


unsigned int CRTProtocol::Get3DLabeledMarkerCount()
{
    return ms3DSettings.s3DLabels.size();
}


const char* CRTProtocol::Get3DLabelName(unsigned int nMarkerIndex)
{
    if (nMarkerIndex < ms3DSettings.s3DLabels.size())
    {
        return ms3DSettings.s3DLabels[nMarkerIndex].oName.c_str();
    }

    maErrorStr[0] = 0; // No error string.
    return NULL;
}


unsigned int CRTProtocol::Get3DLabelColor(unsigned int nMarkerIndex)
{
    if (nMarkerIndex < ms3DSettings.s3DLabels.size())
    {
        return ms3DSettings.s3DLabels[nMarkerIndex].nRGBColor;
    }

    maErrorStr[0] = 0; // No error string.
    return 0;
}


unsigned int CRTProtocol::Get6DOFBodyCount()
{
    return mvs6DOFBodySettings.size();
}


const char* CRTProtocol::Get6DOFBodyName(unsigned int nBodyIndex)
{
    if (nBodyIndex < mvs6DOFBodySettings.size())
    {
        return mvs6DOFBodySettings[nBodyIndex].oName.c_str();
    }

    maErrorStr[0] = 0; // No error string.
    return NULL;
}


unsigned int CRTProtocol::Get6DOFBodyColor(unsigned int nBodyIndex)
{
    if (nBodyIndex < mvs6DOFBodySettings.size())
    {
        return mvs6DOFBodySettings[nBodyIndex].nRGBColor;
    }

    maErrorStr[0] = 0; // No error string.
    return 0;
}


unsigned int CRTProtocol::Get6DOFBodyPointCount(unsigned int nBodyIndex)
{
    if (nBodyIndex < mvs6DOFBodySettings.size())
    {
        return mvs6DOFBodySettings.at(nBodyIndex).vsPoints.size();
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


bool CRTProtocol::Get6DOFBodyPoint(unsigned int nBodyIndex, unsigned int nMarkerIndex, SPoint &sPoint)
{
    if (nBodyIndex < mvs6DOFBodySettings.size())
    {
        if (nMarkerIndex < mvs6DOFBodySettings.at(nBodyIndex).vsPoints.size())
        {
            sPoint.fX = mvs6DOFBodySettings.at(nBodyIndex).vsPoints[nMarkerIndex].fX;
            sPoint.fY = mvs6DOFBodySettings.at(nBodyIndex).vsPoints[nMarkerIndex].fY;
            sPoint.fZ = mvs6DOFBodySettings.at(nBodyIndex).vsPoints[nMarkerIndex].fZ;
            return true;
        }
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


unsigned int CRTProtocol::GetAnalogDeviceCount()
{
    return mvsAnalogDeviceSettings.size();
}


bool CRTProtocol::GetAnalogDevice(unsigned int nDeviceIndex, unsigned int &nDeviceID, unsigned int &nChannels,
                                  char* &pName, unsigned int &nFrequency, char* &pUnit,
                                  unsigned int &nMinRange, unsigned int &nMaxRange)
{
    if (nDeviceIndex < mvsAnalogDeviceSettings.size())
    {
        nDeviceID  = mvsAnalogDeviceSettings.at(nDeviceIndex).nDeviceID;
        pName      = (char*)mvsAnalogDeviceSettings.at(nDeviceIndex).oName.c_str();
        nChannels  = mvsAnalogDeviceSettings.at(nDeviceIndex).nChannels;
        nFrequency = mvsAnalogDeviceSettings.at(nDeviceIndex).nFrequency;
        pUnit      = (char*)mvsAnalogDeviceSettings.at(nDeviceIndex).oUnit.c_str();
        nMinRange  = mvsAnalogDeviceSettings.at(nDeviceIndex).nMinRange;
        nMaxRange  = mvsAnalogDeviceSettings.at(nDeviceIndex).nMaxRange;

        return true;
    }

    maErrorStr[0] = 0; // No error string.
    return false;
}


const char* CRTProtocol::GetAnalogLabel(unsigned int nDeviceIndex, unsigned int nLabelIndex)
{
    if (nDeviceIndex < mvsAnalogDeviceSettings.size())
    {
        if (nLabelIndex < mvsAnalogDeviceSettings.at(nDeviceIndex).voLabels.size())
        {
            return mvsAnalogDeviceSettings.at(nDeviceIndex).voLabels.at(nLabelIndex).c_str();
        }
    }

    maErrorStr[0] = 0; // No error string.
    return NULL;
}


void CRTProtocol::GetForceUnits(char* &pLength, char* &pForce)
{
    pLength = (char*)msForceSettings.oUnitLength.c_str();
    pForce  = (char*)msForceSettings.oUnitForce.c_str();
}


unsigned int CRTProtocol::GetForcePlateCount()
{
    return msForceSettings.vsForcePlates.size();
}


bool CRTProtocol::GetForcePlate(unsigned int nPlateIndex, unsigned int &nID, unsigned int &nAnalogDeviceID,
                                unsigned int &nFrequency, char* &pType, char* &pName, float &fLength, float &fWidth)
{
    if (nPlateIndex < msForceSettings.vsForcePlates.size())
    {
        nID             = msForceSettings.vsForcePlates[nPlateIndex].nID;
        nAnalogDeviceID = msForceSettings.vsForcePlates[nPlateIndex].nAnalogDeviceID;
        nFrequency      = msForceSettings.vsForcePlates[nPlateIndex].nFrequency;
        pType           = (char*)msForceSettings.vsForcePlates[nPlateIndex].oType.c_str();
        pName           = (char*)msForceSettings.vsForcePlates[nPlateIndex].oName.c_str();
        fLength         = msForceSettings.vsForcePlates[nPlateIndex].fLength;
        fWidth          = msForceSettings.vsForcePlates[nPlateIndex].fWidth;
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


bool CRTProtocol::GetForcePlateLocation(unsigned int nPlateIndex, SPoint sCorner[4])
{
    if (nPlateIndex < msForceSettings.vsForcePlates.size())
    {
        memcpy(sCorner, msForceSettings.vsForcePlates[nPlateIndex].asCorner, 3 * 4 * sizeof(float));
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


bool CRTProtocol::GetForcePlateOrigin(unsigned int nPlateIndex, SPoint &sOrigin)
{
    if (nPlateIndex < msForceSettings.vsForcePlates.size())
    {
        sOrigin = msForceSettings.vsForcePlates[nPlateIndex].sOrigin;
        return true;
    }
    maErrorStr[0] = 0; // No error string.
    return false;
}


unsigned int CRTProtocol::GetForcePlateChannelCount(unsigned int nPlateIndex)
{
    if (nPlateIndex < msForceSettings.vsForcePlates.size())
    {
        return msForceSettings.vsForcePlates[nPlateIndex].vChannels.size();
    }
    maErrorStr[0] = 0; // No error string.
    return 0;
}


bool CRTProtocol::GetForcePlateChannel(unsigned int nPlateIndex, unsigned int nChannelIndex,
                                       unsigned int &nChannelNumber, float &fConversionFactor)
{
    if (nPlateIndex < msForceSettings.vsForcePlates.size())
    {
        if (nChannelIndex < msForceSettings.vsForcePlates[nPlateIndex].vChannels.size())
        {
            nChannelNumber    = msForceSettings.vsForcePlates[nPlateIndex].vChannels[nChannelIndex].nChannelNumber;
            fConversionFactor = msForceSettings.vsForcePlates[nPlateIndex].vChannels[nChannelIndex].fConversionFactor;
            return true;
        }
        else
        {
            snprintf(maErrorStr, sizeof(maErrorStr), "No channel index %d on force plate index %d.", nChannelIndex, nPlateIndex);
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "No force plate index %d.", nPlateIndex);
    }
    return false;
}


bool CRTProtocol::GetForcePlateCalibrationMatrix(unsigned int nPlateIndex, float fvCalMatrix[6][6])
{
    if (nPlateIndex < msForceSettings.vsForcePlates.size())
    {
        if (msForceSettings.vsForcePlates[nPlateIndex].bValidCalibrationMatrix)
        {
            memcpy(fvCalMatrix, msForceSettings.vsForcePlates[nPlateIndex].afCalibrationMatrix,
                6 * 6 * sizeof(float));
            return true;
        }
    }
    snprintf(maErrorStr, sizeof(maErrorStr), "No force plate calibration matrix for camera index %d.", nPlateIndex);
    return false;
}


unsigned int CRTProtocol::GetImageCameraCount()
{
    return mvsImageSettings.size();
}


bool CRTProtocol::GetImageCamera(unsigned int nCameraIndex, unsigned int &nCameraID, bool &bEnabled,
                                 CRTPacket::EImageFormat &eFormat, unsigned int &nWidth, unsigned int &nHeight,
                                 float &fCropLeft, float &fCropTop, float &fCropRight, float &fCropBottom)
{
    if (nCameraIndex < mvsImageSettings.size())
    {
        nCameraID   = mvsImageSettings[nCameraIndex].nID;
        bEnabled    = mvsImageSettings[nCameraIndex].bEnabled;
        eFormat     = mvsImageSettings[nCameraIndex].eFormat;
        nWidth      = mvsImageSettings[nCameraIndex].nWidth;
        nHeight     = mvsImageSettings[nCameraIndex].nHeight;
        fCropLeft   = mvsImageSettings[nCameraIndex].fCropLeft;
        fCropTop    = mvsImageSettings[nCameraIndex].fCropTop;
        fCropRight  = mvsImageSettings[nCameraIndex].fCropRight;
        fCropBottom = mvsImageSettings[nCameraIndex].fCropBottom;
        return true;
    }
    snprintf(maErrorStr, sizeof(maErrorStr), "No camera image settings for camera index %d.", nCameraIndex);
    return false;
}


bool CRTProtocol::SetGeneral(const unsigned int* nCaptureFrequency, const float*              fCaptureTime,
                             const bool*         bStartOnExtTrig,   const EProcessingActions* eProcessingActions)
{
    CMarkup oXML;
    CStdString  tVal;

    oXML.AddElem("QTM_Settings");
    oXML.IntoElem();
    oXML.AddElem("General");
    oXML.IntoElem();

    AddXMLElementUnsignedInt(&oXML, "Frequency", nCaptureFrequency);
    AddXMLElementFloat(&oXML, "Capture_Time", fCaptureTime, 3);
    AddXMLElementBool(&oXML, "Start_On_External_Trigger", bStartOnExtTrig);

    if (eProcessingActions)
    {
        oXML.AddElem("Processing_Actions");
        oXML.IntoElem();

        if (*eProcessingActions & ProcessingTracking2D)
        {
            oXML.AddElem("Tracking", "2D");
        }
        else if (*eProcessingActions & ProcessingTracking3D)
        {
            oXML.AddElem("Tracking", "3D");
        }
        else
        {
            oXML.AddElem("Tracking", "False");
        }

        AddXMLElementBool(&oXML, "TwinSystemMerge", (*eProcessingActions & ProcessingTwinSystemMerge) != 0);
        AddXMLElementBool(&oXML, "SplineFill", (*eProcessingActions & ProcessingSplineFill) != 0);
        AddXMLElementBool(&oXML, "AIM", (*eProcessingActions & ProcessingAIM) != 0);
        AddXMLElementBool(&oXML, "Track6DOF", (*eProcessingActions & Processing6DOFTracking) != 0);
        AddXMLElementBool(&oXML, "ForceData", (*eProcessingActions & ProcessingForceData) != 0);
        AddXMLElementBool(&oXML, "ExportTSV", (*eProcessingActions & ProcessingExportTSV) != 0);
        AddXMLElementBool(&oXML, "ExportC3D", (*eProcessingActions & ProcessingExportC3D) != 0);
        AddXMLElementBool(&oXML, "ExportDiff", (*eProcessingActions & ProcessingExportDiff) != 0);
        AddXMLElementBool(&oXML, "ExportMatlabDirect", (*eProcessingActions & ProcessingExportMatlabDirect) != 0);
        AddXMLElementBool(&oXML, "ExportMatlabFile", (*eProcessingActions & ProcessingExportMatlabFile) != 0);

        oXML.OutOfElem(); // Processing_Actions
    }
    oXML.OutOfElem(); // General
    oXML.OutOfElem(); // QTM_Settings

    if (SendXML(oXML.GetDoc().c_str()))
    {
        return true;
    }

    return false;
} // SetGeneral


bool CRTProtocol::SetGeneralExtTimeBase(const bool*         bEnabled,            const ESignalSource* eSignalSource,
                                        const bool*         bSignalModePeriodic, const unsigned int*  nFreqMultiplier,
                                        const unsigned int* nFreqDivisor,        const unsigned int*  nFreqTolerance,
                                        const float*        fNominalFrequency,   const bool*          bNegativeEdge,
                                        const unsigned int* nSignalShutterDelay, const float*         fNonPeriodicTimeout)
{
    CMarkup oXML;
    CStdString  tVal;

    oXML.AddElem("QTM_Settings");
    oXML.IntoElem();
    oXML.AddElem("General");
    oXML.IntoElem();
    oXML.AddElem("External_Time_Base");
    oXML.IntoElem();

    AddXMLElementBool(&oXML, "Enabled", bEnabled);

    if (eSignalSource)
    {
        if (*eSignalSource == SourceControlPort)
        {
            tVal.Format("Control port");
        }
        if (*eSignalSource == SourceIRReceiver)
        {
            tVal.Format("IR receiver");
        }
        if (*eSignalSource == SourceSMPTE)
        {
            tVal.Format("SMPTE");
        }
        if (*eSignalSource == SourceVideoSync)
        {
            tVal.Format("Video sync");
        }
        oXML.AddElem("Signal_Source", tVal);
    }

    AddXMLElementBool(&oXML, "Signal_Mode", bSignalModePeriodic, "Periodic", "Non-periodic");
    AddXMLElementUnsignedInt(&oXML, "Frequency_Multiplier", nFreqMultiplier);
    AddXMLElementUnsignedInt(&oXML, "Frequency_Divisor", nFreqDivisor);
    AddXMLElementUnsignedInt(&oXML, "Frequency_Tolerance", nFreqTolerance);

    if (fNominalFrequency)
    {
        if (*fNominalFrequency < 0)
        {
            oXML.AddElem("Nominal_Frequency", "None");
        }
        else
        {
            AddXMLElementFloat(&oXML, "Nominal_Frequency", fNominalFrequency, 3);
        }
    }

    AddXMLElementBool(&oXML, "Signal_Edge", bNegativeEdge, "Negative", "Positive");
    AddXMLElementUnsignedInt(&oXML, "Signal_Shutter_Delay", nSignalShutterDelay);
    AddXMLElementFloat(&oXML, "Non_Periodic_Timeout", fNonPeriodicTimeout, 3);

    oXML.OutOfElem(); // External_Time_Base
    oXML.OutOfElem(); // General
    oXML.OutOfElem(); // QTM_Settings

    if (SendXML(oXML.GetDoc().c_str()))
    {
        return true;
    }

    return false;
} // SetGeneralExtTimeBase


// nCameraID starts on 1. If nCameraID < 0 then settings are applied to all cameras.
bool CRTProtocol::SetGeneralCamera(const unsigned int nCameraID,       const ECameraMode* eMode,
                                   const float*       fVideoExposure,  const float*       fVideoFlashTime,
                                   const float*       fMarkerExposure, const float*       fMarkerThreshold,
                                   const int*         nOrientation)
{
    CMarkup oXML;
    CStdString  tVal;

    oXML.AddElem("QTM_Settings");
    oXML.IntoElem();
    oXML.AddElem("General");
    oXML.IntoElem();

    oXML.AddElem("Camera");
    oXML.IntoElem();
    tVal.Format("%d", nCameraID);
    oXML.AddElem("ID", tVal);
    if (eMode)
    {
        switch (*eMode)
        {
        case ModeMarker :
            tVal.Format("Marker");
            break;
        case ModeMarkerIntensity :
            tVal.Format("Marker Intensity");
            break;
        case ModeVideo :
            tVal.Format("Video");
            break;
        }
        oXML.AddElem("Mode", tVal);
    }
    AddXMLElementFloat(&oXML, "Video_Exposure", fVideoExposure);
    AddXMLElementFloat(&oXML, "Video_Flash_Time", fVideoFlashTime);
    AddXMLElementFloat(&oXML, "Marker_Exposure", fMarkerExposure);
    AddXMLElementFloat(&oXML, "Marker_Threshold", fMarkerThreshold);
    AddXMLElementInt(&oXML, "Orientation", nOrientation);

    oXML.OutOfElem(); // Camera
    oXML.OutOfElem(); // General
    oXML.OutOfElem(); // QTM_Settings

    if (SendXML(oXML.GetDoc().c_str()))
    {
        return true;
    }

    return false;
} // SetGeneralCamera


// nCameraID starts on 1. If nCameraID < 0 then settings are applied to all cameras.
bool CRTProtocol::SetGeneralCameraSyncOut(const unsigned int  nCameraID,       const ESyncOutFreqMode* peSyncOutMode,
                                          const unsigned int* pnSyncOutValue,   const float*            pfSyncOutDutyCycle,
                                          const bool*         pbSyncOutNegativePolarity)
{
    CMarkup oXML;
    CStdString  tVal;

    oXML.AddElem("QTM_Settings");
    oXML.IntoElem();
    oXML.AddElem("General");
    oXML.IntoElem();

    if (peSyncOutMode)
    {
        oXML.AddElem("Camera");
        oXML.IntoElem();
        tVal.Format("%d", nCameraID);
        oXML.AddElem("ID", tVal);

        oXML.AddElem("Sync_Out");
        oXML.IntoElem();
        switch (*peSyncOutMode)
        {
            case ModeShutterOut :
                tVal.Format("Shutter out");
                break;
            case ModeMultiplier :
                tVal.Format("Multiplier");
                break;
            case ModeDivisor :
                tVal.Format("Divisor");
                break;
            case ModeActualFreq :
                tVal.Format("Camera independent");
                break;
            case ModeMeasurementTime :
                tVal.Format("Measurement time");
                break;
            case ModeFixed100Hz :
                tVal.Format("Continuous 100Hz");
                break;
            default :
                return false; // Should never happen
        }
        oXML.AddElem("Mode", tVal);

        if (*peSyncOutMode == ModeMultiplier ||
            *peSyncOutMode == ModeDivisor    ||
            *peSyncOutMode == ModeActualFreq)
        {
            AddXMLElementUnsignedInt(&oXML, "Value", pnSyncOutValue);
            AddXMLElementFloat(&oXML, "Duty_Cycle", pfSyncOutDutyCycle, 3);
        }
        if (pbSyncOutNegativePolarity &&
            (*peSyncOutMode != ModeSRAMWireSync || *peSyncOutMode != ModeFixed100Hz))
        {
            AddXMLElementBool(&oXML, "Signal_Polarity", pbSyncOutNegativePolarity, "Negative", "Positive");
        }
        oXML.OutOfElem(); // Sync_Out

        oXML.OutOfElem(); // Camera
    }
    oXML.OutOfElem(); // General
    oXML.OutOfElem(); // QTM_Settings

    if (SendXML(oXML.GetDoc().c_str()))
    {
        return true;
    }

    return false;
} // SetGeneralCameraSyncOut


bool CRTProtocol::SetImageSettings(const unsigned int  nCameraID, const bool* bEnable, const CRTPacket::EImageFormat* eFormat,
                                   const unsigned int* nWidth, const unsigned int* nHeight, const float* fLeftCrop,
                                   const float* fTopCrop, const float* fRightCrop, const float* fBottomCrop)
{
    CMarkup oXML;
    CStdString  tVal;

    oXML.AddElem("QTM_Settings");
    oXML.IntoElem();
    oXML.AddElem("Image");
    oXML.IntoElem();

    oXML.AddElem("Camera");
    oXML.IntoElem();

    tVal.Format("%d", nCameraID);
    oXML.AddElem("ID", tVal);

    AddXMLElementBool(&oXML, "Enabled", bEnable);

    if (eFormat)
    {
        switch (*eFormat)
        {
        case CRTPacket::FormatRawGrayscale :
            tVal.Format("RAWGrayscale");
            break;
        case CRTPacket::FormatRawBGR :
            tVal.Format("RAWBGR");
            break;
        case CRTPacket::FormatJPG :
            tVal.Format("JPG");
            break;
        case CRTPacket::FormatPNG :
            tVal.Format("PNG");
            break;
        }
        oXML.AddElem("Mode", tVal);
        tVal.Format("%d", (int)*eFormat);
        oXML.AddElem("Format", tVal);
    }
    AddXMLElementUnsignedInt(&oXML, "Width", nWidth);
    AddXMLElementUnsignedInt(&oXML, "Height", nHeight);
    AddXMLElementFloat(&oXML, "Left_Crop", fLeftCrop);
    AddXMLElementFloat(&oXML, "Top_Crop", fTopCrop);
    AddXMLElementFloat(&oXML, "Right_Crop", fRightCrop);
    AddXMLElementFloat(&oXML, "Bottom_Crop", fBottomCrop);

    oXML.OutOfElem(); // Camera
    oXML.OutOfElem(); // Image
    oXML.OutOfElem(); // QTM_Settings

    if (SendXML(oXML.GetDoc().c_str()))
    {
        return true;
    }

    return false;
} // SetImageSettings


bool CRTProtocol::SetForceSettings(const unsigned int nPlateID, const SPoint* sCorner1, const SPoint* sCorner2,
                                   const SPoint*      sCorner3, const SPoint* sCorner4)
{
    CMarkup oXML;
    CStdString  tVal;

    if (nPlateID > 0)
    {
        oXML.AddElem("QTM_Settings");
        oXML.IntoElem();
        oXML.AddElem("Force");
        oXML.IntoElem();

        oXML.AddElem("Plate");
        oXML.IntoElem();
        tVal.Format("%d", nPlateID);
        if (mnMajorVersion > 1 || mnMinorVersion > 7)
        {
            oXML.AddElem("Plate_ID", tVal);
        }
        else
        {
            oXML.AddElem("Force_Plate_Index", tVal);
        }
        if (sCorner1)
        {
            oXML.AddElem("Corner1");
            oXML.IntoElem();
            AddXMLElementFloat(&oXML, "X", &(sCorner1->fX));
            AddXMLElementFloat(&oXML, "Y", &(sCorner1->fY));
            AddXMLElementFloat(&oXML, "Z", &(sCorner1->fZ));
            oXML.OutOfElem(); // Corner1
        }
        if (sCorner2)
        {
            oXML.AddElem("Corner2");
            oXML.IntoElem();
            AddXMLElementFloat(&oXML, "X", &(sCorner2->fX));
            AddXMLElementFloat(&oXML, "Y", &(sCorner2->fY));
            AddXMLElementFloat(&oXML, "Z", &(sCorner2->fZ));
            oXML.OutOfElem(); // Corner2
        }
        if (sCorner3)
        {
            oXML.AddElem("Corner3");
            oXML.IntoElem();
            AddXMLElementFloat(&oXML, "X", &(sCorner3->fX));
            AddXMLElementFloat(&oXML, "Y", &(sCorner3->fY));
            AddXMLElementFloat(&oXML, "Z", &(sCorner3->fZ));
            oXML.OutOfElem(); // Corner3
        }
        if (sCorner4)
        {
            oXML.AddElem("Corner4");
            oXML.IntoElem();
            AddXMLElementFloat(&oXML, "X", &(sCorner4->fX));
            AddXMLElementFloat(&oXML, "Y", &(sCorner4->fY));
            AddXMLElementFloat(&oXML, "Z", &(sCorner4->fZ));
            oXML.OutOfElem(); // Corner4
        }
        oXML.OutOfElem(); // Plate

        oXML.OutOfElem(); // Force
        oXML.OutOfElem(); // QTM_Settings

        if (SendXML(oXML.GetDoc().c_str()))
        {
            return true;
        }
    }
    else
    {
        snprintf(maErrorStr, sizeof(maErrorStr), "Illegal force plate id: %d.", nPlateID);
    }
    return false;
} // SetForceSettings


char* CRTProtocol::GetErrorString()
{
    return maErrorStr;
}


bool CRTProtocol::SendString(const char* pCmdStr, int nType)
{
    int         nSize;
    //int         nSent      = 0;
    //int         nTotSent   = 0;
    int         nCmdStrLen = strlen(pCmdStr);
    static char aSendBuffer[5000];

    if (nCmdStrLen > static_cast<int>(sizeof(aSendBuffer)))
    {
        snprintf (maErrorStr, sizeof(maErrorStr), "String is larger than send buffer.");
        return false;
    }

    //
    // Header size + length of the string + terminating null char
    //
    nSize = 8 + nCmdStrLen + 1;

    memcpy(aSendBuffer + 8, pCmdStr, nCmdStrLen + 1);

    if ((mnMajorVersion == 1 && mnMinorVersion == 0) || mbBigEndian)
    {
        //*((unsigned int*)aSendBuffer)       = htonl(nSize);
        //*((unsigned int*)(aSendBuffer + 4)) = htonl(nType);

        unsigned int temp = 0;
        temp = htonl(nSize);
        memcpy(aSendBuffer, &temp, sizeof(unsigned int));
        temp = htonl(nType);
        memcpy(aSendBuffer+4, &temp, sizeof(unsigned int));
    }
    else
    {
        //*((unsigned int*)aSendBuffer)       = nSize;
        //*((unsigned int*)(aSendBuffer + 4)) = nType;
        memcpy(aSendBuffer, &nSize, sizeof(int));
        memcpy(aSendBuffer+4, &nType, sizeof(int));
    }

    if (mpoNetwork->Send(aSendBuffer, nSize) == false)
    {
        // Given the definition of GetErrorString, the statement below is just
        // copy maErrorStr to maErrorStr
        //snprintf (maErrorStr, sizeof(maErrorStr), mpoNetwork->GetErrorString());
        return false;
    }

    return true;

} // SendString


bool CRTProtocol::SendCommand(const char* pCmdStr)
{
    return SendString(pCmdStr, CRTPacket::PacketCommand);
} // SendCommand


bool CRTProtocol::SendCommand(const char* pCmdStr, char* pCommandResponseStr, unsigned int nCommandResponseLen)
{
    CRTPacket::EPacketType eType;

    if (SendString(pCmdStr, CRTPacket::PacketCommand))
    {
        if (ReceiveRTPacket(eType, true))
        {
            if (eType == CRTPacket::PacketCommand)
            {
                strncpy(pCommandResponseStr, mpoRTPacket->GetCommandString(), nCommandResponseLen);
                return true;
            }
            if (eType == CRTPacket::PacketError)
            {
                strncpy(pCommandResponseStr, mpoRTPacket->GetErrorString(), nCommandResponseLen);
                return false;
            }
        }
    }
    else
    {
        char pTmpStr[256];
        strncpy(pTmpStr, maErrorStr, sizeof(pTmpStr));
        snprintf(maErrorStr, sizeof(maErrorStr), "\'%s\' command failed. %s", pCmdStr, pTmpStr);
    }
    pCommandResponseStr[0] = 0;
    return false;
} // SendCommand


bool CRTProtocol::SendXML(const char* pCmdStr)
{
    CRTPacket::EPacketType eType;

    if (SendString(pCmdStr, CRTPacket::PacketXML))
    {
        if (ReceiveRTPacket(eType, true))
        {
            if (eType == CRTPacket::PacketCommand)
            {
                if (strcmp(mpoRTPacket->GetCommandString(), "Setting parameters succeeded") == 0)
                {
                    return true;
                }
                else
                {
                    snprintf(maErrorStr, sizeof(maErrorStr),
                        "Expected command response \"Setting parameters succeeded\". Got \"%s\".",
                        mpoRTPacket->GetCommandString());
                }
            }
            else
            {
                snprintf(maErrorStr, sizeof(maErrorStr), "Expected command response packet. Got packet type %d.", (int)eType);
            }
        }
        else
        {
            snprintf(maErrorStr, sizeof(maErrorStr), "Missing command response packet.");
        }
    }
    else
    {
        char pTmpStr[256];
        strncpy(pTmpStr, maErrorStr, sizeof(pTmpStr));
        snprintf(maErrorStr, sizeof(maErrorStr), "Failed to send XML string. %s", pTmpStr);
    }
    return false;
} // SendXML


bool CRTProtocol::GetComponentString(char* pComponentStr, int nComponentStrLen, EComponentType eComponentType)
{
    pComponentStr[0] = 0;

    if (eComponentType & Component2d)
    {
        strncat(pComponentStr, "2D ", nComponentStrLen);
    }
    if (eComponentType & Component2dLin)
    {
        strncat(pComponentStr, "2DLin ", nComponentStrLen);
    }
    if (eComponentType & Component3d)
    {
        strncat(pComponentStr, "3D ", nComponentStrLen);
    }
    if (eComponentType & Component3dRes)
    {
        strncat(pComponentStr, "3DRes ", nComponentStrLen);
    }
    if (eComponentType & Component3dNoLabels)
    {
        strncat(pComponentStr, "3Dnolabels ", nComponentStrLen);
    }
    if (eComponentType & Component3dNoLabelsRes)
    {
        strncat(pComponentStr, "3DNoLabelsRes ", nComponentStrLen);
    }
    if (eComponentType & Component6d)
    {
        strncat(pComponentStr, "6D ",nComponentStrLen);
    }
    if (eComponentType & Component6dRes)
    {
        strncat(pComponentStr,   "6DRes ", nComponentStrLen);
    }
    if (eComponentType & Component6dEuler)
    {
        strncat(pComponentStr, "6DEuler ", nComponentStrLen );
    }
    if (eComponentType & Component6dEulerRes)
    {
        strncat(pComponentStr,  "6DEulerRes ", nComponentStrLen);
    }
    if (eComponentType & ComponentAnalog)
    {
        strncat(pComponentStr,"Analog ", nComponentStrLen);
    }
    if (eComponentType & ComponentAnalogSingle)
    {
        strncat(pComponentStr,  "AnalogSingle ", nComponentStrLen);
    }
    if (eComponentType & ComponentForce)
    {
        strncat(pComponentStr, "Force ", nComponentStrLen );
    }
    if (eComponentType & ComponentForceSingle)
    {
        strncat(pComponentStr,  "ForceSingle ", nComponentStrLen);
    }
    if (eComponentType & ComponentImage)
    {
        strncat(pComponentStr,  "Image ", nComponentStrLen);
    }
    if (eComponentType & ComponentAll)
    {
        strncat(pComponentStr,  "All", nComponentStrLen);
    }

    return (pComponentStr[0] != 0);
}


void CRTProtocol::AddXMLElementBool(CMarkup* oXML, _TCHAR* tTag, const bool* pbValue, _TCHAR* tTrue, _TCHAR* tFalse)
{
    if (pbValue)
    {
        oXML->AddElem(tTag, *pbValue ? tTrue : tFalse);
    }
}


void CRTProtocol::AddXMLElementBool(CMarkup* oXML, _TCHAR* tTag, const bool pbValue, _TCHAR* tTrue, _TCHAR* tFalse)
{
    oXML->AddElem(tTag, pbValue ? tTrue : tFalse);
}


void CRTProtocol::AddXMLElementInt(CMarkup* oXML, _TCHAR* tTag, const int* pnValue)
{
    if (pnValue)
    {
        CStdString tVal;

        tVal.Format("%d", *pnValue);
        oXML->AddElem(tTag, tVal);
    }
}


void CRTProtocol::AddXMLElementUnsignedInt(CMarkup* oXML, _TCHAR* tTag, const unsigned int* pnValue)
{
    if (pnValue)
    {
        CStdString tVal;

        tVal.Format("%u", *pnValue);
        oXML->AddElem(tTag, tVal);
    }
}

void CRTProtocol::AddXMLElementFloat(CMarkup* oXML, _TCHAR* tTag, const float* pfValue, unsigned int pnDecimals)
{
    if (pfValue)
    {
        CStdString tVal;
        char       fFormat[10];

        snprintf(fFormat, sizeof(fFormat), "%%.%df", pnDecimals);
        tVal.Format(fFormat, *pfValue);
        oXML->AddElem(tTag, tVal);
    }
}
