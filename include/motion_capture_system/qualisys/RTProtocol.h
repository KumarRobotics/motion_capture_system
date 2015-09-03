#ifndef RTPROTOCOL_H
#define RTPROTOCOL_H
#include "RTPacket.h"
#include "StdString.h"

#include <vector>
#include "Network.h"


#ifdef EXPORT_DLL
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif
// added some unicode defintion for Linux probably not all are neccesssary
#ifdef UNICODE

#define _tcslen     wcslen
#define _tcscpy     wcscpy
#define _tcscpy_s   wcscpy_s
#define _tcsncpy    wcsncpy
#define _tcsncpy_s  wcsncpy_s
#define _tcscat     wcscat
#define _tcscat_s   wcscat_s
#define _tcsupr     wcsupr
#define _tcsupr_s   wcsupr_s
#define _tcslwr     wcslwr
#define _tcslwr_s   wcslwr_s

#define _stprintf_s swprintf_s
#define _stprintf   swprintf
#define _tprintf    wprintf

#define _vstprintf_s    vswprintf_s
#define _vstprintf      vswprintf

#define _tscanf     wscanf


#define _TCHAR wchar_t

#else

#define _tcslen     strlen
#define _tcscpy     strcpy
#define _tcscpy_s   strcpy_s
#define _tcsncpy    strncpy
#define _tcsncpy_s  strncpy_s
#define _tcscat     strcat
#define _tcscat_s   strcat_s
#define _tcsupr     strupr
#define _tcsupr_s   strupr_s
#define _tcslwr     strlwr
#define _tcslwr_s   strlwr_s

#define _stprintf_s sprintf_s
#define _stprintf   sprintf
#define _tprintf    printf

#define _vstprintf_s    vsprintf_s
#define _vstprintf      vsprintf

#define _tscanf     scanf

#define _TCHAR const char
#endif

#define DEFAULT_AUTO_DESCOVER_PORT 22226
#define WAIT_FOR_DATA_TIMEOUT      2000000 // micro seconds

class CMarkup;

class DLL_EXPORT CRTProtocol
{
public:
    enum EStreamRate
    {
        RateAllFrames        = 1,
        RateFrequency        = 2,
        RateFrequencyDivisor = 3
    };

    enum EComponentType
    {
        Component3d            = 0x0001,
        Component3dNoLabels    = 0x0002,
        ComponentAnalog        = 0x0004,
        ComponentForce         = 0x0008,
        Component6d            = 0x0010,
        Component6dEuler       = 0x0020,
        Component2d            = 0x0040,
        Component2dLin         = 0x0080,
        Component3dRes         = 0x0100,
        Component3dNoLabelsRes = 0x0200,
        Component6dRes         = 0x0400,
        Component6dEulerRes    = 0x0800,
        ComponentAnalogSingle  = 0x1000,
        ComponentImage         = 0x2000,
        ComponentForceSingle   = 0x4000,
        ComponentAll           = 0x8000
    };

    enum ECameraModel
    {
        ModelMacReflex     = 0,
        ModelProReflex120  = 1,
        ModelProReflex240  = 2,
        ModelProReflex500  = 3,
        ModelProReflex1000 = 4,
        ModelOqus100       = 5,
        ModelOqus300       = 6,
        ModelOqus300Plus   = 7,
        ModelOqus400       = 8,
        ModelOqus500       = 9,
        ModelOqus200       = 10,
        ModelOqus500Plus   = 11
    };

    enum ECameraMode
    {
        ModeMarker          = 0,
        ModeMarkerIntensity = 1,
        ModeVideo           = 2
    };

    enum ESyncOutFreqMode
    {
        ModeShutterOut = 1, // A pulse per frame is sent
        ModeMultiplier,
        ModeDivisor,
        ModeActualFreq,
        ModeMeasurementTime,
        ModeSRAMWireSync,
        ModeFixed100Hz
    };

    enum ESignalSource
    {
        SourceControlPort = 0,
        SourceIRReceiver  = 1,
        SourceSMPTE       = 2,
        SourceVideoSync   = 3
    };

    enum EAxis
    {
        XPos = 0,
        XNeg = 1,
        YPos = 2,
        YNeg = 3,
        ZPos = 4,
        ZNeg = 5
    };

    enum EProcessingActions
    {
        ProcessingNone               = 0x0000,
        ProcessingTracking2D         = 0x0001,
        ProcessingTracking3D         = 0x0002,
        ProcessingTwinSystemMerge    = 0x0004,
        ProcessingSplineFill         = 0x0008,
        ProcessingAIM                = 0x0010,
        Processing6DOFTracking       = 0x0020,
        ProcessingForceData          = 0x0040,
        ProcessingExportTSV          = 0x0080,
        ProcessingExportC3D          = 0x0100,
        ProcessingExportDiff         = 0x0200,
        ProcessingExportMatlabDirect = 0x0400,
        ProcessingExportMatlabFile   = 0x0800
    };

    struct SPoint
    {
        float fX;
        float fY;
        float fZ;
    };

    struct SDiscoverResponse
    {
        char           pMessage[128];
        unsigned int   nAddr;
        unsigned short nBasePort;
    };


private:
    struct SSettingsGeneralCamera
    {
        unsigned int nID;
        ECameraModel eModel;
        bool         bUnderwater;
        unsigned int nSerial;
        ECameraMode  eMode;
        unsigned int nVideoExposure;      // Micro seconds
        unsigned int nVideoExposureMin;   // Micro seconds
        unsigned int nVideoExposureMax;   // Micro seconds
        unsigned int nVideoFlashTime;     // Micro seconds
        unsigned int nVideoFlashTimeMin;  // Micro seconds
        unsigned int nVideoFlashTimeMax;  // Micro seconds
        unsigned int nMarkerExposure;     // Micro seconds
        unsigned int nMarkerExposureMin;  // Micro seconds
        unsigned int nMarkerExposureMax;  // Micro seconds
        unsigned int nMarkerThreshold;
        unsigned int nMarkerThresholdMin;
        unsigned int nMarkerThresholdMax;
        float        fPositionX;
        float        fPositionY;
        float        fPositionZ;
        float        fPositionRotMatrix[3][3];
        unsigned int nOrientation;             // Degrees
        unsigned int nMarkerResolutionWidth;   // Sub pixels
        unsigned int nMarkerResolutionHeight;  // Sub pixels
        unsigned int nVideoResolutionWidth;    // Pixels
        unsigned int nVideoResolutionHeight;   // Pixels
        unsigned int nMarkerFOVLeft;           // Pixels
        unsigned int nMarkerFOVTop;            // Pixels
        unsigned int nMarkerFOVRight;          // Pixels
        unsigned int nMarkerFOVBottom;         // Pixels
        unsigned int nVideoFOVLeft;            // Pixels
        unsigned int nVideoFOVTop;             // Pixels
        unsigned int nVideoFOVRight;           // Pixels
        unsigned int nVideoFOVBottom;          // Pixels
        ESyncOutFreqMode eSyncOutMode;
        unsigned int nSyncOutValue;
        float        fSyncOutDutyCycle;        // Percent
        bool         bSyncOutNegativePolarity;
    };

    struct SSettingsGeneralExternalTimebase
    {
        bool          bEnabled;
        ESignalSource eSignalSource;
        bool          bSignalModePeriodic;
        unsigned int  nFreqMultiplier;
        unsigned int  nFreqDivisor;
        unsigned int  nFreqTolerance;
        float         fNominalFrequency;
        bool          bNegativeEdge;
        unsigned int  nSignalShutterDelay;
        float         fNonPeriodicTimeout;
    };

    struct SSettingsGeneral
    {
        unsigned int                          nCaptureFrequency;
        float                                 fCaptureTime;
        bool                                  bStartOnExternalTrigger;
        SSettingsGeneralExternalTimebase      sExternalTimebase;
        EProcessingActions                    eProcessingActions; // Binary flags.
        std::vector< SSettingsGeneralCamera > vsCameras;
    };

    struct SSettings3DLabel
    {
        CStdStringA  oName;
        unsigned int nRGBColor;
    };

    struct SSettings3D
    {
        EAxis                           eAxisUpwards;
        char                            pCalibrationTime[32];
        std::vector< SSettings3DLabel > s3DLabels;
    };

    struct SSettings6DOF
    {
        CStdStringA           oName;
        unsigned int          nRGBColor;
        std::vector< SPoint > vsPoints;
    };

    struct SAnalogDevice
    {
        unsigned int               nDeviceID;
        unsigned int               nChannels;
        CStdStringA                oName;
        std::vector< CStdStringA > voLabels;
        unsigned int               nFrequency;
        CStdStringA                oUnit;
        unsigned int               nMinRange;
        unsigned int               nMaxRange;
    };

    struct SForceChannel
    {
        unsigned int nChannelNumber;
        float        fConversionFactor;
    };

    struct SForcePlate
    {
        unsigned int                 nID;
        unsigned int                 nAnalogDeviceID;
        CStdStringA                  oType;
        CStdStringA                  oName;
        unsigned int                 nFrequency;
        float                        fLength;
        float                        fWidth;
        SPoint                       asCorner[4];
        SPoint                       sOrigin;
        std::vector< SForceChannel > vChannels;
        bool                         bValidCalibrationMatrix;
        float                        afCalibrationMatrix[6][6];
    };

    struct SSettingsForce
    {
        CStdStringA                 oUnitLength;
        CStdStringA                 oUnitForce;
        std::vector< SForcePlate >  vsForcePlates;
    };

    struct SImageCamera
    {
        unsigned int            nID;
        bool                    bEnabled;
        CRTPacket::EImageFormat eFormat;
        unsigned int            nWidth;
        unsigned int            nHeight;
        float                   fCropLeft;
        float                   fCropTop;
        float                   fCropRight;
        float                   fCropBottom;
    };


public:
    CRTProtocol();
    ~CRTProtocol();
    bool       Connect(char* pServerAddr, int nPort, int nUDPServerPort = 0, int nMajorVersion = MAJOR_VERSION,
                       int nMinorVersion = MINOR_VERSION, bool bBigEndian = IS_BIG_ENDIAN);
    void       Disconnect();
    bool       Connected();
    bool       SetVersion(int nMajorVersion, int nMinorVersion);
    bool       GetVersion(int &nMajorVersion, int &nMinorVersion);
    bool       GetQTMVersion(char* pVersion, unsigned int nVersionLen);
    bool       GetByteOrder(bool &bBigEndian);
    bool       CheckLicense(char* pLicenseCode);
    bool       DiscoverRTServer(short nServerPort, bool bNoLocalResponses, short nDiscoverPort = DEFAULT_AUTO_DESCOVER_PORT);
    int        GetNumberOfDiscoverResponses();
    bool       GetDiscoverResponse(unsigned int nIndex, unsigned int &nAddr, unsigned short &nBasePort,
                                   bool &bLocalResponse, char* pMessage, int nMessageLen);

    bool       GetCurrentFrame(EComponentType eComponentType);
    bool       StreamFrames(EStreamRate eRate, unsigned int nRateArg, unsigned short nUDPPort, const char* pUDPAddr,
                            EComponentType eComponentType);
    bool       StreamFramesStop();
    bool       GetState(CRTPacket::EEvent &eEvent, bool bUpdate = true);
    bool       GetCapture(char* pFileName, bool bC3D);
    bool       SendTrig();
    bool       SetQTMEvent(char* pLabel);
    bool       TakeControl(char* pPassword = NULL);
    bool       ReleaseControl();
    bool       NewMeasurement();
    bool       CloseMeasurement();
    bool       StartCapture();
    bool       StopCapture();
    bool       SaveCapture(char* pFileName, bool bOverwrite);

    CRTPacket* ReceiveRTPacket(CRTPacket::EPacketType &eType, bool bSkipEvents = true,
                               int nTimeout = WAIT_FOR_DATA_TIMEOUT);    // nTimeout < 0 : Blocking receive
    CRTPacket* GetRTPacket();

    bool       ReadGeneralSettings();
    bool       Read3DSettings();
    bool       Read6DOFSettings();
    bool       ReadAnalogSettings();
    bool       ReadForceSettings();
    bool       ReadImageSettings();


    void         GetGeneral(unsigned int &nCaptureFrequency, float              &fCaptureTime,
                            bool         &bStartOnExtTrig,   EProcessingActions &eProcessingActions);
    void         GetGeneralExtTimeBase(bool         &bEnabled,            ESignalSource &eSignalSource,
                                       bool         &bSignalModePeriodic, unsigned int  &nFreqMultiplier,
                                       unsigned int &nFreqDivisor,        unsigned int  &nFreqTolerance,
                                       float        &fNominalFrequency,   bool          &bNegativeEdge,
                                       unsigned int &nSignalShutterDelay, float         &fNonPeriodicTimeout);
    unsigned int GetGeneralCameraCount();
    bool         GetGeneralCamera(unsigned int nCameraIndex, unsigned int &nID,     ECameraModel &eModel,
                                  bool         &bUnderwater, unsigned int &nSerial, ECameraMode  &eMode);
    bool         GetGeneralCameraMarker(unsigned int nCameraIndex, unsigned int &nCurrentExposure,
                                        unsigned int &nMinExposure, unsigned int &nMaxExposure,
                                        unsigned int &nCurrentThreshold, unsigned int &nMinThreshold,
                                        unsigned int &nMaxThreshold);
    bool         GetGeneralCameraVideo(unsigned int nCameraIndex, unsigned int &nCurrentExposure,
                                       unsigned int &nMinExposure, unsigned int &nMaxExposure,
                                       unsigned int &nCurrentFlashTime, unsigned int &nMinFlashTime,
                                       unsigned int &nMaxFlashTime);
    bool         GetGeneralCameraPosition(unsigned int nCameraIndex, SPoint &sPoint, float fvRotationMatrix[3][3]);
    bool         GetGeneralCameraOrientation(unsigned int nCameraIndex, int &nOrientation);
    bool         GetGeneralCameraResolution(unsigned int nCameraIndex, unsigned int &nMarkerWidth,
                                            unsigned int &nMarkerHeight, unsigned int &nVideoWidth,
                                            unsigned int &nVideoHeight);
    bool         GetGeneralCameraFOV(unsigned int nCameraIndex,  unsigned int &nMarkerLeft,  unsigned int &nMarkerTop,
                                     unsigned int &nMarkerRight, unsigned int &nMarkerBottom,
                                     unsigned int &nVideoLeft,   unsigned int &nVideoTop,
                                     unsigned int &nVideoRight,  unsigned int &nVideoBottom);
    bool         GetGeneralCameraSyncOut(unsigned int nCameraIndex,   ESyncOutFreqMode &eSyncOutMode,
                                         unsigned int &nSyncOutValue, float            &fSyncOutDutyCycle,
                                         bool         &bSyncOutNegativePolarity);

    EAxis        Get3DUpwardAxis();
    char*        Get3DCalibrated();
    unsigned int Get3DLabeledMarkerCount();
    const char*  Get3DLabelName(unsigned int nMarkerIndex);
    unsigned int Get3DLabelColor(unsigned int nMarkerIndex);

    unsigned int Get6DOFBodyCount();
    const char*  Get6DOFBodyName(unsigned int nBodyIndex);
    unsigned int Get6DOFBodyColor(unsigned int nBodyIndex);
    unsigned int Get6DOFBodyPointCount(unsigned int nBodyIndex);
    bool         Get6DOFBodyPoint(unsigned int nBodyIndex, unsigned int nMarkerIndex, SPoint &sPoint);

    unsigned int GetAnalogDeviceCount();
    bool         GetAnalogDevice(unsigned int nDeviceIndex, unsigned int &nDeviceID, unsigned int &nChannels,
                                 char* &pName, unsigned int &nFrequency, char* &pUnit,
                                 unsigned int &nMinRange, unsigned int &nMaxRange);
    const char*  GetAnalogLabel(unsigned int nDeviceIndex, unsigned int nLabelIndex);

    void         GetForceUnits(char* &pLength, char* &pForce);
    unsigned int GetForcePlateCount();
    bool         GetForcePlate(unsigned int nPlateIndex, unsigned int &nID, unsigned int &nAnalogDeviceID,
                               unsigned int &nFrequency, char* &pType, char* &pName, float &fLength, float &fWidth);
    bool         GetForcePlateLocation(unsigned int nPlateIndex, SPoint sCorner[4]);
    bool         GetForcePlateOrigin(unsigned int nPlateIndex, SPoint &sOrigin);
    unsigned int GetForcePlateChannelCount(unsigned int nPlateIndex);
    bool         GetForcePlateChannel(unsigned int nPlateIndex, unsigned int nChannelIndex,
                                      unsigned int &nChannelNumber, float &fConversionFactor);
    bool         GetForcePlateCalibrationMatrix(unsigned int nPlateIndex, float fvCalMatrix[6][6]);

    unsigned int GetImageCameraCount();
    bool         GetImageCamera(unsigned int nCameraIndex, unsigned int &nCameraID, bool &bEnabled,
                                CRTPacket::EImageFormat &eFormat, unsigned int &nWidth, unsigned int &nHeight,
                                float &fCropLeft, float &fCropTop, float &fCropRight, float &fCropBottom);


    bool SetGeneral(const unsigned int* nCaptureFrequency, const float*              fCaptureTime,
                    const bool*         bStartOnExtTrig,   const EProcessingActions* eProcessingActions);

    bool SetGeneralExtTimeBase(const bool*         bEnabled,            const ESignalSource* eSignalSource,
                               const bool*         bSignalModePeriodic, const unsigned int*  nFreqMultiplier,
                               const unsigned int* nFreqDivisor,        const unsigned int*  nFreqTolerance,
                               const float*        fNominalFrequency,   const bool*          bNegativeEdge,
                               const unsigned int* nSignalShutterDelay, const float*         fNonPeriodicTimeout);

    bool SetGeneralCamera(const unsigned int nCameraID,       const ECameraMode* eMode,           const float* fVideoExposure,
                          const float*       fVideoFlashTime, const float*       fMarkerExposure, const float* fMarkerThreshold,
                          const int*         nOrientation);

    bool SetGeneralCameraSyncOut(const unsigned int  nCameraID,      const ESyncOutFreqMode* peSyncOutMode,
                                 const unsigned int* pnSyncOutValue, const float*            pfSyncOutDutyCycle,
                                 const bool*         pbSyncOutNegativePolarity);

    bool SetImageSettings(const unsigned int  nCameraID, const bool*         bEnable,    const CRTPacket::EImageFormat* eFormat,
                          const unsigned int* nWidth,    const unsigned int* nHeight,    const float* fLeftCrop,
                          const float*        fTopCrop,  const float*        fRightCrop, const float* fBottomCrop);

    bool SetForceSettings(const unsigned int nPlateID, const SPoint* sCorner1, const SPoint* sCorner2,
                          const SPoint*      sCorner3, const SPoint* sCorner4);

    char* GetErrorString();


private:
    bool SendString(const char* pCmdStr, int nType);
    bool SendCommand(const char* pCmdStr);
    bool SendCommand(const char* pCmdStr, char* pCommandResponseStr, unsigned int nCommandResponseLen);
    bool SendXML(const char* pCmdStr);
    bool GetComponentString(char* pComponentStr, int nComponentStrLen, EComponentType eComponentType);
    void AddXMLElementBool(CMarkup* oXML, _TCHAR* tTag, const bool* pbValue,
                           _TCHAR* tTrue = ("True"), _TCHAR* tFalse = ("False"));
    void AddXMLElementBool(CMarkup* oXML, _TCHAR* tTag, const bool bValue,
                           _TCHAR* tTrue = ("True"), _TCHAR* tFalse = ("False"));
    void AddXMLElementInt(CMarkup* oXML, _TCHAR* tTag, const int* pnValue);
    void AddXMLElementUnsignedInt(CMarkup* oXML, _TCHAR* tTag, const unsigned int* pnValue);
    void AddXMLElementFloat(CMarkup* oXML, _TCHAR* tTag, const float* pfValue, unsigned int pnDecimals = 6);

private:
    COutput*                      mpoOutput;
    CNetwork*                     mpoNetwork;
    CRTPacket*                    mpoRTPacket;
    char                          maDataBuff[65536];
    CRTPacket::EEvent             meLastEvent;
    int                           mnMinorVersion;
    int                           mnMajorVersion;
    bool                          mbBigEndian;
    SSettingsGeneral              msGeneralSettings;
    SSettings3D                   ms3DSettings;
    std::vector< SSettings6DOF >  mvs6DOFBodySettings;
    std::vector< SAnalogDevice >  mvsAnalogDeviceSettings;
    SSettingsForce                msForceSettings;
    std::vector< SImageCamera >   mvsImageSettings;
    char                          maErrorStr[256];
    bool                          bBroadcastSocketCreated;
    FILE*                         mpFileBuffer;
    std::vector< SDiscoverResponse > mvsDiscoverResponseList;
};


#endif // RTPROTOCOL_H
