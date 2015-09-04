/*****************************************************************************/
/*                                                                           */
/* Collaborative Research Centre of the DFG:                                 */
/*                                                                           */
/*   SFB453 High-Fidelity Telepresence and Teleaction                        */
/*                                                                           */
/*   Project I1: Integrated Mobile and Bimanual Multi-User Telepresence and  */
/*               Teleaction Systems with Haptic Feedback                     */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Camera live video streaming client for LSR stereo camera heads            */
/*                                                                           */
/* Organization: Institute of Automatic Control Engineering (LSR)            */
/* Authors:      Ulrich Unterhinninghofen                                    */
/*               Peter Hinterseer (LKN)                                      */
/* Contact:      mailto:ulrich.unterhinninghofen@tum.de                      */
/*                                                                           */
/* (C) 2006 - 2007, Technische Universitaet Muenchen. All rights reserved.   */
/*                                                                           */
/*****************************************************************************/
#ifndef __MP_SFBC_WRAPPER__
#define __MP_SFBC_WRAPPER__

#include "tinyxml.h"
#include "connection.h"
using namespace sfbcomm;

class SFBCWrapper
{
public:
	SFBCWrapper(TiXmlElement *source);
	virtual ~SFBCWrapper(void);

	void print(FILE *file,const char *filler="");

	void           addLong    (const char *name,long *ptr,int compressID=0);
	void           addBuffer  (const char *name,unsigned char *ptr,long size);
	void           addFrame   (const char *name,int width,int height,int colorSpace,int compressID=0);
	unsigned char *getFrame   (const char *name);
	int            receiveData(bool unbuf=true);
	int            sendData   (void);

	int         getLocalPort (void);
	const char *getRemoteHost(void);
	int         getRemotePort(void);
	int         getStreamID  (void);

protected:
	void initSFBcomm(TiXmlElement *source);

	connection *cnct;
	datatype   *data;

	int         localPort;
	const char *remoteHost;
	int         remotePort;
	int         streamID;
};

#endif
