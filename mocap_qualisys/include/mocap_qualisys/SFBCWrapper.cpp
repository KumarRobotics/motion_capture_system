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
#include <stdio.h>
#include "SFBCWrapper.h"

SFBCWrapper::SFBCWrapper(TiXmlElement *source)
{
	cnct=0;data=0;

	initSFBcomm(source);

	cnct=new connection(localPort);
	if(remoteHost&&(remotePort!=-1))
		cnct->connect(const_cast<char *>(remoteHost),remotePort,streamID);
	else
		cnct->addreceiver("any",0,streamID);

	data=new datatype;
}

SFBCWrapper::~SFBCWrapper(void)
{
	if(cnct)
		delete cnct;

	if(data)
		delete data;
}

void SFBCWrapper::print(FILE *file,const char *filler)
{
	fprintf(file,"%s- SFBcomm:\n",filler);
	fprintf(file,"%s    localPort:  %d\n",filler,localPort);
	if(remoteHost&&(remotePort!=-1))
	{
		fprintf(file,"%s    remoteHost: %s\n",filler,remoteHost);
		fprintf(file,"%s    remotePort: %d\n",filler,remotePort);
	}
	fprintf(file,"%s    streamID:   %d\n",filler,streamID);
}

void SFBCWrapper::addLong(const char *name,long *ptr,int compressID)
{
	data->createlong(name,ptr,compressID);
}

void SFBCWrapper::addBuffer(const char *name,unsigned char *ptr,long size)
{
	data->createbuffer(name,ptr,size);
}

void SFBCWrapper::addFrame(const char *name,int width,int height,int colorSpace,int compressID)
{
	data->createframe(name,width,height,colorSpace,compressID);
}

unsigned char *SFBCWrapper::getFrame(const char *name)
{
	return data->getframe(name);
}

int  SFBCWrapper::receiveData(bool unbuf)
{
	if(cnct->checkstream(streamID)!=-1)
	{
		if(unbuf)
			return cnct->receivedataunbuf(data,streamID);
		else
			return cnct->receivedata(data,streamID);
	}
	else
		return -1;
}

int  SFBCWrapper::sendData(void)
{
	if(cnct->checkstream(streamID)!=-1)
		return cnct->senddata(data,streamID);
	else
		return -1;
}

int SFBCWrapper::getLocalPort(void)
{
	return localPort;
}

const char *SFBCWrapper::getRemoteHost(void)
{
	return remoteHost;
}

int SFBCWrapper::getRemotePort(void)
{
	return remotePort;
}

int SFBCWrapper::getStreamID(void)
{
	return streamID;
}

void SFBCWrapper::initSFBcomm(TiXmlElement *source)
{
	if(source->QueryIntAttribute("localPort",&localPort)!=TIXML_SUCCESS)
		throw "Attribute 'localPort' of <SFBcomm> missing";

	remoteHost=source->Attribute("remoteHost");

	if(source->QueryIntAttribute("remotePort",&remotePort)!=TIXML_SUCCESS)
		remotePort=-1;

	if(source->QueryIntAttribute("streamID",&streamID)!=TIXML_SUCCESS)
		throw "Attribute 'streamID' of <SFBcomm> missing";
}
