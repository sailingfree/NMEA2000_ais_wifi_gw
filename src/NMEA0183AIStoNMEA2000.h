/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once 

#include <Arduino.h>
#include "ais_decoder.h"
#include "default_sentence_parser.h"
#include <N2kTypes.h>
//#include <GwDefs.h>

class MyAisDecoder : public AIS::AisDecoder
{
  public:
    MyAisDecoder();

  protected:
    virtual void onType123(unsigned int _uMsgType, unsigned int _uMmsi, unsigned int _uNavstatus,
                           int _iRot, unsigned int _uSog, bool _bPosAccuracy,
                           long _iPosLon, long _iPosLat, int _iCog, int _iHeading, int _Repeat, bool _Raim,
                           unsigned int _timestamp, unsigned int _maneuver_i) override;
                           
    virtual void onType411(unsigned int , unsigned int , unsigned int , unsigned int , unsigned int , unsigned int , unsigned int , unsigned int , bool , int , int ) override;

    virtual void onType5(unsigned int _uMsgType, unsigned int _uMmsi, unsigned int _uImo, const std::string &_strCallsign,
                         const std::string &_strName,
                         unsigned int _uType, unsigned int _uToBow, unsigned int _uToStern,
                         unsigned int _uToPort, unsigned int _uToStarboard, unsigned int _uFixType,
                         unsigned int _uEtaMonth, unsigned int _uEtaDay, unsigned int _uEtaHour,
                         unsigned int _uEtaMinute, unsigned int _uDraught,
                         const std::string &_strDestination, unsigned int _ais_version,
                         unsigned int _repeat, bool _dte) override;
                         
    virtual void onType9(unsigned int , unsigned int , bool , int , int , int , unsigned int ) override;
    
    virtual void onType14(unsigned int _repeat, unsigned int _uMmsi,
                          const std::string &_strText, int _iPayloadSizeBits) override;
                          
    virtual void onType18(unsigned int _uMsgType, unsigned int _uMmsi, unsigned int _uSog, bool _bPosAccuracy,
                          long _iPosLon, long _iPosLat, int _iCog, int _iHeading, bool _raim, unsigned int _repeat,
                          bool _unit, bool _diplay, bool _dsc, bool _band, bool _msg22, bool _assigned,
                          unsigned int _timestamp, bool _state ) override;
                          
    virtual void onType19(unsigned int _uMmsi, unsigned int _uSog, bool _bPosAccuracy, int _iPosLon, int _iPosLat,
                          int _iCog, int _iHeading, const std::string &_strName, unsigned int _uType,
                          unsigned int _uToBow, unsigned int _uToStern, unsigned int _uToPort,
                          unsigned int _uToStarboard, unsigned int _timestamp, unsigned int _fixtype,
                          bool _dte, bool _assigned, unsigned int _repeat, bool _raim) override;
                          
    virtual void onType21(unsigned int , unsigned int , const std::string &, bool , int , int , unsigned int , 
              unsigned int , unsigned int , unsigned int ) override;
              
    virtual void onType24A(unsigned int _uMsgType, unsigned int _repeat, unsigned int _uMmsi,
                           const std::string &_strName) override;
                           
    virtual void onType24B(unsigned int _uMsgType, unsigned int _repeat, unsigned int _uMmsi,
                           const std::string &_strCallsign, unsigned int _uType,
                           unsigned int _uToBow, unsigned int _uToStern, unsigned int _uToPort,
                           unsigned int _uToStarboard, const std::string &_strVendor) override;
                           
    virtual void onType27(unsigned int , unsigned int , unsigned int , bool , int , int , int ) override;
    
    virtual void onSentence(const AIS::StringRef &_Stc) override;
    
    virtual void onMessage(const AIS::StringRef &, const AIS::StringRef &, const AIS::StringRef &) override;
    
    virtual void onNotDecoded(const AIS::StringRef &, int ) override;
    
    virtual void onDecodeError(const AIS::StringRef &_strMessage, const std::string &_strError) override;
    
    virtual void onParseError(const AIS::StringRef &_strMessage, const std::string &_strError) override;
};
