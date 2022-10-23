// n2k message handlers


#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>
#include <N2kMsg.h>
#include <nmea2000Handlers.h>

// These are the n2k messages we look for to see if there are other sources
static const tNMEA2000Handler NMEA2000Handlers[] = {
    {127250L, &Heading},
    {129026L, &COGSOG},
    {129029L, &GNSS},
    {0, 0}};

//*****************************************************************************
template <typename T>
void PrintLabelValWithConversionCheckUnDef(const char *label, T val, double (*ConvFunc)(double val) = 0, bool AddLf = false, int8_t Desim = -1) {
    if(!OutputStream) {
      return;
    }
    OutputStream->print(label);
    if (!N2kIsNA(val)) {
        if (Desim < 0) {
            if (ConvFunc) {
                OutputStream->print(ConvFunc(val));
            } else {
                OutputStream->print(val);
            }
        } else {
            if (ConvFunc) {
                OutputStream->print(ConvFunc(val), Desim);
            } else {
                OutputStream->print(val, Desim);
            }
        }
    } else
        OutputStream->print("not available");
    if (AddLf) OutputStream->println();
}


//*****************************************************************************
void Heading(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    tN2kHeadingReference HeadingReference;
    double Heading;
    double Deviation;
    double Variation;

    if(!OutputStream) {
      return;
    }
    if (ParseN2kHeading(N2kMsg, SID, Heading, Deviation, Variation, HeadingReference)) {
        OutputStream->println("Heading:");
        PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
        OutputStream->print("  reference: ");
        PrintN2kEnumType(HeadingReference, OutputStream);
        PrintLabelValWithConversionCheckUnDef("  Heading (deg): ", Heading, &RadToDeg, true);
        PrintLabelValWithConversionCheckUnDef("  Deviation (deg): ", Deviation, &RadToDeg, true);
        PrintLabelValWithConversionCheckUnDef("  Variation (deg): ", Variation, &RadToDeg, true);
    } else {
        OutputStream->print("Failed to parse PGN: ");
        OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void COGSOG(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    tN2kHeadingReference HeadingReference;
    double COG;
    double SOG;

    if(!OutputStream) {
      return;
    }
    if (ParseN2kCOGSOGRapid(N2kMsg, SID, HeadingReference, COG, SOG)) {
        OutputStream->println("COG/SOG:");
        PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
        OutputStream->print("  reference: ");
        PrintN2kEnumType(HeadingReference, OutputStream);
        PrintLabelValWithConversionCheckUnDef("  COG (deg): ", COG, &RadToDeg, true);
        PrintLabelValWithConversionCheckUnDef("  SOG (m/s): ", SOG, 0, true);
    } else {
        OutputStream->print("Failed to parse PGN: ");
        OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void GNSS(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    uint16_t DaysSince1970;
    double SecondsSinceMidnight;
    double Latitude;
    double Longitude;
    double Altitude;
    tN2kGNSStype GNSStype;
    tN2kGNSSmethod GNSSmethod;
    unsigned char nSatellites;
    double HDOP;
    double PDOP;
    double GeoidalSeparation;
    unsigned char nReferenceStations;
    tN2kGNSStype ReferenceStationType;
    uint16_t ReferenceSationID;
    double AgeOfCorrection;

    if(!OutputStream) {
      return;
    }
    if (ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight,
                     Latitude, Longitude, Altitude,
                     GNSStype, GNSSmethod,
                     nSatellites, HDOP, PDOP, GeoidalSeparation,
                     nReferenceStations, ReferenceStationType, ReferenceSationID,
                     AgeOfCorrection)) {
        OutputStream->println("GNSS info:");
        PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
        PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ", DaysSince1970, 0, true);
        PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ", SecondsSinceMidnight, 0, true);
        PrintLabelValWithConversionCheckUnDef("  latitude: ", Latitude, 0, true, 9);
        PrintLabelValWithConversionCheckUnDef("  longitude: ", Longitude, 0, true, 9);
        PrintLabelValWithConversionCheckUnDef("  altitude: (m): ", Altitude, 0, true);
        OutputStream->print("  GNSS type: ");
        PrintN2kEnumType(GNSStype, OutputStream);
        OutputStream->print("  GNSS method: ");
        PrintN2kEnumType(GNSSmethod, OutputStream);
        PrintLabelValWithConversionCheckUnDef("  satellite count: ", nSatellites, 0, true);
        PrintLabelValWithConversionCheckUnDef("  HDOP: ", HDOP, 0, true);
        PrintLabelValWithConversionCheckUnDef("  PDOP: ", PDOP, 0, true);
        PrintLabelValWithConversionCheckUnDef("  geoidal separation: ", GeoidalSeparation, 0, true);
        PrintLabelValWithConversionCheckUnDef("  reference stations: ", nReferenceStations, 0, true);
    } else {
        OutputStream->print("Failed to parse PGN: ");
        OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
    int iHandler;

    // Find handler
    for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++)
        ;

    if (NMEA2000Handlers[iHandler].PGN != 0) {
        NMEA2000Handlers[iHandler].Handler(N2kMsg);
    }
}
