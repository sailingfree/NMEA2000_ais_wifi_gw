#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5 (Caution!!! Pin 2 before)
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4

#include <Arduino.h>
#include <NMEA2000.h>
#include <NMEA2000_CAN.h>
#include <MyN2k.h>
#include <N2ktoYD.h>
#include <N2kMessages.h>
#include <N2kMsg.h>
#include <N2kDeviceList.h>
#include <oled_func.h>
#include <oled_func.h>
#include <GwPrefs.h>
#include <list>
#include <map>
#include <bmp180_functions.h>
#include <GwDefs.h>

#define MiscSendOffset 120
#define VerySlowDataUpdatePeriod 10000  // temperature etc
#define SlowDataUpdatePeriod 1000       // Time between CAN Messages sent
#define FastDataUpdatePeriod 100        // Fast data period



/////// Variables
using namespace std;

// List of n2k devices for the device scanner
tN2kDeviceList *pN2kDeviceList;


// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {127489L,  // Engine dynamic
                                                  127488L,  // Engine fast dynamic
                                                  127250,   // Vessel heading
                                                  130577,   // Direction data
                                                  130310,   // Outside environmental
                                                  0};
const unsigned long ReceiveMessages[] PROGMEM = {/*126992L,*/  // System time
                                                 127250L,      // Heading
                                                 127258L,      // Magnetic variation
                                                 128259UL,     // Boat speed
                                                 128267UL,     // Depth
                                                 129025UL,     // Position
                                                 129026L,      // COG and SOG
                                                 129029L,      // GNSS
                                                 130306L,      // Wind
                                                 128275UL,     // Log
                                                 127245UL,     // Rudder
                                                 0};



// Handle some other gw messages from other gw nodes
void handle_gw_msgs(const tN2kMsg &N2kMsg) {
    ulong PGN = N2kMsg.PGN;
    switch (PGN) {
        case 127508L:   // Battery
        case 127488:    // Engine rapid
        case 130306:    // Wind
        case 129026:    // SOG/COG
        case 128267:    // Depth
        case 129029:    // GNSS
        case 129540:    // GNSS sats in view
        case 130310:    // Outside environment
        case 130312:    // Temperature
        case 130313:    // Humidity
        case 130314:    // Pressure
        default:
            GwSendYD(N2kMsg);
            break;
    }
}




void initN2k(uint32_t id) {
   // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega

    NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.SetN2kCANSendFrameBufSize(250);

    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 50);
    pN2kDeviceList = new tN2kDeviceList(&NMEA2000);

    NMEA2000.SetProductInformation(hostName.c_str(),     // Manufacturer's Model serial code
                                   100,                   // Manufacturer's product code
                                   modelName.c_str(),         // Manufacturer's Model ID
                                   "1.0.0 (2021-06-11)",  // Manufacturer's Software version code
                                   "1.0.0 (2021-06-11)"   // Manufacturer's Model version
    );
    // Set device information
    NMEA2000.SetDeviceInformation(id,   // Unique number. Use e.g. Serial number. Id is generated from MAC-Address
                                  130,  // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  25,   // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  2046  // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
    );

    oledPrintf(0, lineh * 2, "My ID 0x%x", id);

    NMEA2000.SetConfigurationInformation("Naiad ",
                                         "Must be installed internally, not water or dust proof.",
                                         "Connect AIS NMEA at 34000");

    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);  // Show in clear text. Leave uncommented for default Actisense format.

    NMEA2000.SetMsgHandler(handle_gw_msgs);

    nodeAddress = gwGetVal(LASTNODEADDRESS, "32").toInt();

    Console->printf("NodeAddress=%d\n", nodeAddress);

    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, nodeAddress);

    NMEA2000.ExtendTransmitMessages(TransmitMessages);
    NMEA2000.ExtendReceiveMessages(ReceiveMessages);
    NMEA2000.Open();
}

bool IsTimeToUpdate(unsigned long NextUpdate) {
    return (NextUpdate < millis());
}
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset = 0) {
    return millis() + Period + Offset;
}

void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period) {
    while (NextUpdate < millis()) NextUpdate += Period;
}

void SendN2kEngineSlow() {
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, MiscSendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(SlowDataUpdated)) {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

        SetN2kEngineDynamicParam(N2kMsg, 0,
                                 N2kDoubleNA,      // Oil Pressure
                                 N2kDoubleNA,      // Oil temp
                                 CToKelvin(temp),  // Coolant temp
                                 voltage,          // alternator voltage
                                 N2kDoubleNA,      // fule rate
                                 N2kDoubleNA,      // engine hours
                                 N2kDoubleNA,      // coolant pressure
                                 N2kDoubleNA,      // fuel pressure
                                 N2kInt8NA, N2kInt8NA, true);
        NMEA2000.SendMsg(N2kMsg);
        GwSendYD(N2kMsg);

        mapSensors["Voltage"] = String(voltage);
    }
}

//*****************************************************************************
void PrintUlongList(const char *prefix, const unsigned long *List, Stream &stream) {
    uint8_t i;
    if (List != 0) {
        stream.printf(prefix);
        for (i = 0; List[i] != 0; i++) {
            if (i > 0) stream.print(", ");
            stream.printf("%lud", List[i]);
        }
        stream.println();
    }
}

//*****************************************************************************
void PrintText(const char *Text, bool AddLineFeed, Stream &stream) {
    if (Text != 0)
        stream.print(Text);
    if (AddLineFeed)
        stream.println();
}

//*****************************************************************************
void PrintDevice(const tNMEA2000::tDevice *pDevice, Stream &stream) {
    if (pDevice == 0) return;

    stream.printf("----------------------------------------------------------------------\n");
    stream.printf("%s\n", pDevice->GetModelID());
    stream.printf("  Source: %d\n", pDevice->GetSource());
    stream.printf("  Manufacturer code:        %d\n", pDevice->GetManufacturerCode());
    stream.printf("  Serial Code:              %s\n", pDevice->GetModelSerialCode());
    stream.printf("  Unique number:            %d\n", pDevice->GetUniqueNumber());
    stream.printf("  Software version:         %s\n", pDevice->GetSwCode());
    stream.printf("  Model version:            %s\n", pDevice->GetModelVersion());
    stream.printf("  Manufacturer Information: ");
    PrintText(pDevice->GetManufacturerInformation(), true, stream);
    stream.printf("  Installation description1: ");
    PrintText(pDevice->GetInstallationDescription1(), true, stream);
    stream.printf("  Installation description2: ");
    PrintText(pDevice->GetInstallationDescription2(), true, stream);
    PrintUlongList("  Transmit PGNs :", pDevice->GetTransmitPGNs(), stream);
    PrintUlongList("  Receive PGNs  :", pDevice->GetReceivePGNs(), stream);
    stream.printf("\n");
}



void SendN2kEnvironment() {
    static unsigned long VerySlowDataUpdated = InitNextUpdate(VerySlowDataUpdatePeriod, MiscSendOffset);
    tN2kMsg N2kMsg;
    double waterTemp = 0.0, outsideTemp = 0.0, pressure = 0.0;

    if (IsTimeToUpdate(VerySlowDataUpdated)) {
        SetNextUpdate(VerySlowDataUpdated, VerySlowDataUpdatePeriod);

        // read the sensors. The temperature should be done first
        // See the bmp180 docs and examples
        outsideTemp = bmp180Temperature();
        pressure = bmp180Pressure();

        oledPrintf(0, 4 * lineh, "Temp %.1f Pres %.0f", outsideTemp, pressure / 100.0);

        SetN2kOutsideEnvironmentalParameters(N2kMsg, 0,
                                             waterTemp,
                                             outsideTemp,
                                             pressure);

        mapSensors["Outside temp"] = String(outsideTemp);
        mapSensors["Pressure"] = String(pressure / 100);

        GwSendYD(N2kMsg);
    }
}

#define START_DELAY_IN_S 8
//*****************************************************************************
void ListDevices(Stream &stream, bool force = false) {
    static bool StartDelayDone = false;
    static int StartDelayCount = 0;
    static unsigned long NextStartDelay = 0;

    if (!StartDelayDone) {  // We let system first collect data to avoid printing all changes
        if (millis() > NextStartDelay) {
            if (StartDelayCount == 0) {
                stream.print("Reading device information from bus ");
                NextStartDelay = millis();
            }
            stream.print(".");
            NextStartDelay += 1000;
            StartDelayCount++;
            if (StartDelayCount > START_DELAY_IN_S) {
                StartDelayDone = true;
                stream.println();
            }
        }
        return;
    }
    if (!force && !pN2kDeviceList->ReadResetIsListUpdated()) return;

    stream.println();
    stream.println("**********************************************************************");
    for (uint8_t i = 0; i < N2kMaxBusDevices; i++) {
        PrintDevice(pN2kDeviceList->FindDeviceBySource(i), stream);
    }
}

void handleN2k() {
    SendN2kEnvironment();

    NMEA2000.ParseMessages();

    int SourceAddress = NMEA2000.GetN2kSource();
    if (SourceAddress != nodeAddress) {  // Save potentially changed Source Address to NVS memory
        nodeAddress = SourceAddress;     // Set new Node Address (to save only once)
        GwSetVal(LASTNODEADDRESS, String(SourceAddress));
        Console->printf("Address Change: New Address=%d\n", SourceAddress);
    }
}

