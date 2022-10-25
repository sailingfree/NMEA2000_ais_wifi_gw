// Preferences and settings for the n2kgw

#include <Arduino.h>
#include <GwPrefs.h>
#include <Preferences.h>

#include <vector>

// Holds the list of configurable registers in the preference object
static std::vector<String> Reg;

static Preferences shellPref;
static const char* prefname = "settings";
static bool doneInit = false;

void GwPrefsInit() {
    if (!doneInit) {
        Reg.clear();
        Reg.push_back(WIFIMODE);
        Reg.push_back(SSID1);
        Reg.push_back(SSID2);
        Reg.push_back(SSPW1);
        Reg.push_back(SSPW2);
        Reg.push_back(GWHOST);
        Reg.push_back(GWPASS);
        Reg.push_back(ENGINEDIA);
        Reg.push_back(ALTDIA);
        Reg.push_back(ALTPOLES);
        Reg.push_back(PRESSCAL);
        Reg.push_back(COMPASSOFF);
        Reg.push_back(LASTNODEADDRESS);
        Reg.push_back(USEGPS);
        Reg.push_back(SENDHEADING);
        doneInit = true;
    }
}

void GwPrint(Stream & s) {
 //   GwPrefsInit();
    s.printf("Preferences\n");
    for(String str: Reg) {
        String val = GwGetVal(str.c_str());
        s.printf("%s : %s\n", str.c_str(), val.c_str());
    }
}


// Check to see if the register is in the list
bool isGwKey(String key) {
//    GwPrefsInit();
    bool isreg = false;
    for (String str : Reg) {
        if (key == str) {
            isreg = true;
        }
    }
    return isreg;
}

// List the allowable registers
void GwListRegs(Stream& s) {
//    GwPrefsInit();
    for (String str : Reg) {
        s.printf("\t%s\n", str.c_str());
    }
}

String GwGetVal(const char* key, String defval) {
    //GwPrefsInit();
    shellPref.begin(prefname, false);
    String val = shellPref.getString(key);
    shellPref.end();
    //Serial.printf("GWGETVAL %s '%s'\n", key, val.c_str());
    if(val == "") {
        val = defval;
    }
    return val;
}

String GwSetVal(const char* key, String newval) {
    //GwPrefsInit();
    String curval;
    shellPref.begin(prefname, false);
    shellPref.putString(key, newval);
    curval = shellPref.getString(key);
    shellPref.end();
    return curval;
}