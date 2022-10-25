// Sender for N2k messages

#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>
#include <N2kMsg.h>

// Send the n2k message to the yacht devices interface over UDP
void GwSendYD(const tN2kMsg &);

extern uint16_t DaysSince1970;
extern double SecondsSinceMidnight;