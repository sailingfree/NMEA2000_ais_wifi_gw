// nmea2000 handlers header

#pragma once

typedef struct {
    unsigned long PGN;
    void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

void Heading(const tN2kMsg &N2kMsg);
void COGSOG(const tN2kMsg &N2kMsg);
void GNSS(const tN2kMsg &N2kMsg);
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

// Where the output goes.
extern Stream *OutputStream;