#pragma once

#include <Arduino.h>

#define MAXSTRINGLEN 1024
class StringStream : public Stream {
    public:
        StringStream();
        String data;
//        char data[MAXSTRINGLEN];
//        int cur;

      /*
        size_t printf(const char * fmt, ...);
        size_t print(const char *);
        size_t print(unsigned long);
        size_t println(const char *);
        size_t println();
      */
        /** Clear the buffers */
  void clear(); 

  virtual size_t write(uint8_t);
  virtual int availableForWrite(void);
  
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();
};