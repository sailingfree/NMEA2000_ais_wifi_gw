//

#include <StringStream.h>

StringStream::StringStream() {
}

void StringStream::clear() {
    data.clear();
}

int StringStream::read() {
    return 0;
}

size_t StringStream::write(uint8_t v) {
    data += (char)v;
    return 1;
}

int StringStream::available() {
    return 0;
}

int StringStream::availableForWrite() {
    return 1;
}

int StringStream::peek() {
    return 0;
}

void StringStream::flush() {
}

/*
size_t StringStream::printf(const char * fmt, ...) {

    char buffer[128];
    memset(buffer, 0, sizeof(buffer));
    va_list myargs;
    va_start(myargs, fmt);
    vsprintf(buffer, fmt, myargs);
    va_end(myargs);
    data += buffer;
    return strlen(buffer);
}

size_t StringStream::print(const char *str) {
    data += str;
    return strlen(str);
}

size_t StringStream::println(const char * str) {
    data += str;
    data += "\n";
    return strlen(str) + 1;
}

size_t StringStream::println() {
    data += "\n";
    return 1;
}
*/