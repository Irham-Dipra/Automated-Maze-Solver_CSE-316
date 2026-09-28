#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <cmath>
#include <deque>
typedef bool boolean;
#define INPUT_PULLUP 2
#define OUTPUT 1
#define LOW 0
#define HIGH 1
#define SERIAL_8N1 0x800001c
extern unsigned long g_fake_millis;
class String {
public: std::string s;
  String(){} String(const char*c):s(c){} String(int v){char b[32];sprintf(b,"%d",v);s=b;}
  String(unsigned int v){char b[32];sprintf(b,"%u",v);s=b;}
  String(long v){char b[32];sprintf(b,"%ld",v);s=b;}
  String(double v,int=2){char b[32];sprintf(b,"%f",v);s=b;}
  void trim(){} void toUpperCase(){}
  int indexOf(char c)const{size_t p=s.find(c);return p==std::string::npos?-1:(int)p;}
  String substring(int a)const{return String(s.substr(a).c_str());}
  String substring(int a,int b)const{return String(s.substr(a,b-a).c_str());}
  float toFloat()const{return atof(s.c_str());}
  unsigned length()const{return s.size();}
  const char*c_str()const{return s.c_str();}
  String& operator+=(char c){s+=c;return *this;}
  String& operator=(const char*c){s=c;return *this;}
  bool operator==(const char*c)const{return s==c;}
};
class SerialC {
public: void begin(unsigned long){} void begin(unsigned long,int,int,int){}
  void println(const char*){} void println(const String&){} void println(){}
  void print(const char*){} void printf(const char*,...){}
  int available(){return 0;} int read(){return -1;} void write(uint8_t){}
};
extern SerialC Serial, Serial2;
unsigned long millis(); void delay(unsigned long); void delayMicroseconds(unsigned);
void pinMode(uint8_t,uint8_t); void digitalWrite(uint8_t,uint8_t); int digitalRead(uint8_t);
class TwoWire { public:
  void begin(int,int){} void begin(){} void end(){} void setClock(uint32_t){}
  void beginTransmission(uint8_t){} void write(uint8_t){}
  uint8_t endTransmission(){return 0;} uint8_t endTransmission(bool){return 0;}
  uint8_t requestFrom(uint8_t,uint8_t){return 6;} int read(){return 0;}
};
extern TwoWire Wire;
