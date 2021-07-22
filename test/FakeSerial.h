#pragma once
#include <iostream>
#include <string>

using namespace std;

class FakeSerial
{
private:
  long baud;
public:
  FakeSerial();
  void begin(long b);
  void println(char *out);
  void println(string out);
};