#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include "UnicoreMsgs.hpp"

class UnicoreDriver;

using UnicoreMsgs::UNICORE_HEADER_LEN;

// Helper template to read various datatypes from binary LOGs
template<typename T>
T readValue(const uint8_t* log_buffer, size_t offset) {
  T val;
  std::memcpy(&val, &log_buffer[offset], sizeof(T));
  return val;
}

class UNIHEADINGParser {
public:
  explicit UNIHEADINGParser(UnicoreDriver* driver);

  void operator()(const uint8_t* data);
private:
    bool parseUNIHEADINGB(const uint8_t* log_buffer,
                        UnicoreMsgs::UNIHEADINGShort& dual_heading);
    UnicoreDriver* driver_;
};


class AGRICParser {
public:
  explicit AGRICParser(UnicoreDriver* driver);

  void operator()(const uint8_t* data);
private:
  bool parseAGRICB(const uint8_t* log_buffer,
                        UnicoreMsgs::AGRICShort& agric_short);
  UnicoreDriver* driver_;
};


class PVTSLNParser {
public:
  explicit PVTSLNParser(UnicoreDriver* driver);

  void operator()(const uint8_t* data);

private:
  bool parsePVTSLNB(const uint8_t* log_buffer,
                    UnicoreMsgs::PVTSLNShort& pvtsln_short);
  UnicoreDriver* driver_;
};

