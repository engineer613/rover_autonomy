#include "../include/um982_driver/core/UnicoreParsers.hpp"
#include "../include/um982_driver/core/UnicoreDriver.hpp"

UNIHEADINGParser::UNIHEADINGParser(UnicoreDriver* driver):driver_(driver) {}

void UNIHEADINGParser::operator()(const uint8_t* data) {
  if(parseUNIHEADINGB(data, driver_->uniheading_short_output_))
    driver_->publishUNIHEADING();
}

bool UNIHEADINGParser::parseUNIHEADINGB(const uint8_t* log_buffer,
                                        UnicoreMsgs::UNIHEADINGShort& dual_heading) {
  const size_t H = UNICORE_HEADER_LEN;

  dual_heading.sol_status = readValue<uint32_t>(log_buffer, H);
  dual_heading.pos_type = readValue<uint32_t>(log_buffer, H + 4);
  dual_heading.baseline_len = readValue<float>(log_buffer, H + 8);
  dual_heading.heading = readValue<float>(log_buffer, H + 12);
  dual_heading.pitch = readValue<float>(log_buffer, H + 16);
  dual_heading.heading_sigma = readValue<float>(log_buffer, H + 20);
  dual_heading.pitch_sigma = readValue<float>(log_buffer, H + 24);

  return true;
}


AGRICParser::AGRICParser(UnicoreDriver* driver):driver_(driver) {}

void AGRICParser::operator()(const uint8_t* data) {
  if(parseAGRICB(data, driver_->agric_short_output_))
    driver_->publishAGRIC();
}

bool AGRICParser::parseAGRICB(const uint8_t* log_buffer,
                              UnicoreMsgs::AGRICShort& agric_short) {

  const size_t H = UNICORE_HEADER_LEN;

  agric_short.lat = readValue<double>(log_buffer, H + 80);
  agric_short.lon = readValue<double>(log_buffer, H + 88);
  agric_short.alt = readValue<double>(log_buffer, H + 96);

  agric_short.lat_sigma = readValue<double>(log_buffer, H + 128);
  agric_short.lon_sigma = readValue<double>(log_buffer, H + 132);
  agric_short.alt_sigma = readValue<double>(log_buffer, H + 136);

  agric_short.speed = readValue<float>(log_buffer, H + 52);
  agric_short.vel_n = readValue<float>(log_buffer, H + 56);
  agric_short.vel_e = readValue<float>(log_buffer, H + 60);
  agric_short.vel_u = readValue<float>(log_buffer, H + 64);

  agric_short.vel_n_sigma = readValue<float>(log_buffer, H + 68);
  agric_short.vel_e_sigma = readValue<float>(log_buffer, H + 72);
  agric_short.vel_u_sigma = readValue<float>(log_buffer, H + 76);

  agric_short.heading = readValue<float>(log_buffer, H + 40);
  agric_short.pitch = readValue<float>(log_buffer, H + 44);
  agric_short.roll = readValue<float>(log_buffer, H + 48);

  agric_short.speed_heading = readValue<float>(log_buffer, 208);

  return true;
}


PVTSLNParser::PVTSLNParser(UnicoreDriver* driver): driver_(driver) {}

void PVTSLNParser::operator()(const uint8_t* data) {
  if(parsePVTSLNB(data, driver_->pvtsln_short_output_))
    driver_->publishPVTSLN();
}

bool PVTSLNParser::parsePVTSLNB(const uint8_t* log_buffer,
                  UnicoreMsgs::PVTSLNShort& pvtsln_short) {

  const size_t H = UNICORE_HEADER_LEN;

  pvtsln_short.bestpos_type = readValue<uint32_t>(log_buffer, H + 24);
  pvtsln_short.bestpos_lat = readValue<double>(log_buffer, H + 32);
  pvtsln_short.bestpos_lon = readValue<double>(log_buffer, H + 40);
  pvtsln_short.bestpos_hgt = readValue<float>(log_buffer, H+28);

  pvtsln_short.psrvel_north = readValue<double>(log_buffer, H + 96);
  pvtsln_short.psrvel_east = readValue<double>(log_buffer, H + 104);
  pvtsln_short.psrvel_ground = readValue<double>(log_buffer, H + 112);

  pvtsln_short.heading_type = readValue<uint32_t>(log_buffer, H + 120);
  pvtsln_short.baseline = readValue<float>(log_buffer, H + 124);
  pvtsln_short.heading_deg = readValue<float>(log_buffer, H + 128);
  pvtsln_short.pitch_deg = readValue<float>(log_buffer, H + 132);

  return true;
}
