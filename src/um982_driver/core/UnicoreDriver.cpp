#include "um982_driver/core/UnicoreDriver.hpp"

UnicoreDriver::UnicoreDriver(
  const std::string& device_path,
  int baud_rate,
  int timeout_ms,
  uint8_t data_bits,
  uint8_t stop_bits,
  Parity parity)
{

  log_router_ = {
          {972, UNIHEADINGParser(this)},
          {1021, PVTSLNParser(this)},
          {11276, AGRICParser(this)}
  };

  this->serial_ = std::make_unique<SerialInterface>(device_path,
        baud_rate,
        timeout_ms,
        data_bits,
        stop_bits,
        parity);

  if (this->serial_)
    registerSerialQueue(serial_->getQueuePtr());

  if(serial_->isOpen()) {
    serial_->startReading();
  }

  driver_thread_ = std::thread(&UnicoreDriver::driverThreadFunction, this);
}

UnicoreDriver::~UnicoreDriver() {
  if (driver_thread_.joinable())
    driver_thread_.join();
}

void UnicoreDriver::registerSerialQueue(std::shared_ptr<SerialQueue> queue_ptr){
  this->serial_queue_ = queue_ptr;
}

void UnicoreDriver::frameNextLog() {
  sync_it_ = std::search(buffer_.begin(), buffer_.end(), UnicoreMsgs::SYNC_BYTES.begin(), UnicoreMsgs::SYNC_BYTES.end());

  if (sync_it_ != buffer_.end()) {
    size_t offset = std::distance(buffer_.begin(), sync_it_);
  }

  // Make sure there are atleast 8 bytes including the sync bytes
  // so message ID and payload length are parsed.
  if (std::distance(sync_it_, buffer_.end()) >= 8) {
    // both little endian
    uint16_t msg_id = sync_it_[4] | (sync_it_[5] << 8);
    uint16_t payload_len = sync_it_[6] | (sync_it_[7] << 8);
    size_t log_len = UnicoreMsgs::UNICORE_HEADER_LEN + payload_len;

    const uint8_t* log_data_ptr = &(*sync_it_);

    if (std::distance(sync_it_, buffer_.end()) >= log_len) {
      log_router_.at(msg_id)(log_data_ptr);
    } else {
      std::this_thread::sleep_for(0.05s);
    }
  }
}


bool UnicoreDriver::getSerialData() {
  SerialBuffer* rx_buffer;
  serial_queue_->popQueue(rx_buffer);
  buffer_.insert(buffer_.end(), rx_buffer->begin(), rx_buffer->end());
  serial_queue_->releaseBuffer(rx_buffer);

  return true;
}

void UnicoreDriver::publishAGRIC() {
  agric_ros_pub_->notify(agric_short_output_);
}

void UnicoreDriver::publishUNIHEADING() {
  uniheading_ros_pub_->notify(uniheading_short_output_);
}

void UnicoreDriver::publishPVTSLN() {
  pvtsln_ros_pub_->notify(pvtsln_short_output_);
}


void UnicoreDriver::driverThreadFunction() {
  while(serial_queue_->isQueueing()) {
    bool ok = getSerialData();
    frameNextLog();
  }
}

