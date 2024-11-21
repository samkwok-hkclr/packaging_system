//
// Created by Dafan Wang on 7/26/24.
// Modified by Sam Kwok on 21 Nov 2024.
//

#include "printer/config.h"
#include "printer/printer.h"
#include "printer/libusbxx.hpp"

Printer::Printer(uint16_t vendor_id, uint16_t product_id, std::string_view serial_num)
  : usb_(std::make_unique<libusbxx>()) 
{
  usb_->openDevice(vendor_id, product_id, serial_num);
}

Printer::Printer(uint16_t vendor_id, uint16_t product_id)
  : usb_(std::make_unique<libusbxx>()) 
{
  usb_->openDevice(vendor_id, product_id);
}

Printer::~Printer() = default;

void Printer::configure(uint8_t endpoint_in, uint8_t endpoint_out, uint32_t timeout) 
{
  endpoint_in_ = endpoint_in;
  endpoint_out_ = endpoint_out;
  timeout_ = timeout;
}

void Printer::addDefaultConfig(const std::string &name, const std::string &config) 
{
  default_cmd_.try_emplace(name, config);
}

void Printer::addDefaultConfig(const std::string &cmd) 
{
    default_cmd_.try_emplace(cmd, "");
}

bool Printer::updateDefaultConfig(const std::string &name, const std::string &config) 
{
  // if (default_cmd_.contains(name)) {
  if (default_cmd_.find(name) != default_cmd_.end()) {
    default_cmd_[name] = config;
    return true;
  }

  return false;
}

void Printer::runTask(const std::vector<std::string> &cmds) 
{
  for (auto &[name, config]: default_cmd_) {
    // usb_->bulkTransfer(endpoint_out_, std::format("{} {}\r\n", name, config), timeout_);
    usb_->bulkTransfer(endpoint_out_, name + " " + config + "\r\n", timeout_);
  }

  for (auto command: cmds) {
    // usb_->bulkTransfer(endpoint_out_, std::format("{}\r\n", command), timeout_);
    usb_->bulkTransfer(endpoint_out_, command + "\r\n", timeout_);
  }
}
