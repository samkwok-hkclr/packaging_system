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

std::string Printer::convert_utf8_to_gbk(const std::string &utf8_string) 
{
  // 设置转换
  iconv_t cd = iconv_open("GBK", "UTF-8");
  if (cd == (iconv_t)(-1)) {
      perror("iconv_open failed");
      return "";
  }

  // 准备输入和输出
  char *in_buf = const_cast<char*>(utf8_string.c_str());
  size_t in_bytes_left = utf8_string.size();
  
  // 预分配输出缓冲区，GBK 编码可能会稍大
  size_t out_buf_size = in_bytes_left * 2; // 预留空间
  char *out_buf = new char[out_buf_size];
  char *out_ptr = out_buf;
  size_t out_bytes_left = out_buf_size;

  // 执行转换
  size_t result = iconv(cd, &in_buf, &in_bytes_left, &out_ptr, &out_bytes_left);
  
  if (result == (size_t)(-1)) {
    perror("iconv failed");
    delete[] out_buf;
    iconv_close(cd);
    return "";
  }

  // 计算转换后的字符串长度
  size_t converted_length = out_buf_size - out_bytes_left;
  
  // 生成输出字符串
  std::string gbk_string(out_buf, converted_length);

  // 清理
  delete[] out_buf;
  iconv_close(cd);

  return gbk_string;
}