//
// Created by Dafan Wang on 7/25/24.
// Modified by Sam Kwok on 21 Nov 2024.
//

#ifndef LIBUSBXX_HPP_
#define LIBUSBXX_HPP_

#include <string>
#include <iostream>
#include <stdexcept>

#include <libusb-1.0/libusb.h>

class libusbxx {
  struct Deleter;

public:
  explicit libusbxx(int log_level = 0);

  ~libusbxx();

  void openDevice(uint16_t vendor_id, uint16_t product_id);

  void openDevice(uint16_t vendor_id, uint16_t product_id, std::string_view serial);

  template<typename T, typename = typename std::enable_if<
    std::is_convertible<decltype(std::declval<T>().data()), const uint8_t*>::value &&
    std::is_convertible<decltype(std::declval<T>().size()), std::size_t>::value
  >::type>
  int bulkTransfer(uint8_t endpoint, T data, unsigned int timeout);

  int bulkTransfer(uint8_t endpoint, std::string data, unsigned int timeout) const;

  int bulkTransfer(uint8_t endpoint, uint8_t data[], int length, unsigned int timeout) const;

private:
  void init() const;

  libusb_context *ctx_ = nullptr;

  libusb_device_handle *handle_ = nullptr;
};

template<typename T, typename = typename std::enable_if<
  std::is_convertible<decltype(std::declval<T>().data()), const uint8_t*>::value &&
  std::is_convertible<decltype(std::declval<T>().size()), std::size_t>::value
>::type>
int libusbxx::bulkTransfer(uint8_t endpoint, T data, unsigned int timeout) 
{
  return bulkTransfer(endpoint, data.data(), data.size(), timeout);
}

inline libusbxx::libusbxx(int log_level) 
{
  if (int r = libusb_init(&ctx_); r < 0) 
  {
    throw std::runtime_error(std::string("init failed ") + libusb_error_name(r));
  }

  // 设置调试级别
  libusb_set_option(ctx_, LIBUSB_OPTION_LOG_LEVEL, log_level);
}

inline libusbxx::~libusbxx() 
{
  if (handle_ != nullptr) 
  {
    // 释放接口
    libusb_release_interface(handle_, 0);

#ifndef IS_WIN32
  // 重新附加内核驱动程序（如果之前分离了）
  if (libusb_kernel_driver_active(handle_, 0)) 
  {
      libusb_attach_kernel_driver(handle_, 0);
  }
#endif
    libusb_close(handle_);
  }

  if (ctx_ != nullptr) 
  {
    libusb_exit(ctx_);
  }
}

inline void libusbxx::init() const 
{
#ifndef IS_WIN32
  // 检查并分离内核驱动程序（如果需要）
  if (libusb_kernel_driver_active(handle_, 0)) 
  {
    if (auto r = libusb_detach_kernel_driver(handle_, 0); r < 0) 
    {
      // throw std::runtime_error(std::format("kernel driver detach failed: {}", libusb_error_name(r)));
      throw std::runtime_error("kernel driver detach failed: " + std::string(libusb_error_name(r)));
    }
  }
#endif
  // 声明接口 0
  if (auto r = libusb_claim_interface(handle_, 0); r < 0) 
  {
    // throw std::runtime_error(std::format("claim failed: {}", libusb_error_name(r)));
    throw std::runtime_error("claim failed: " + std::string(libusb_error_name(r)));
  }
}

inline int libusbxx::bulkTransfer(uint8_t endpoint, std::string data, unsigned int timeout) const 
{
  return bulkTransfer(endpoint, reinterpret_cast<uint8_t *>(data.data()), data.length(), timeout);
}

inline int libusbxx::bulkTransfer(uint8_t endpoint, uint8_t data[], int length, unsigned int timeout) const 
{
  // 向设备传输数据
  int transferred{};
  auto r = libusb_bulk_transfer(handle_, endpoint, data, length, &transferred, timeout);
  if (r != LIBUSB_SUCCESS || transferred != length) 
  {
      // throw std::runtime_error(std::format("transfer failed: {}", libusb_error_name(r)));
      throw std::runtime_error("transfer failed: " + std::string(libusb_error_name(r)));
  }

  return transferred;
}

inline void libusbxx::openDevice(uint16_t vendor_id, uint16_t product_id) 
{
  handle_ = libusb_open_device_with_vid_pid(ctx_, vendor_id, product_id);
  if (handle_ == nullptr) 
  {
    // throw std::runtime_error(std::format("Device {}:{} open failed", vendor_id, product_id));
    throw std::runtime_error("Device " + std::to_string(vendor_id) + ":" + std::to_string(product_id) + "open failed");
  }

  init();
}

inline void libusbxx::openDevice(uint16_t vendor_id, uint16_t product_id, std::string_view serial) 
{
  libusb_device_handle *handle = nullptr;
  libusb_device **list = nullptr;

  // 获取设备列表
  auto cnt = libusb_get_device_list(ctx_, &list);
  if (cnt < 0) 
  {
    // throw std::runtime_error(std::format("Failed to get device list: {}", libusb_error_name(cnt)));
    throw std::runtime_error("Failed to get device list: " + std::string(libusb_error_name(cnt)));
  }

  for (ssize_t i = 0; i < cnt; i++) 
  {
    libusb_device *device = list[i];
    libusb_device_descriptor desc{};
    auto r = libusb_get_device_descriptor(device, &desc);
    if (r < 0) 
    {
      std::cerr << libusb_error_name(r) << std::endl;
      continue;
    }

    if (desc.idVendor == vendor_id && desc.idProduct == product_id)
    {
      r = libusb_open(device, &handle);
      if (r < 0) 
      {
          // std::cerr << std::format("Device {}:{}, serial: {} open failed: {}", vendor_id, product_id, serial,
          //                          libusb_error_name(r)) << std::endl;
          std::cerr << "Device" + std::to_string(vendor_id) + ":" + std::to_string(product_id) +
                        ", serial: " + std::string(serial) + " open failed: " + std::string(libusb_error_name(r))
                        << std::endl;
          continue;
      }

      unsigned char serial_number[256];
      r = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, serial_number, sizeof(serial_number));
      if (r < 0) 
      {
        libusb_close(handle);
        throw std::runtime_error("Failed to get serial number: " + std::string(libusb_error_name(r)));
      }

      if (serial == reinterpret_cast<char *>(serial_number)) 
      {
        break; // 找到匹配的设备，跳出循环
      }

      libusb_close(handle);
      handle = nullptr;
    }
  }

  // 释放设备列表
  libusb_free_device_list(list, 1);

  if (handle == nullptr) {
      // throw std::runtime_error(std::format("Device {}:{} ({}), not found", vendor_id, product_id, serial));
      throw std::runtime_error("Device " + std::to_string(vendor_id) + ":" + std::to_string(product_id) + " (" +
                                std::string(serial) + "), not found");
  }

  handle_ = handle;
  init();
}

#endif //LIBUSBXX_HPP_
