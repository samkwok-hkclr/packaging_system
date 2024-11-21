//
// Created by Dafan Wang on 7/26/24.
// Modified by Sam Kwok on 21 Nov 2024.
//

#ifndef PRINTER_H_
#define PRINTER_H_

#include <map>
#include <vector>
#include <memory>
#include <cstdint>
#include <string_view>

class libusbxx;

class Printer {
public:
  explicit Printer(uint16_t, uint16_t, std::string_view);

  explicit Printer(uint16_t, uint16_t);

  ~Printer();

  void configure(uint8_t, uint8_t, uint32_t);

  void addDefaultConfig(const std::string &name, const std::string &config);

  void addDefaultConfig(const std::string &cmd);

  bool updateDefaultConfig(const std::string &name, const std::string &config);

  void runTask(const std::vector<std::string> &);

private:
  std::unique_ptr<libusbxx> usb_;
  std::map<std::string, std::string> default_cmd_;

  uint8_t endpoint_in_{};
  uint8_t endpoint_out_{};
  uint32_t timeout_{};
};

#endif //PRINTER_H_
