//
// Created by Dafan Wang on 7/30/24.
// Modified by Sam Kwok on 21 Nov 2024.
//

#ifndef CONFIG_H_
#define CONFIG_H_

#include <string>
#include <cstdint>

struct Config {
  uint16_t vendor_id;
  uint16_t product_id;
  std::string serial;

  uint8_t endpoint_in;
  uint8_t endpoint_out;
  uint32_t timeout;

  uint8_t dots_per_mm;
  uint8_t direction;
  uint32_t total;
  uint32_t interval;

  bool offset_x;
  bool offset_y;
};

#endif //CONFIG_H_
