#pragma once
#include <cstdint>
struct gpio_dev_t { volatile uint32_t in; };
inline gpio_dev_t GPIO = { 0xFFFFFFFFu };   // all bright
