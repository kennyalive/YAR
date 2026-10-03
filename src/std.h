#pragma once

#ifdef _MSC_VER
#define _ITERATOR_DEBUG_LEVEL 0
#endif

#include <array>
#include <atomic>
#include <charconv>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#include <intrin.h>
#endif
