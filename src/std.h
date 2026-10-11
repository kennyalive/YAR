#pragma once

#ifdef _MSC_VER
#define _ITERATOR_DEBUG_LEVEL 0
#endif

#include <atomic>
#include <charconv>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>

#ifdef _WIN32
#include <intrin.h>
#endif
