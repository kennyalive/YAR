#pragma once

#include "../renderer/vk.h"

struct GLFWwindow;

namespace platform
{
VkSurfaceKHR create_surface(VkInstance instance, GLFWwindow* window);
void sleep(int milliseconds);
}
