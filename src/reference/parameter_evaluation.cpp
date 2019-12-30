#include "std.h"
#include "lib/common.h"
#include "parameter_evaluation.h"

#include "lib/parameter.h"

ColorRGB evaluate_rgb_parameter(const RGB_Parameter& param) {
    if (param.is_constant)
        return param.constant_value;

    ASSERT(false);
    return ColorRGB{};
}
