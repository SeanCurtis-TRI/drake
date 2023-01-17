#pragma once

#include <iostream>

#include <gmock/gmock.h>

namespace drake {
namespace math {

/* We have no streaming operator for gtest to use. Furthermore, to help the
 readability of the failure output, it's good to be able to inject a newline
 before the rotation matrix gets printed. So, we overload PrintTo to not
 interfere with any future implementation of operator<< for RotationMatrix. */
template <typename T>
void PrintTo(const RotationMatrix<T>& R, std::ostream* os) {
    (*os) << "\n" << R.matrix();
}

MATCHER_P(RotationEq, R_ref, "") {
    auto delta = arg.matrix() - R_ref.matrix();
    *result_listener << "\nwhere the difference is:\n" << delta;

    return arg.IsExactlyEqualTo(R_ref);
}

MATCHER_P2(RotationNear, R_ref, tolerance, "") {
    // TODO: Communicate the tolerance and observed error.
    return arg.IsNearlyEqualTo(R_ref, tolerance);
}

}
}  // namespace drake
