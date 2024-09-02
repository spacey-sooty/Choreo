// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

template <typename T>
class TRAJOPT_DLLEXPORT Line {
 public:
  Translation2<T> startPoint;
  Translation2<T> endPoint;
};

}  // namespace trajopt
