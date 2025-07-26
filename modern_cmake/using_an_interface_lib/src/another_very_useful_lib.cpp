#include <cmath>

#include "using_an_interface_lib/another_very_useful_lib.hpp"

namespace example {

std::vector<example::ASimpleDataStructure> populateData() {
  ASimpleDataStructure new_data;
  new_data.id = "wololo";
  new_data.some_value = 0.0;
  return std::vector<example::ASimpleDataStructure>{new_data};
}
}  // namespace example
