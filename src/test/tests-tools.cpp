#include "catch.hpp"

#include "../tools.h"

Tools tools;

TEST_CASE( "Numbers are computed", "[NUMBERS]") {
  
  REQUIRE( tools.returnNumberPlus2(1) == 3);
  REQUIRE( tools.returnNumberPlus2(3) > 4);
  REQUIRE( tools.returnNumberPlus2(3) < 6);
}