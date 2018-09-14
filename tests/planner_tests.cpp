//
// Created by Andrii Garkavyi on 9/10/18.
//
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main()
#include "catch.hpp"
#include "../src/planner.h"

SCENARIO( "Car behavior", "[behavior]" ) {
  GIVEN("car") {
    WHEN("in middle lane") {
      int lane = 1;

      AND_WHEN("going KL") {
        BhState state = BhState::KL;
        vector<BhState> states = getNextStates(state, lane);
        THEN("there are 3 possible states") {
          REQUIRE(states.size() == 3);
        }
      }

      AND_WHEN("going LCR") {
        BhState state = BhState::LCR;
        vector<BhState> states = getNextStates(state, lane);
        THEN("just KL") {
          REQUIRE(states.size() == 1);
          REQUIRE(states[0] == BhState::KL);
        }
      }

      AND_WHEN("going LCL") {
        BhState state = BhState::LCL;
        vector<BhState> states = getNextStates(state, lane);
        THEN("just KL") {
          REQUIRE(states.size() == 1);
          REQUIRE(states[0] == BhState::KL);
        }
      }
    }

    WHEN("in right lane ") {
      int lane = 2;

      AND_WHEN("going KL") {
        BhState state = BhState::KL;
        vector<BhState> states = getNextStates(state, lane);
        THEN("there are 2 possible states") {
          REQUIRE(states.size() == 2);
          REQUIRE(states[0] == BhState::KL);
          REQUIRE(states[1] == BhState::LCL);
        }
      }
    }

    WHEN("in left lane ") {
      int lane = 0;

      AND_WHEN("going KL") {
        BhState state = BhState::KL;
        vector<BhState> states = getNextStates(state, lane);
        THEN("there are 2 possible states") {
          REQUIRE(states.size() == 2);
          REQUIRE(states[0] == BhState::KL);
          REQUIRE(states[1] == BhState::LCR);
        }
      }
    }
  }
}

