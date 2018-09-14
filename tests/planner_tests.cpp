//
// Created by Andrii Garkavyi on 9/10/18.
//
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main()
#include "catch.hpp"
#include "../src/planner.h"
#include "../src/json.hpp"



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


TEST_CASE("Average speed", "[behavior]") {


  SECTION("empty road") {
    std::string s = "[]";//"[[1, 0,0, 10, 10, -20, 3],[2, 0.0, 0.0, 14.0, 15.0, 30, 2.5]]";
    vector<vector<double>> j = nlohmann::json::parse(s);

    std::map<int, double> speeds = getLaneAvgSpeed(j, 0);

    REQUIRE(speeds.empty());
  }

  SECTION("one left behind") { //don't count cars behind
    std::string s = "[[1, 0,0, 10, 10, -20, 3]]";
    vector<vector<double>> j = nlohmann::json::parse(s);

    std::map<int, double> speeds = getLaneAvgSpeed(j, 0);

    REQUIRE(speeds.size() == 0);
//    REQUIRE(speeds[0] == Approx(14.0).margin(0.2));
  }

  SECTION("two left") {
    std::string s = "[[1, 0,0, 10, 10, -20, 3],[2, 0,0, 20, 20, 20, 3.5]]";
    vector<vector<double>> j = nlohmann::json::parse(s);

    std::map<int, double> speeds = getLaneAvgSpeed(j, 0);

    REQUIRE(speeds.size() == 1);
    REQUIRE(speeds[0] == Approx(sqrt(20*20*2)).margin(0.1));
  }

  SECTION("3 left") {
    std::string s = "[[1, 0,0, 10, 10, 10, 3],[2, 0,0, 5, 2, 20, 3.5],[3, 0,0, 3, 4, 40, 1.5]]";
    vector<vector<double>> j = nlohmann::json::parse(s);

    std::map<int, double> speeds = getLaneAvgSpeed(j, 0);

    REQUIRE(speeds.size() == 1);
    REQUIRE(speeds[0] == Approx((14.1+5.38+5)/3).margin(0.2));
  }

  SECTION("two cars") {
    std::string s = "[[1, 0,0, 10, 10, 10, 3],[2, 0,0, 3, 4, 20, 9.0]]";
    vector<vector<double>> j = nlohmann::json::parse(s);

    std::map<int, double> speeds = getLaneAvgSpeed(j, 0);

    REQUIRE(speeds.size() == 2);
    REQUIRE(speeds[0] == Approx(14.1).margin(0.1));
    REQUIRE(speeds[2] == Approx(5).margin(0.1));
  }
}

TEST_CASE("Cost based on speed", "[behavior]") {


  SECTION("slow car ahead") {
    std::string s = "[[2, 0,0, 3, 4, 20, 2.0]]";
    vector<vector<double>> j = nlohmann::json::parse(s);

    double cost = costSpeed(0, 0, j);

    REQUIRE(Approx(cost).margin(0.01) == 0.787);
    REQUIRE(Approx(costSpeed(1, 0, j)).margin(0.01) == 0);
    REQUIRE(Approx(costSpeed(2, 0, j)).margin(0.01) == 0);
  }

  SECTION("empty road") {
    std::string s = "[]";//"[[1, 0,0, 10, 10, -20, 3],[2, 0.0, 0.0, 14.0, 15.0, 30, 2.5]]";
    vector<vector<double>> j = nlohmann::json::parse(s);

    double cost = costSpeed(0, 0, j);

    REQUIRE(Approx(cost).margin(0.01) == 0);
  }
}

