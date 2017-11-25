#include "gtest/gtest.h"
#include "../src/discrete_trajectory_planner.h"

namespace {

    /*
       Empty Road
       ------------------------------------

       O- -  -  -  -  -  -

       ------------------------------------
     */
    TEST(DiscreteTrajectoryTest, EmptyRoad) {
        int ego_s=0, ego_d=2, ego_v =0;
        int simulate_steps=5, horizon_steps=5;
        int max_v=3, max_a=1, num_lanes=3;
        int crash_distance=1, preferred_distance=3;
        vector<int>
                other_s = {},
                other_d = {},
                other_v = {};
        DiscreteTrajectoryPlanner planner(ego_s, ego_d, ego_v,
                                          other_s, other_d, other_v,
                                          simulate_steps, horizon_steps,
                                          max_v, max_a, num_lanes,
                                          crash_distance, preferred_distance);
        planner.calculateSeconds(1.0);
        EXPECT_TRUE(planner.finished());
        vector<int> expected_path_s = {0,0,1,3,6,9};
        vector<int> expected_path_d = {2,2,2,2,2,2};
        vector<int> expected_path_v = {0,1,2,3,3,3};
        EXPECT_EQ(planner.pathS(),expected_path_s);
        EXPECT_EQ(planner.pathD(),expected_path_d);
        EXPECT_EQ(planner.pathV(),expected_path_v);
    }

    /*
       Clear Path
       ------------------------------------
                X
       O- -  -  -  -  -  -
                X
       ------------------------------------
     */
    TEST(DiscreteTrajectoryTest, ClearPath) {
        int ego_s=0, ego_d=2, ego_v =0;
        int simulate_steps=5, horizon_steps=5;
        int max_v=3, max_a=1, num_lanes=3;
        int crash_distance=1, preferred_distance=3;
        vector<int>
                other_s = {5,5},
                other_d = {0,4},
                other_v = {0,0};
        DiscreteTrajectoryPlanner planner(ego_s, ego_d, ego_v,
                                          other_s, other_d, other_v,
                                          simulate_steps, horizon_steps,
                                          max_v, max_a, num_lanes,
                                          crash_distance, preferred_distance);
        planner.calculateSeconds(1.0);
        EXPECT_TRUE(planner.finished());
        vector<int> expected_path_s = {0,0,1,3,6,9};
        vector<int> expected_path_d = {2,2,2,2,2,2};
        vector<int> expected_path_v = {0,1,2,3,3,3};
        EXPECT_EQ(planner.pathS(),expected_path_s);
        EXPECT_EQ(planner.pathD(),expected_path_d);
        EXPECT_EQ(planner.pathV(),expected_path_v);
    }
}
