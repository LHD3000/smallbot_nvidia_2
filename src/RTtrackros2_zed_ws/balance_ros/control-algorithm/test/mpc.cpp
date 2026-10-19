#include "ctrl_alg/mpc/mpc.hpp"
#include "gtest/gtest.h"

TEST(mpc_test, valur_test)
{
    using MPC = ctrl_alg::MPC<double, 2, 1, 5>;
    auto A = (MPC::MatAType() << 0, 1, -24.5, 0).finished();
    auto B = (MPC::MatBType() << 0, 1).finished();
    MPC::MPCMatrix mat{.A = (MPC::MatAType() << 0, 1, -24.5, 0).finished(),
                       .B = (MPC::MatBType() << 0, 1).finished(),
                       .Q = (MPC::MatQType() << 100, 0, 0, 1).finished(),
                       .R = (MPC::MatRType() << 1.0).finished()};
    MPC mpc{mat, {-1}, {1}};
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}