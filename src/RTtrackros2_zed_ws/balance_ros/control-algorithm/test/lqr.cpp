#include "ctrl_alg/lqr/lqr.hpp"
#include "gtest/gtest.h"

TEST(lqr_gain_test, valur_test)
{
    using LQR = ctrl_alg::LQR<double, 2, 1>;
    LQR lqr;
    LQR::LQRMatrix mat{.A = (LQR::MatAType() << 0, 1, -24.5, 0).finished(),
                       .B = (LQR::MatBType() << 0, 1).finished(),
                       .Q = (LQR::MatQType() << 100, 0, 0, 1).finished(),
                       .R = (LQR::MatRType() << 1.0).finished()};
    auto gain = lqr.CanculateGain(mat);
    // for continus time
    // auto result = (LQR::MatKType() << 1.96223724, 2.21911570).finished();
    // for discrete time
    auto result = (LQR::MatKType() << -24.4650695, 0.0000000).finished();
    EXPECT_NEAR(gain[0], result[0], LQR::Tolerance);
    EXPECT_NEAR(gain[1], result[1], LQR::Tolerance);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}