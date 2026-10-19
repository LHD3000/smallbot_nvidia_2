#include <array>
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace ctrl_alg
{
/*
 * @brief Linear Quadratic Regulator
 * @details X[n+1]=AX[n]+Bu[n]
 */
template <class T, size_t state_size, size_t control_size> class LQR
{
  public:
    using MatAType = Eigen::Matrix<T, state_size, state_size>;
    using MatBType = Eigen::Matrix<T, state_size, control_size>;
    using MatQType = Eigen::Matrix<T, state_size, state_size>;
    using MatRType = Eigen::Matrix<T, control_size, control_size>;
    using MatKType = Eigen::Matrix<T, control_size, state_size>;
    using StateType = std::array<T, state_size>;
    using ControlType = std::array<T, control_size>;
    struct LQRMatrix
    {
        MatAType A;
        MatBType B;
        MatQType Q;
        MatRType R;
    };
    constexpr static double Tolerance = 1e-6;

  private:
    MatKType K;

  public:
    const MatKType &CanculateGain(const LQRMatrix &mat)
    {
        Eigen::Matrix<T, state_size, state_size> P = mat.Q;
        for (int i = 0; i < 100; i++)
        {
            K = (mat.B.transpose() * P * mat.B + mat.R).inverse() * mat.B.transpose() * P * mat.A;
            // std::cout << K << std::endl;
            Eigen::Matrix<T, state_size, state_size> P_next =
                (mat.A - mat.B * K).transpose() * P * (mat.A - mat.B * K) + K.transpose() * mat.R * K + mat.Q;
            if ((P - P_next).norm() < Tolerance)
            {
                std::cout << "LQR converged" << std::endl;
                break;
            }
            P = P_next;
        }
        return K;
    }
    ControlType GetControl(StateType error)
    {
        Eigen::Matrix<T, control_size, 1> ctrl = -this->K * Eigen::Map<Eigen::Matrix<T, state_size, 1>>(error.data());
        ControlType control;
        Eigen::Map<Eigen::Matrix<T, control_size, 1>>(control.data()) = ctrl;
        return control;
    }
};
} // namespace ctrl_alg