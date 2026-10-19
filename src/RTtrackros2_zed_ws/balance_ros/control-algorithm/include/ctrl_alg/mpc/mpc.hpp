#pragma once
#include "ctrl_alg/state_space.hpp"
#include <eigen3/Eigen/Dense>
#include <memory>
#include <osqp/osqp.h>

// https://osqp.org/docs/examples/setup-and-solve.html refer to this link
namespace ctrl_alg
{
/* MPC class
 * @tparam T data type
 * @tparam state_size state size
 * @tparam control_size control size
 * @tparam horizon_size horizon size
 */
template <typename T, size_t state_size, size_t control_size, size_t horizon_size> class MPC
{
    static_assert(state_size > 0, "StateSize must be greater than 0");
    static_assert(control_size > 0, "ControlSize must be greater than 0");
    static_assert(horizon_size > 0, "ObserverSize must be greater than 0");

  public:
    using MatAType = Eigen::Matrix<T, state_size, state_size>;
    using MatBType = Eigen::Matrix<T, state_size, control_size>;
    using MatQType = Eigen::Matrix<T, state_size, state_size>;
    using MatRType = Eigen::Matrix<T, control_size, control_size>;
    using StateType = std::array<T, state_size>;
    using ControlType = std::array<T, control_size>;
    struct MPCMatrix
    {
        MatAType A;
        MatBType B;
        MatQType Q;
        MatRType R;
    };

  private:
    Eigen::Matrix<T, state_size, state_size> E;
    Eigen::Matrix<T, state_size, control_size> H2;
    OSQPFloat min[control_size];
    OSQPFloat max[control_size];
    std::unique_ptr<OSQPSolver> solver;
    std::unique_ptr<OSQPSettings> settings;
    std::unique_ptr<OSQPCscMatrix> P;
    std::unique_ptr<OSQPCscMatrix> A;
    OSQPFloat q[state_size];

  public:
    MPC(const MPCMatrix &mat, ControlType min, ControlType max)
    {
        // X=M*xk+C*U
        // M
        Eigen::Matrix<T, state_size * horizon_size + state_size, state_size> M;
        M.setZero();
        M.block(0, 0, state_size, state_size) = Eigen::MatrixXd::Identity(state_size, state_size);
        for (size_t i = 1; i < horizon_size; i++)
        {
            M.block(state_size * i, 0, state_size, state_size) =
                M.block(state_size * (i - 1), 0, state_size, state_size) * mat.A;
        }

        // C
        Eigen::Matrix<T, state_size * horizon_size + state_size, state_size * horizon_size + state_size> C;
        C.setZero();
        for (size_t i = 0; i < horizon_size; i++)
        {
            C.block(state_size * (i + 1), state_size * i, state_size, state_size) = mat.B;
            for (size_t j = i + 1; j < horizon_size; j++)
            {
                C.block(state_size * (j + 1), state_size * i, state_size, state_size) =
                    mat.A * C.block(state_size * j, state_size * i, state_size, state_size);
            }
        }

        // Qbar=diag(Q,Q,...,Q)[hor+1]
        Eigen::Matrix<T, state_size *(horizon_size + 1), state_size *(horizon_size + 1)> Qbar =
            Eigen::Matrix<T, state_size *(horizon_size + 1), state_size *(horizon_size + 1)>::Zero();
        for (size_t i = 0; i <= horizon_size; i++)
        {
            Qbar.block(i * state_size, i * state_size, state_size, state_size) = mat.Q;
        }
        // Rbar=diag(R,R,...,R)[hor]
        Eigen::Matrix<T, control_size * horizon_size, control_size *horizon_size> Rbar =
            Eigen::Matrix<T, control_size * horizon_size, control_size * horizon_size>::Zero();
        for (size_t i = 0; i < horizon_size; i++)
        {
            Rbar.block(i * control_size, i * control_size, control_size, control_size) = mat.R;
        }

        // E=M'*Q~*C
        E = M.transpose() * Qbar * C;
        // H2=2*C'*Q~*C+R~
        H2 = 2 * C.transpose() * Qbar * C + Rbar;

        // creat osqp solver resource
        settings = std::make_unique<OSQPSettings>();
        if (settings)
        {
            osqp_set_default_settings(settings.get());
            settings->alpha = 1.0; /* Change alpha parameter */
        }
        // P = std::make_unique<OSQPCscMatrix>();
        // P = DenseMatrixToOSQP(H2);
        // A = std::make_unique<OSQPCscMatrix>();
        // A = DenseMatrixToOSQP(Eigen::MatrixXd::Identity(state_size, state_size));
        //  setup constraints
        for (int i = 0; i < state_size; i++)
        {
            this->min[i] = min[i];
            this->max[i] = max[i];
        }
    }

    ~MPC()
    {
        if (solver.get() != nullptr)
        {
            osqp_cleanup(solver.get());
        }
    }
    /*
    ControlType solve(const StateType &current, const StateType &target)
    {
        // q'=2*xk'*E

        OSQPInt exitflag = 0;
        if (solver.get() == nullptr)
        {
            // todo
            exitflag = osqp_setup(solver, P, q, A, min, max, state_size, state_size, settings);
        }
        else
        {
            //exitflag = osqp_update_data_mat(solver, P, OSQP_NULL, 3, A, OSQP_NULL, 4);
            //exitflag = osqp_update_data_vec(solver, q, min, max);
        }
        if (!exitflag)
            exitflag = osqp_solve(solver.get());

        // result = solver->solution->x;
    }

    std::shared_ptr<OSQPCscMatrix> DenseMatrixToOSQP(const Eigen::MatrixXd &A) {}

    std::shared_ptr<OSQPCscMatrix> sparseMatrixToOSQP(const Eigen::SparseMatrixBase<double> &A) {}
    */
};
} // namespace ctrl_alg