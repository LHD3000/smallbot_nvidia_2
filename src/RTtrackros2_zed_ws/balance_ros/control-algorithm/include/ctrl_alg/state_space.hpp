#include <cassert>
#include <eigen3/Eigen/Dense>

namespace ctrl_alg
{
  
/*
 * StateSpace
 * @brief State Space Model
 * @details X[n+1]=AX[n]+Bu[n] for discrete time
 * @details dX=AX+Bu for continus time
 * @param A state transition matrix
 * @param B control matrix
 * @param C output matrix
 * @param D feedforward matrix
 */
template <class T, size_t state_size, size_t control_size, size_t observer_size> class StateSpace
{
  public:
    enum Mode
    {
        Discrete,
        Continus
    };
    enum ConvertFunction
    {
        Eulur,
        // HoldOn,
    };
    using TypeA = Eigen::Matrix<T, state_size, state_size>;
    using TypeB = Eigen::Matrix<T, state_size, control_size>;
    using TypeC = Eigen::Matrix<T, observer_size, state_size>;
    using TypeD = Eigen::Matrix<T, observer_size, control_size>;

  private:
    TypeA A;
    TypeB B;
    TypeC C;
    TypeD D;
    Mode mode;

  public:
    StateSpace(TypeA A, TypeB B, TypeC C, TypeD D, Mode mode) {}
    StateSpace Convert(double samplerTime, Mode mode, ConvertFunction function)
    {
        if (mode == this->mode)
        {
            return *this;
        }
        else
        {
            assert(mode == Discrete);
            if (function == Eulur)
            {
                return ConvertToDiscreteByEulur(*this);
            }
            else
            {
                throw std::runtime_error("Not implemented");
            }
        }
    }

    Mode GetMode() const
    {
        return mode;
    }

  public:
    static StateSpace ConvertToContinusByEulur(const StateSpace &discrete, double samplerTime)
    {
        return StateSpace(discrete.A * samplerTime + Eigen::MatrixXd::Identity(state_size, state_size),
                          discrete.B * samplerTime, discrete.C, discrete.D, Continus);
    }
};
} // namespace ctrl_alg