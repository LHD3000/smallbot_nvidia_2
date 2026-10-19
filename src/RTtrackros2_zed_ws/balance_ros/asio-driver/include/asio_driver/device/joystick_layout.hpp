#pragma once

namespace AD
{
template <class T> int inline AsCode(T layout)
{
    return static_cast<int>(layout);
}

enum class XBoxButtonLayout : int
{
    A = 0,
    B,
    X,
    Y,
    LB,
    RB,
    LT,
    RT,
    Back,
    Start,
    LeftJS,
    RightJS,
    Up,
    Down,
    Left,
    Right,
    Icon = 16
};
enum class XBoxAixLayout : int
{
    LeftHor = 0,
    LeftVec,
    RightHor,
    RightVec,
};

enum class BFMButtonLayout : int
{
    A = 0,
    B,
    X = 3,
    Y,
    LB = 6,
    RB,
    LT,
    RT,
    Back = 10,
    Start,
    Icon,
    LeftJS,
    RightJS
};

enum class BFMAixLayout : int
{
    LeftHor = 0,
    LeftVec,
    RightHor,
    RightVec,
    ArrowHor = 6,
    ArrowVec,
};
} // namespace AD