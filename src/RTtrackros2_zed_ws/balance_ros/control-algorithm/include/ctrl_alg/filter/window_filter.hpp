#pragma once
#include <deque>

namespace CtrlAlg
{
template <class T> class WindowFilter
{
  public:
    WindowFilter(int count = 1)
    {
        size = count;
        while (que.size() < size)
        {
            que.push_back(0);
        }
    }
    T GetControl(T value)
    {
        que.push_back(value);
        que.pop_front();

        float sum = 0;
        for (int i = 0; i < size; i++)
        {
            sum += que[i];
        }
        return sum / size;
    }

  private:
    int size;
    std::deque<T> que;
};
} // namespace CtrlAlg
