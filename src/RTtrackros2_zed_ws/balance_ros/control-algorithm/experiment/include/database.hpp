#pragma once
#include <string>
#include <tuple>
#include <vector>

class DataBase
{
  public:
    using StateVector = std::vector<double>;
    using ControlVector = std::vector<double>;
    using DataPoint = std::tuple<double, StateVector, ControlVector>;
    using DataInfo = std::tuple<double, int>;

  private:
    std::vector<DataPoint> data;
    int currentDataIndex = 0;
    double endTime = 0;

  public:
    DataPoint GetNextData();
    DataPoint GetLastData();
    DataPoint SamplerData(double t);
    void SaveData(DataPoint data);
    DataInfo GetDataInfo();
    void ResetIndex();
    void Clear();
    void SaveFile(const std::string &path);
    void LoadFile(const std::string &path);
};
