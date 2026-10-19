#include "database.hpp"
#include <QMessageBox>
#include <regex>
#include <fstream>
DataBase::DataPoint DataBase::GetNextData()
{
    if (currentDataIndex < data.size())
    {
        return data[currentDataIndex++];
    }
    else
    {
        throw std::out_of_range("No more data available");
    }
}

DataBase::DataPoint DataBase::GetLastData()
{
    if (!data.empty())
    {
        return data.back();
    }
    else
    {
        throw std::out_of_range("No data available");
    }
}

DataBase::DataPoint DataBase::SamplerData(double t)
{
    if (data.empty())
    {
        throw std::out_of_range("No data available");
    }

    if (t <= std::get<0>(data.front()))
    {
        return data.front();
    }

    if (t >= std::get<0>(data.back()))
    {
        return data.back();
    }

    for (size_t i = 1; i < data.size(); ++i)
    {
        if (std::get<0>(data[i]) >= t)
        {
            const auto &[t1, state1, control1] = data[i - 1];
            const auto &[t2, state2, control2] = data[i];

            double ratio = (t - t1) / (t2 - t1);

            StateVector interpolatedState(state1.size());
            ControlVector interpolatedControl(control1.size());

            for (size_t j = 0; j < state1.size(); ++j)
            {
                interpolatedState[j] = state1[j] + ratio * (state2[j] - state1[j]);
            }

            for (size_t j = 0; j < control1.size(); ++j)
            {
                interpolatedControl[j] = control1[j] + ratio * (control2[j] - control1[j]);
            }

            return std::make_tuple(t, interpolatedState, interpolatedControl);
        }
    }

    throw std::out_of_range("No data available for the given time");
}

void DataBase::SaveData(DataPoint dataPoint)
{
    data.push_back(dataPoint);
    endTime = std::get<0>(dataPoint);
}
DataBase::DataInfo DataBase::GetDataInfo()
{
    return std::make_tuple(endTime, data.size());
}
void DataBase::ResetIndex()
{
    currentDataIndex = 0;
}
void DataBase::Clear()
{
    data.clear();
    currentDataIndex = 0;
    endTime = 0;
}

void DataBase::SaveFile(const std::string &path)
{
    std::ofstream outFile(path);
    if (!outFile)
    {
        throw std::ios_base::failure("Failed to open file for writing");
    }
    // add header first
    int stateSize = std::get<1>(data.front()).size();
    int controlSize = std::get<2>(data.front()).size();
    outFile << "# Time[1] State[" << stateSize << "] Control[" << controlSize << "]\n";
    // add data
    for (const auto &point : data)
    {
        outFile << std::get<0>(point) << " ";
        for (const auto &state : std::get<1>(point))
        {
            outFile << state << " ";
        }
        for (const auto &control : std::get<2>(point))
        {
            outFile << control << " ";
        }
        outFile << "\n";
    }
    outFile.close();
}

bool static ParseHeader(const std::string &header, int &timeSize, int &stateSize, int &controlSize)
{
    std::regex pattern(R"(# Time\[(\d+)\] State\[(\d+)\] Control\[(\d+)\])");
    std::smatch matches;
    if (std::regex_match(header, matches, pattern))
    {
        timeSize = std::stoi(matches[1].str());
        stateSize = std::stoi(matches[2].str());
        controlSize = std::stoi(matches[3].str());
        return true;
    }
    else
    {
        return false;
    }
}

void DataBase::LoadFile(const std::string &path)
{
    std::ifstream inFile(path);
    if (!inFile)
    {
        throw std::ios_base::failure("Failed to open file for reading");
    }
    // read header #time[1] state[4] control[0]
    std::string header;
    std::getline(inFile, header);
    int timeSize = 1, stateSize = 0, controlSize = 0;
    if (!ParseHeader(header, timeSize, stateSize, controlSize))
    {
        QMessageBox::critical(nullptr, "Error", "Load File Failed: Possible invalid file header");
        return;
    }

    data.clear();
    double t;
    StateVector stateVector(stateSize);
    ControlVector controlVector(controlSize);
    while (inFile >> t)
    {
        double value;
        for (size_t i = 0; i < stateVector.size(); ++i)
        {
            inFile >> value;
            stateVector[i] = value;
        }
        for (size_t i = 0; i < controlVector.size(); ++i)
        {
            inFile >> value;
            controlVector[i] = value;
        }
        data.emplace_back(t, stateVector, controlVector);
    }
    inFile.close();
    endTime = std::get<0>(data.back());
}