#pragma once
#include <vector>
#include <algorithm>
#include <iterator>
#include <map>
#include <cmath>
#include <unordered_map>

class DataCompression
{
public:
    using TOD_VEC = std::vector<std::vector<double>>;
    static TOD_VEC compress(const TOD_VEC& data, size_t maxSamples);
private:
    static TOD_VEC shannonEntropy(const TOD_VEC& data, int subsetSize);

    static std::map<std::vector<double>, int> getFrequencyMap(const TOD_VEC &data, int subsetSize);

    static double calculateEntropy(const TOD_VEC &data, std::map<std::vector<double>, int> &counts);
};
