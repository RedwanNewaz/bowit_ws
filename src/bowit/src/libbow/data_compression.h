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

DataCompression::TOD_VEC DataCompression::shannonEntropy(const TOD_VEC& data, int subsetSize) {

    std::map<std::vector<double>, int> counts = getFrequencyMap(data, subsetSize);

    // Calculate total entropy
    double totalEntropy = calculateEntropy(data, counts);

    // Initialize best entropy and subset
    double bestEntropy = -std::numeric_limits<double>::max();
    TOD_VEC bestSubset;

    // Try all possible subsets of the desired size
    for (int start = 0; start <= data.size() - subsetSize; ++start) {
        // Calculate entropy of the current subset
        double currentEntropy = totalEntropy;
        std::map<std::vector<double>, int> currentCounts(counts);
        for (int i = 0; i < subsetSize; ++i) {
            const auto& element = data[start + i];
            currentCounts[element]--;
            if (currentCounts[element] > 0) {
                currentEntropy += currentCounts[element] * log2(currentCounts[element]) / data.size();
            }
        }

        // Update best entropy and subset if necessary
        if (currentEntropy > bestEntropy) {
            bestEntropy = currentEntropy;
            bestSubset = TOD_VEC(data.begin() + start, data.begin() + start + subsetSize);
        }
    }
    return bestSubset;
}

double
DataCompression::calculateEntropy(const DataCompression::TOD_VEC &data, std::map<std::vector<double>, int> &counts) {
    double totalEntropy = 0.0;
    for (const auto& [value, count] : counts) {
        // Avoid log(0) with small probabilities
        double probability = static_cast<double>(count) / data.size();
        if (probability > 0) {
            totalEntropy -= probability * log2(probability);
        }
    }
    return totalEntropy;
}

std::map<std::vector<double>, int> DataCompression::getFrequencyMap(const DataCompression::TOD_VEC &data, int subsetSize) {
    // compute entropy from the third column of data matrix which contains scalar observation
    std::vector<double> observations(data.size());
    std::transform(data.begin(), data.end(), observations.begin(),
                   [&](const std::vector<double>&val){return val[2];});

    // compute resolution to convert real value observation to integer key index
    double maxVal = *std::max_element(observations.begin(), observations.end());
    double minVal = *std::min_element(observations.begin(), observations.end());
    double resolution = (maxVal - minVal) / subsetSize;

    std::unordered_map<int, int> observationCounter;
    for(const auto& item:data)
    {
        int key = int(item[2]/resolution);
        bool seen = (observationCounter.find(key) != observationCounter.end());
        observationCounter[key] = seen ? observationCounter[key] + 1 : 1;
    }

    // Count occurrences of each element
    std::map<std::vector<double>, int> counts;
    for (const auto& element : data) {
        int key = int(element[2]/resolution);
        counts[element] = observationCounter[key];
    }
    return counts;
}


DataCompression::TOD_VEC DataCompression::compress(const TOD_VEC& data, size_t maxSamples)
{
    // convert to x , y,  z vec
    if(data.size() < maxSamples)
        return data;
    return shannonEntropy(data, maxSamples);
}
