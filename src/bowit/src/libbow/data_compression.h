#pragma once 

#include <algorithm>
#include <iterator>
#include <map>
#include <cmath>

class DataCompression
{
    using TOD_VEC = std::vector<std::vector<double>>;
public:
    static TOD_VEC compress(const TOD_VEC& data, size_t maxSamples);
private:

};


template <typename T>
inline std::vector<T> shannonEntropy(const std::vector<T>& data, int subsetSize) {
    // Count occurrences of each element
    std::map<T, int> counts;
    for (const auto& element : data) {
        counts[element]++;
    }

    // Calculate total entropy
    double totalEntropy = 0.0;
    for (const auto& [value, count] : counts) {
        // Avoid log(0) with small probabilities
        double probability = static_cast<double>(count) / data.size();
        if (probability > 0) {
            totalEntropy -= probability * log2(probability);
        }
    }

    // Initialize best entropy and subset
    double bestEntropy = -std::numeric_limits<double>::max();
    std::vector<T> bestSubset;

    // Try all possible subsets of the desired size
    for (int start = 0; start <= data.size() - subsetSize; ++start) {
        // Calculate entropy of the current subset
        double currentEntropy = totalEntropy;
        std::map<T, int> currentCounts(counts);
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
            bestSubset = std::vector<T>(data.begin() + start, data.begin() + start + subsetSize);
        }
    }

    return bestSubset;
}


DataCompression::TOD_VEC DataCompression::compress(const TOD_VEC& data, size_t maxSamples)
{
    // convert to x , y,  z vec
    if(data.size() < maxSamples)
        return data;
    return shannonEntropy(data, maxSamples);
}
