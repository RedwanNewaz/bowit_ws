#pragma once 
#include <iostream>
#include <Eigen/Dense>
#include <unordered_set>
// #define DEBUG(x) std::cout << x << std::endl  

class DataCompression
{
    using TOD_VEC = std::vector<std::vector<double>>; 
public:
    static TOD_VEC compress(const TOD_VEC& data, size_t maxSamples);
private:
    static std::unordered_set<int> maximize_entropy_subset(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z, int n);
    static double calculate_entropy(const Eigen::VectorXd& probabilities);
    static double conditional_entropy(const Eigen::VectorXd& y, const Eigen::VectorXd& x);
    static double computeMinDist(const Eigen::VectorXd& x, const Eigen::VectorXd& y, int currentIndex, std::unordered_set<int>& selectedIndex);  

}; 


DataCompression::TOD_VEC DataCompression::compress(const TOD_VEC& data, size_t maxSamples)
{
    // convert to x , y,  z vec 
    if(data.size() < maxSamples)
        return data; 

    size_t N = data.size();
    
    // std::cout << "data compression " << N << std::endl;    
    
    int n = std::min(N, maxSamples);
    Eigen::VectorXd x(N), y(N), z(N); 
    int i = 0;
    for(const auto& val: data)
    {
        x(i) = val[0]; 
        y(i) = val[1]; 
        z(i) = val[2];
        ++i;
    }

    TOD_VEC result; 
    for(auto& index: maximize_entropy_subset(x, y, z, n))
    {
        result.emplace_back(data[index]);
    }

    return result;  
}



std::unordered_set<int> DataCompression::maximize_entropy_subset(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z, int n)
{
    // Ensure n is within the range of the data
    n = std::min(n, static_cast<int>(y.size()));

    // Start with an empty subset
    Eigen::VectorXd subset;
    std::unordered_set<int> selectedIndex;
    Eigen::VectorXd distance;

    auto info_gain_lambda = [=](const Eigen::VectorXd& candidate_subset) {
        return conditional_entropy(z, candidate_subset);
    };

    for (int _ = 0; _ < n; ++_) {
        
        double max_information_gain = 0.0;
        double best_measurement = 0.0;
        int best_index = 0;
        double best_distance = 0.0;

        for (int j = 0; j < z.size(); ++j) {
            if (std::find(subset.data(), subset.data() + subset.size(), z(j)) == subset.data() + subset.size() &&
                selectedIndex.count(j) == 0) 
            {
                Eigen::VectorXd candidate_subset(subset.size() + 1);
                candidate_subset << subset, z(j);
                double info_gain = info_gain_lambda(candidate_subset);

                // compute distance distribution
                double d = computeMinDist(x, y, j, selectedIndex);
                double beta = 0.01;
                info_gain += d * beta;

                // keep track of best sample 
                if (info_gain > max_information_gain) 
                {
                    max_information_gain = info_gain;
                    best_measurement = z(j);
                    best_index = j;
                    best_distance = d;
                }
            }
        }
        // Add the best measurement to the subset
        subset.conservativeResize(subset.size() + 1);
        subset(subset.size() - 1) = best_measurement;
        selectedIndex.insert(best_index);
        distance.conservativeResize(distance.size() + 1);
        distance(distance.size() - 1) = best_distance;
    }

    return selectedIndex;
}

double DataCompression::calculate_entropy(const Eigen::VectorXd& probabilities) 
{
    double entropy = 0.0;
    for (int i = 0; i < probabilities.size(); ++i) {
        if (probabilities(i) > 0.0) {
            entropy -= probabilities(i) * log2(probabilities(i));
        }
    }
    return entropy;
}

double DataCompression::conditional_entropy(const Eigen::VectorXd& y, const Eigen::VectorXd& x) 
{
    // Calculate H(Y|X)
    return calculate_entropy(y) - calculate_entropy(x);
}

double DataCompression::computeMinDist(const Eigen::VectorXd& x, const Eigen::VectorXd& y, int currentIndex, std::unordered_set<int>& selectedIndex) 
{
    if (selectedIndex.size() == 0) {
        return 0.0;
    }

    double dist = std::numeric_limits<double>::infinity();

    Eigen::Vector2d X0(x(currentIndex), y(currentIndex));
    for (const auto& cand: selectedIndex) 
    {
        Eigen::Vector2d X1(x(cand), y(cand));
        dist = std::min(dist, (X1 - X0).norm());
    }
    return dist;
}