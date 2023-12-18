//
// Created by redwan on 11/18/23.
//

#ifndef INFOTHEOPLANNER_HYPERMAP_H
#define INFOTHEOPLANNER_HYPERMAP_H
#include "dataloader.h"
#include <numeric>
#include <random>

class HyperMap
{
    using MESH = std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>>;
public:
    HyperMap(const std::string& coord_path, const std::string& data_path)
    {
        values_ = std::make_unique<DataLoader>(data_path);
        DataLoader coord(coord_path);
        auto coord_size = coord.size();

        std::vector<float> x;
        std::vector<float> y;

        xmin_ = ymin_ = std::numeric_limits<float>::max();
        xmax_ = ymax_ = std::numeric_limits<float>::min();

        for (int i = 0; i < coord_size.first; ++i) {
            for (int j = 0; j < coord_size.second; ++j) {
                if(j==0)
                {
                    x.emplace_back(coord(i, j));
                    xmax_ = std::max(xmax_, coord(i, j));
                    xmin_ = std::min(xmin_, coord(i, j));
                }
                else
                {
                    y.emplace_back(coord(i, j));
                    ymax_ = std::max(ymax_, coord(i, j));
                    ymin_ = std::min(ymin_, coord(i, j));
                }
            }
        }

        auto [X, Y] = meshgrid(x, y);
        std::copy(X.begin(), X.end(), std::back_inserter(X_));
        std::copy(Y.begin(), Y.end(), std::back_inserter(Y_));
    }
    std::tuple<float, float, float>operator()(int i, int j) const
    {
        return std::make_tuple(X_[i][j], Y_[i][j], values_->operator()(i, j));
    }
    std::pair<size_t, size_t> size() const
    {
        return values_->size();
    }

    std::tuple<float, float, float, float> getBoundingBox() const
    {
        return std::make_tuple(xmin_, xmax_, ymin_, ymax_);
    }


    static MESH meshgrid(const std::vector<float>& x, const std::vector<float>& y) {
        std::vector<std::vector<float>> X(x.size(), std::vector<float>(y.size()));
        std::vector<std::vector<float>> Y(x.size(), std::vector<float>(y.size()));

        for (size_t i = 0; i < x.size(); ++i) {
            for (size_t j = 0; j < y.size(); ++j) {
                X[i][j] = x[i];
                Y[i][j] = y[j];
            }
        }

        return std::make_pair(X, Y);
    }


protected:
    std::vector<std::vector<float>> X_, Y_;
    std::unique_ptr<DataLoader> values_;
    float xmin_, xmax_, ymin_, ymax_;

};


class MetricMap: public HyperMap
{
public:
    MetricMap(const std::string &coordPath, const std::string &dataPath) : HyperMap(coordPath, dataPath) {

    }

    bool isValidCoord(float x, float y) const
    {
        return x >= xmin_ && x < xmax_ && y >= ymin_ && y < ymax_;
    }

    std::pair<float, float> generateRandomCoord() const
    {
        // Seed for the random number generator
        std::random_device rd;
        std::mt19937 generator(rd());

        // Generate random x and y coordinates
        std::uniform_real_distribution<float> distributionX(xmin_, xmax_);
        std::uniform_real_distribution<float> distributionY(ymin_, ymax_);

        float randomX = distributionX(generator);
        float randomY = distributionY(generator);
        return std::make_pair(randomX, randomY);

    }
    std::pair<int, int>getPairedIndex(float x, float y) const
    {
        auto gridSize = values_->size();
        int row = y * gridSize.second / ymax_;
        int column = x * gridSize.first / xmax_;
        return std::make_pair(row, column);
    }

    template<class T>
    std::vector<double> getObsAlongTraj(const std::vector<T>& traj)&
    {
        std::vector<double> result(traj.size());
        std::transform(traj.begin(), traj.end(),
                       result.begin(),[&](const T& val){
            auto [i, j] = getPairedIndex(val(0), val(1));
            float x, y, z;
            std::tie(x, y, z) = this->operator()(i, j);
            return (double )z;
        });
        return result;
    }

    int getIndex(float x, float y) const
    {
        if(!isValidCoord(x, y))
            return -1;

        auto [row, column] = getPairedIndex(x, y);
//        debug(row << " " << column);
        return row * values_->size().second + column;
    }

    std::vector<double> getSub(int index) const
    {
        double size = static_cast<double>(values_->size().second);
        double y = index / size;
        double x = fmod(index, size);
        return {x, y};
    }
};
#endif //INFOTHEOPLANNER_HYPERMAP_H
