//
// Created by redwan on 11/18/23.
//

#ifndef INFOTHEOPLANNER_DATALOADER_H
#define INFOTHEOPLANNER_DATALOADER_H

#include <vector>
#include <memory>
#include <cassert>
#include "rapidcsv.h"


class DataLoader{
public:
    DataLoader(const std::string& csvfile)
    {
        doc_ = std::make_unique<rapidcsv::Document>(csvfile, rapidcsv::LabelParams(-1, -1));
        std::vector<float> row = doc_->GetRow<float>(0);
        std::vector<float> col = doc_->GetColumn<float>(0);
        size_ = std::make_pair(col.size(), row.size() );

    }
    float operator()(int i, int j) const
    {
        assert(i < size_.first && j < size_.second);
        return doc_->GetCell<float>(j, i);
    }
    std::pair<size_t, size_t> size()const
    {
        return size_;
    }
private:
    std::unique_ptr<rapidcsv::Document> doc_;
    std::pair<size_t, size_t> size_;
};

#endif //INFOTHEOPLANNER_DATALOADER_H
