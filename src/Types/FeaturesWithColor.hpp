//
// Created by akarbarc on 03.08.17.
//

#ifndef FEATURESWITHCOLOR_HPP_
#define FEATURESWITHCOLOR_HPP_

#include "Types/Drawable.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace Types {

    class FeaturesWithColor : public Drawable {
    public:
        struct ColorPair {
            cv::Scalar color;
            std::vector<cv::KeyPoint> features;

            ColorPair() {}

            ColorPair(const cv::Scalar & _color, const std::vector<cv::KeyPoint> & _features) :
                    color(_color), features(_features) {}
        };

        FeaturesWithColor() {}

        FeaturesWithColor(const FeaturesWithColor & _features) {
            features.insert(features.end(), _features.features.begin(), _features.features.end());
        }

        FeaturesWithColor(std::vector<ColorPair> & _features) {
            features.insert(features.end(), _features.begin(), _features.end());
        }

        virtual ~FeaturesWithColor() {}

        virtual void draw(cv::Mat & image, cv::Scalar color, int offsetX = 0, int offsetY = 0) {
            for (const ColorPair & entry : features) {
                cv::drawKeypoints(image, entry.features, image, entry.color);
            }
        }

        virtual Drawable * clone() {
            return new FeaturesWithColor(*this);
        }

    protected:
        std::vector<ColorPair> features;
    };

} //: namespace Types



#endif /* FEATURES_HPP_ */
