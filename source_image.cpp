/*
 * MVL Stereo Processor: input source: image
 * Copyright (C) 2014-2016 Rok Mandeljc
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "source_image.h"
#include "utils.h"

#include <opencv2/imgcodecs.hpp>


namespace MVL {
namespace StereoProcessor {


SourceImage::SourceImage (const QString &filename)
    : Source(filename)
{
}

SourceImage::~SourceImage ()
{
}


void SourceImage::getFrame (int frame, cv::Mat &imageLeft, cv::Mat &imageRight)
{
    QHash<QString, QVariant> variableMap;
    variableMap["f"] = frame;

    // Left image
    variableMap["s"] = "L";
    QString filenameLeft = Utils::formatString(filename, variableMap);

    imageLeft = cv::imread(filenameLeft.toStdString());
    if (imageLeft.empty()) {
        throw QString("Failed to open image '%1'").arg(filenameLeft);
    }

    // Right image
    variableMap["s"] = "R";
    QString filenameRight = Utils::formatString(filename, variableMap);

    imageRight = cv::imread(filenameRight.toStdString());
    if (imageRight.empty()) {
        throw QString("Failed to open image '%1'").arg(filenameRight);
    }
}



} // StereoProcessor
} // MVL
