/*
 * MVL Stereo Processor: input source: video
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

#include "source_video.h"

namespace MVL {
namespace StereoProcessor {


SourceVideo::SourceVideo (const QString &filename)
    : Source(filename)
{
    capture.open(filename.toStdString());
    if (!capture.isOpened()) {
        throw QString("Failed to open video source %1").arg(filename);
    }
}

SourceVideo::~SourceVideo ()
{
}


void SourceVideo::getFrame (int frame, cv::Mat &imageLeft, cv::Mat &imageRight)
{
    // Rewind back, if necessary (forward seek is currently implemented
    // as skipping frames)
    int pos = capture.get(cv::CAP_PROP_POS_FRAMES);
    if (frame < pos) {
        capture.set(cv::CAP_PROP_POS_FRAMES, frame);
    }

    while (true) {
        if (!capture.grab()) {
            throw QString("Failed to retrieve frame!");
        }

        if (capture.get(cv::CAP_PROP_POS_FRAMES) > frame) {
            break;
        }
    }

    capture.retrieve(image);

    // Split frame into left and right
    image(cv::Rect(0, 0, image.cols/2, image.rows)).copyTo(imageLeft);
    image(cv::Rect(image.cols/2, 0, image.cols/2, image.rows)).copyTo(imageRight);
}



} // StereoProcessor
} // MVL
