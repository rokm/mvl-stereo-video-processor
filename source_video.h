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

#ifndef MVL_STEREO_PROCESSOR__SOURCE_VIDEO_H
#define MVL_STEREO_PROCESSOR__SOURCE_VIDEO_H

#include "source.h"

#include <opencv2/videoio.hpp>


namespace MVL {
namespace StereoProcessor {


class SourceVideo : public Source
{
public:
    SourceVideo (const QString &filename);
    virtual ~SourceVideo ();

    virtual void getFrame (int frame, cv::Mat &imageLeft, cv::Mat &imageRight);

protected:
    cv::VideoCapture capture;
    cv::Mat image;
};


} // StereoProcessor
} // MVL


#endif