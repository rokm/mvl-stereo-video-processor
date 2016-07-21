/*
 * MVL Stereo Processor: input source: VRMS video file
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

#include "source_vrms.h"

namespace MVL {
namespace StereoProcessor {


SourceVrms::SourceVrms (const QString &filename)
    : Source(filename)
{
#ifdef ENABLE_VRMS
    reader = new MVL::VRMS::Reader(this);
    if (!reader->openFile(filename)) {
        throw QString("Failed to open VRMS file '%1'").arg(filename);
    }
    reader->buildSeekTable();
#else
    throw QString("VRMS support not enabled!");
#endif
}

SourceVrms::~SourceVrms ()
{
}


void SourceVrms::getFrame (int frame, cv::Mat &imageLeft, cv::Mat &imageRight)
{
    // Seek to frame
    try {
        reader->setVideoPosition(frame);
    } catch (const QString &error) {
        throw QString("VRMS reader error: %1").arg(error);
    }

    // Get images
    reader->getImages(imageLeft, imageRight);
}


} // StereoProcessor
} // MVL
