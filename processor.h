/*
 * MVL Stereo Processor: processor
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

#ifndef MVL_STEREO_PROCESSOR__PROCESSOR_H
#define MVL_STEREO_PROCESSOR__PROCESSOR_H

#include <QtCore>

#include <stereo-pipeline/rectification.h>
#include <stereo-pipeline/reprojection.h>
#include <stereo-pipeline/stereo_method.h>


namespace MVL {
namespace StereoProcessor {


class Source;

class Processor
{
public:
    Processor ();
    virtual ~Processor ();

    void run ();

protected:
    struct FrameRange {
        int start;
        int step;
        int end;
    };
    FrameRange parseFrameRange (const QString &range) const;

    void parseCommandLine ();
    void validateOptions ();
    void setupPipeline ();
    void processFrameRange (const FrameRange &frameRange);

protected:
    QCommandLineParser parser;

    // Input
    QString inputFile;
    QString inputFileType;

    // Config files
    QString stereoCalibrationFile;
    QString stereoMethodFile;

    // Ranges of frames to process
    QVector<FrameRange> frameRanges;

    // Output formats
    QStringList outputFrames;
    QStringList outputRectified;
    QStringList outputDisparity;
    QStringList outputPoints;

    // Pipeline
    QPointer<Source> inputSource;

    QPointer<MVL::StereoToolbox::Pipeline::Rectification> stereoRectification;
    QPointer<MVL::StereoToolbox::Pipeline::Reprojection> stereoReprojection;

    QPointer<QObject> stereoMethod;
};


} // StereoProcessor
} // MVL


#endif
