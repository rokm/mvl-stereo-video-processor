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

#include "processor.h"
#include "debug.h"
#include "utils.h"

#include "source_image.h"
#include "source_video.h"
#include "source_vrms.h"

#include <stereo-pipeline/pipeline.h>
#include <stereo-pipeline/plugin_manager.h>
#include <stereo-pipeline/plugin_factory.h>
#include <stereo-pipeline/utils.h>

#include <opencv2/imgcodecs.hpp>


namespace MVL {
namespace StereoProcessor {


Processor::Processor ()
{
}

Processor::~Processor ()
{
}


// *********************************************************************
// *                            Main function                          *
// *********************************************************************
void Processor::run ()
{
    // Parse command-line arguments
    parseCommandLine();

    // Display options
    qCInfo(mvlStereoProcessor) << "";
    qCInfo(mvlStereoProcessor) << "Input file:" << inputFile;
    qCInfo(mvlStereoProcessor) << "Input file type:" << inputFileType;
    qCInfo(mvlStereoProcessor) << "";
    qCInfo(mvlStereoProcessor) << "Stereo calibration file:" << stereoCalibrationFile;
    qCInfo(mvlStereoProcessor) << "Stereo method config file:" << stereoMethodFile;
    qCInfo(mvlStereoProcessor) << "";
    qCInfo(mvlStereoProcessor) << "Frame range(s):";
    for (const FrameRange &range : frameRanges) {
        qCInfo(mvlStereoProcessor) << " *" << range.start << "to" << range.end << "with step" << range.step;
    }
    qCInfo(mvlStereoProcessor) << "";
    qCInfo(mvlStereoProcessor) << "Output frame format(s):";
    for (const QString &format : outputFrames) {
        qCInfo(mvlStereoProcessor) << " *" << format;
    }
    qCInfo(mvlStereoProcessor) << "Output rectified format(s):";
    for (const QString &format : outputRectified) {
        qCInfo(mvlStereoProcessor) << " *" << format;
    }
    qCInfo(mvlStereoProcessor) << "Output disparity format(s):";
    for (const QString &format : outputDisparity) {
        qCInfo(mvlStereoProcessor) << " *" << format;
    }
    qCInfo(mvlStereoProcessor) << "Output points format(s):";
    for (const QString &format : outputPoints) {
        qCInfo(mvlStereoProcessor) << " *" << format;
    }
    qCInfo(mvlStereoProcessor) << "";

    // Validate options
    validateOptions();

    // Setup pipeline
    setupPipeline();

    // Process
    for (const FrameRange &range : frameRanges) {
        qCInfo(mvlStereoProcessor) << "";
        qCInfo(mvlStereoProcessor) << "Processing frame range:" << range.start << "to" << range.end << "with step" << range.step;

        processFrameRange(range);

        qCInfo(mvlStereoProcessor) << "Done!";
    }
}


// *********************************************************************
// *                        Main processing loop                       *
// *********************************************************************
void Processor::processFrameRange (const FrameRange &range)
{
    cv::Mat imageLeft, imageRight;
    cv::Mat rectifiedLeft, rectifiedRight;
    cv::Mat disparity;
    cv::Mat points;

    int numDisparities;

    // Variable map for filename formatting
    QHash<QString, QVariant> variableMap;
    variableMap["rangeStart"] = range.start;
    variableMap["rangeEnd"] = range.end;
    variableMap["rangeStep"] = range.step;

    for (int frame = range.start; range.end < 0 || frame <= range.end; frame += range.step) {
        variableMap["f"] = frame;

        qCDebug(mvlStereoProcessor) << "Processing frame" << frame;

        // *** Grab frames ***
        try {
            inputSource->getFrame(frame, imageLeft, imageRight);
        } catch (const QString &error) {
            // If range is open-ended, just break; otherwise, propagate
            // the error
            if (range.end < 0) {
                qCInfo(mvlStereoProcessor) << "Reached end of sequence!";
                break;
            } else {
                throw error;
            }
        }

        // Export frames
        for (const QString &format : outputFrames) {
            // Left
            variableMap["s"] = "L";
            QString filenameLeft = Utils::formatString(format, variableMap);
            Utils::ensureParentDirectoryExists(filenameLeft);

            if (!cv::imwrite(filenameLeft.toStdString(), imageLeft)) {
                throw QString("Failed to write output image '%1'").arg(filenameLeft);
            }


            // Right
            variableMap["s"] = "R";
            QString filenameRight = Utils::formatString(format, variableMap);
            Utils::ensureParentDirectoryExists(filenameRight);

            if (!cv::imwrite(filenameRight.toStdString(), imageRight)) {
                throw QString("Failed to write output image '%1'").arg(filenameRight);
            }
        }

        // *** Undistort frames ***
        if (stereoRectification) {
            // Rectify
            stereoRectification->rectifyImagePair(imageLeft, imageRight, rectifiedLeft, rectifiedRight);
        } else {
            // Passthrough (assume images are already rectified)
            rectifiedLeft = imageLeft;
            rectifiedRight = imageRight;
        }

        // Export rectified frames
        for (const QString &format : outputRectified) {
            // Left
            variableMap["s"] = "L";
            QString filenameLeft = Utils::formatString(format, variableMap);
            Utils::ensureParentDirectoryExists(filenameLeft);

            if (!cv::imwrite(filenameLeft.toStdString(), rectifiedLeft)) {
                throw QString("Failed to write output image '%1'").arg(filenameLeft);
            }


            // Right
            variableMap["s"] = "R";
            QString filenameRight = Utils::formatString(format, variableMap);
            Utils::ensureParentDirectoryExists(filenameRight);

            if (!cv::imwrite(filenameRight.toStdString(), rectifiedRight)) {
                throw QString("Failed to write output image '%1'").arg(filenameRight);
            }
        }


        // *** Compute disparity ***
        if (stereoMethod) {
            // Compute disparity
            qobject_cast<MVL::StereoToolbox::Pipeline::StereoMethod *>(stereoMethod)->computeDisparity(rectifiedLeft, rectifiedRight, disparity, numDisparities);

            // Export disparity
            for (const QString &format : outputDisparity) {
                QString filename = Utils::formatString(format, variableMap);
                QString ext = QFileInfo(filename).completeSuffix();

                Utils::ensureParentDirectoryExists(filename);

                if (ext == "xml" || ext == "yml" || ext == "yaml") {
                    // Save raw disparity in OpenCV storage format
                    try {
                        cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
                        fs << "disparity" << disparity;
                    } catch (const cv::Exception &error) {
                        throw QString("Failed to save matrix to file %1: %2").arg(filename).arg(QString::fromStdString(error.what()));
                    }
                } else if (ext == "bin") {
                    // Save raw disparity in custom binary matrix format
                    try {
                        MVL::StereoToolbox::Utils::writeMatrixToBinaryFile(disparity, filename);
                    } catch (const QString &error) {
                        throw QString("Failed to save binary file %1: %2").arg(filename).arg(error);
                    }
                } else {
                    // Save disparity visualization as image using cv::imwrite
                    try {
                        cv::imwrite(filename.toStdString(), disparity);
                    } catch (const cv::Exception &error) {
                        throw QString("Failed to save image %1: %2").arg(filename).arg(QString::fromStdString(error.what()));
                    }
                }
            }
        } else {
            continue; // No further steps possible if stereo method is not active
        }

        // *** Reproject point cloud ***
        if (stereoReprojection) {
            stereoReprojection->reprojectStereoDisparity(disparity, points);

            // Export point cloud
            for (const QString &format : outputPoints) {
                QString filename = Utils::formatString(format, variableMap);
                QString ext = QFileInfo(filename).completeSuffix();

                Utils::ensureParentDirectoryExists(filename);

                if (ext == "xml" || ext == "yml" || ext == "yaml") {
                    // Save raw matrix in OpenCV storage format
                    try {
                        cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
                        fs << "points" << points;
                    } catch (const cv::Exception &error) {
                        throw QString("Failed to save matrix to file %1: %2").arg(filename).arg(QString::fromStdString(error.what()));
                    }
                } else if (ext == "bin") {
                    // Save raw matrix in custom binary matrix format
                    try {
                        MVL::StereoToolbox::Utils::writeMatrixToBinaryFile(points, filename);
                    } catch (const QString &error) {
                        throw QString("Failed to save binary file %1: %2").arg(filename).arg(error);
                    }
                } else if (ext == "pcd") {
                    try {
                        MVL::StereoToolbox::Utils::writePointCloudToPcdFile(rectifiedLeft, points, filename, true);
                    } catch (const QString &error) {
                        throw QString("Failed to save PCD file %1: %2").arg(filename).arg(error);
                    }
                } else {
                    throw QString("Invalid output format for reprojection: %1").arg(ext);
                }
            }
        }
    }
}


// *********************************************************************
// *                          Pipeline setup                           *
// *********************************************************************
void Processor::setupPipeline ()
{
    qCDebug(mvlStereoProcessor) << "Setting up pipeline...";

    // Create input source
    if (inputFileType == "image") {
        inputSource = new SourceImage(inputFile);
    } else if (inputFileType == "vrms") {
        inputSource = new SourceVrms(inputFile);
    } else if (inputFileType == "video") {
        inputSource = new SourceVideo(inputFile);
    } else {
        throw QString("Unhandled input source type: %1").arg(inputFileType);
    }

    // Create rectitifaction and load stereo calibration
    if (!stereoCalibrationFile.isEmpty()) {
        qCDebug(mvlStereoProcessor) << "Setting up rectification:" << qPrintable(stereoCalibrationFile);

        stereoRectification = new MVL::StereoToolbox::Pipeline::Rectification();
        try {
            stereoRectification->loadStereoCalibration(stereoCalibrationFile);
        } catch (const QString &error) {
            throw QString("Failed to load stereo calibration: %1").arg(error);
        } catch (const std::exception &error) {
            throw QString("Failed to load stereo calibration: %1").arg(error.what());
        }
    }

    // Create stereo method
    if (!stereoMethodFile.isEmpty()) {
        qCDebug(mvlStereoProcessor) << "Setting up stereo method:" << qPrintable(stereoMethodFile);

        // Open config file
        cv::FileStorage storage(stereoMethodFile.toStdString(), cv::FileStorage::READ);
        if (!storage.isOpened()) {
            throw QString("Failed to open OpenCV file storage on '%1'").arg(stereoMethodFile);
        }

        std::string methodName;
        storage["MethodName"] >> methodName;

        // Traverse list of plugins and try to find stereo method based
        // on its name
        MVL::StereoToolbox::Pipeline::PluginManager pluginManager;
        for (QObject *pluginFactoryObject : pluginManager.getAvailablePlugins()) {
            MVL::StereoToolbox::Pipeline::PluginFactory *pluginFactory = qobject_cast<MVL::StereoToolbox::Pipeline::PluginFactory *>(pluginFactoryObject);
            if (pluginFactory->getPluginType() == MVL::StereoToolbox::Pipeline::PluginFactory::PluginStereoMethod) {
                if (pluginFactory->getShortName().toStdString() == methodName) {
                    stereoMethod = pluginFactory->createObject();
                    break;
                }
            }
        }

        if (!stereoMethod) {
            throw QString("Plugin for stereo method '%1' not found!").arg(QString::fromStdString(methodName));
        }

        // Load config
        try {
            qobject_cast<MVL::StereoToolbox::Pipeline::StereoMethod *>(stereoMethod)->loadParameters(stereoMethodFile);
        } catch (const QString &error) {
            throw QString("Failed to load method parameters: %1").arg(error);
        }
    }

    // Create reprojection (only if we have rectification available!)
    if (stereoRectification) {
        qCDebug(mvlStereoProcessor) << "Setting up reprojection object...";
        stereoReprojection = new MVL::StereoToolbox::Pipeline::Reprojection();
        stereoReprojection->setReprojectionMatrix(stereoRectification->getReprojectionMatrix());
    }
}


// *********************************************************************
// *                        Command-line parser                        *
// *********************************************************************
Processor::FrameRange Processor::parseFrameRange (const QString &range) const
{
    // Split on colon(s)
    QStringList tokens = range.split(":");
    if (tokens.size() != 2 && tokens.size() != 3) {
        throw QString("Invalid frame range string '%1'").arg(range);
    }

    int start = 0;
    int step = 1;
    int end = -1;
    bool ok;

    if (tokens.size() == 2) {
        // Two-token version: start:end
        if (!tokens[0].isEmpty()) {
            start = tokens[0].toInt(&ok);
            if (!ok) {
                throw QString("Invalid number token in frame range: '%1'").arg(tokens[0]);
            }
        }
        if (!tokens[1].isEmpty()) {
            end = tokens[1].toInt(&ok);
            if (!ok) {
                throw QString("Invalid number token in frame range: '%1'").arg(tokens[1]);
            }
        }
    } else {
        // Three-token version: start:step:end
        if (!tokens[0].isEmpty()) {
            start = tokens[0].toInt(&ok);
            if (!ok) {
                throw QString("Invalid number token in frame range: '%1'").arg(tokens[0]);
            }
        }
        if (!tokens[1].isEmpty()) {
            step = tokens[1].toInt(&ok);
            if (!ok) {
                throw QString("Invalid number token in frame range: '%1'").arg(tokens[1]);
            }
        }
        if (!tokens[2].isEmpty()) {
            end = tokens[2].toInt(&ok);
            if (!ok) {
                throw QString("Invalid number token in frame range: '%1'").arg(tokens[2]);
            }
        }
    }

    return FrameRange({ start, step, end });
}


void Processor::parseCommandLine ()
{
    // *** Setup command-line parser ***
    parser.setSingleDashWordOptionMode(QCommandLineParser::ParseAsLongOptions);
    parser.setApplicationDescription("MVL Stereo Processor");
    parser.addHelpOption();
    parser.addVersionOption();

    // Input file
    parser.addPositionalArgument("input-file", QCoreApplication::translate("main", "Input file."));

    // Input type
    QCommandLineOption optionInputType("input-type",
        QCoreApplication::translate("main", "Input file type (image, video, vrms)."),
        QCoreApplication::translate("main", "type"));
    parser.addOption(optionInputType);

    // Stereo calibration
    QCommandLineOption optionStereoCalibration("stereo-calibration",
        QCoreApplication::translate("main", "Stereo calibration file."),
        QCoreApplication::translate("main", "file"));
    parser.addOption(optionStereoCalibration);

    // Stereo method
    QCommandLineOption optionStereoMethod("stereo-method",
        QCoreApplication::translate("main", "Stereo method configuration file."),
        QCoreApplication::translate("main", "file"));
    parser.addOption(optionStereoMethod);

    // Frame range
    QCommandLineOption optionFrameRange(QStringList() << "f" << "frame-range",
        QCoreApplication::translate("main", "Frame range to process."),
        QCoreApplication::translate("main", "start:step:end"));
    optionFrameRange.setDefaultValue("0:1:-1");
    parser.addOption(optionFrameRange);

    // Output: frames
    QCommandLineOption optionOutputFrames("output-frames",
        QCoreApplication::translate("main", "Output format for extracted frames."),
        QCoreApplication::translate("main", "format"));
    parser.addOption(optionOutputFrames);

    // Output: rectified
    QCommandLineOption optionOutputRectified("output-rectified",
        QCoreApplication::translate("main", "Output format for rectified frames."),
        QCoreApplication::translate("main", "format"));
    parser.addOption(optionOutputRectified);

    // Output: disparity
    QCommandLineOption optionOutputDisparity("output-disparity",
        QCoreApplication::translate("main", "Output format for disparity."),
        QCoreApplication::translate("main", "format"));
    parser.addOption(optionOutputDisparity);

    // Output: points
    QCommandLineOption optionOutputPoints("output-points",
        QCoreApplication::translate("main", "Output format for point cloud."),
        QCoreApplication::translate("main", "format"));
    parser.addOption(optionOutputPoints);

    // *** Process ***
    parser.process(*qApp);

    // *** Gather options ***
    inputFileType = parser.value(optionInputType);
    stereoCalibrationFile = parser.value(optionStereoCalibration);
    stereoMethodFile = parser.value(optionStereoMethod);

    outputFrames = parser.values(optionOutputFrames);
    outputRectified = parser.values(optionOutputRectified);
    outputDisparity = parser.values(optionOutputDisparity);
    outputPoints = parser.values(optionOutputPoints);

    // Parse frame range(s)
    for (const QString &range : parser.values(optionFrameRange)) {
        frameRanges.append(parseFrameRange(range));
    }

    // We require exactly one positional argument
    QStringList positionalArguments = parser.positionalArguments();
    if (positionalArguments.size() != 1) {
        throw QString("Exactly one positional argument (input-file) is required; %1 were provided!").arg(positionalArguments.size());
    }

    inputFile = positionalArguments[0];
}

void Processor::validateOptions ()
{
    // Validate input file type string, if provided
    if (!inputFileType.isEmpty()) {
        if (inputFileType != "image" &&
            inputFileType != "video" &&
            inputFileType != "vrms") {
            throw QString("Invalid input file type specified: '%1'").arg(inputFileType);
        }
    } else {
        QString suffix = QFileInfo(inputFile).suffix();
        if (suffix == "jpeg" || suffix == "jpg" || suffix == "png" || suffix == "ppm" || suffix == "bmp") {
            inputFileType = "image";
        } else if (suffix == "vrms") {
            inputFileType = "vrms";
        } else if (suffix == "avi" || suffix == "mp4" || suffix == "mkv" || suffix == "mpg") {
            inputFileType = "video";
        } else {
            throw QString("Unrecognized input file type; unhandled suffix '%1'").arg(suffix);
        }
        qCDebug(mvlStereoProcessor) << "Auto-determined input type:" << inputFileType;
    }

    // Is some output required?
    if (outputFrames.isEmpty() && outputRectified.isEmpty() &&
        outputDisparity.isEmpty() && outputPoints.isEmpty()) {
        throw QString("No output formats specified; nothing to do!");
    }

    // Stereo calibration is needed for rectified images, disparity, and reprojected points
    if (stereoCalibrationFile.isEmpty()) {
        if (!outputRectified.isEmpty()) {
            throw QString("Rectified images output requires stereo calibration!");
        }
        // If stereo calibration is not given, we assume that images are
        // already rectified
        //if (!outputDisparity.isEmpty()) {
        //    throw QString("Disparity output requires stereo calibration!");
        //}
        if (!outputPoints.isEmpty()) {
            throw QString("Reprojected points output requires stereo calibration!");
        }
    }

    // Stereo method is needed for disparity image and reprojected points
    if (stereoMethodFile.isEmpty()) {
        if (!outputDisparity.isEmpty()) {
            throw QString("Disparity output requires stereo method!");
        }
        if (!outputPoints.isEmpty()) {
            throw QString("Reprojected points output requires stereo method!");
        }
    }
}


} // StereoProcessor
} // MVL
