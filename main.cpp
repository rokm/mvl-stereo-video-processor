/*
 * MVL Stereo Video Processor
 * Copyright (C) 2014-2015 Rok Mandeljc
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

#include <QtCore>

#include <boost/any.hpp>
#include <boost/program_options.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <stereo-pipeline/pipeline.h>
#include <stereo-pipeline/plugin_manager.h>
#include <stereo-pipeline/plugin_factory.h>
#include <stereo-pipeline/rectification.h>
#include <stereo-pipeline/reprojection.h>
#include <stereo-pipeline/stereo_method.h>
#include <stereo-pipeline/utils.h>

#include <iostream>
#include <fstream>
#include <string>

#ifdef ENABLE_VRMS
#include <vrms/reader.h>
#endif


// Add support for QString to boost::program_options
static void validate (boost::any &v, const std::vector<std::string> &values, QString *, int)
{
    using namespace boost::program_options;

    // Make sure no previous assignment to 'a' was made.
    validators::check_first_occurrence(v);

    // Extract the first string from 'values'. If there is more than
    // one string, it's an error, and exception will be thrown.
    const std::string &s = validators::get_single_string(values);

    v = boost::any(QString::fromStdString(s));
}

// Universal string formatter
namespace Utils {

static QString formatString (const QString &format, const QHash<QString, QVariant> &dictionary)
{
    QString output;
    QRegularExpression placeholder("\\%\\{(?<type>\\w+)(?:\\|(?<format>\\w+))?\\}");

    int index = 0;

    QRegularExpressionMatchIterator m = placeholder.globalMatch(format);
    while (m.hasNext()) {
        QRegularExpressionMatch match = m.next();

        output += QStringRef(&format, index, match.capturedStart() - index);

        // Search for substitution
        auto d = dictionary.find(match.capturedRef(1).toString());
        if (d != dictionary.end()) {
            QString replacement;
            switch (d->type()) {
                case QVariant::String: {
                    QString fmt = "%s";
                    if (match.lastCapturedIndex() > 1) {
                       fmt = QString("%") + match.capturedRef(2).toString();
                    }
                    replacement.sprintf(fmt.toLatin1().constData(), d->toString().toUtf8().constData());
                    break;
                }
                case QVariant::UInt:
                case QVariant::Int: {
                    QString fmt = "%d";
                    if (match.lastCapturedIndex() > 1) {
                       fmt = QString("%") + match.capturedRef(2).toString();
                    }
                    replacement.sprintf(fmt.toLatin1().constData(), d->toInt());
                    break;
                }
                default: {
                    // Do nothing
                    break;
                }
            }

            output += replacement;
        } else {
            // Keep the un-substituted token
            output += match.captured();
        }

        index = match.capturedEnd();
    }

    output += QStringRef(&format, index, format.length() - index);

    return output;
}

};


// *********************************************************************
// *                             Processor                             *
// *********************************************************************
static int run_processor (int argc, char **argv)
{
    // *** Variables ***
    QString inputCombined;
    QString inputLeft;
    QString inputRight;

    QString stereoCalibrationFile;
    QString stereoMethodFile;

    int startFrame, endFrame;

    // Calibration
    float calibrationAlpha;
    bool calibrationZeroDisparity;

    // Output
    bool combineRectifiedImages;
    QString outputFormatRectified;
    QString outputFormatDisparity;
    QString outputFormatReprojected;

    MVL::StereoToolbox::Pipeline::Rectification *stereoRectification = NULL;
    QObject *stereoMethod = NULL;
    MVL::StereoToolbox::Pipeline::Reprojection *stereoReprojection = NULL;

    // Video source
    bool useCombinedSource;
    bool useVrmsSource;
    cv::VideoCapture captureCombined, captureLeft, captureRight;
#ifdef ENABLE_VRMS
    MVL::VRMS::Reader vrmsReader;
#endif

    // *** Command-line parser ***
    boost::program_options::options_description commandLineArguments("USV video processor");
    boost::program_options::variables_map optionsMap;

    // Setup command-line arguments
    boost::program_options::options_description argMandatory("Mandatory arguments");

    argMandatory.add_options()
        ("input-combined", boost::program_options::value<QString>(&inputCombined), "name of input video (combined)")
        ("input-left", boost::program_options::value<QString>(&inputLeft), "name of input video (left)")
        ("input-right", boost::program_options::value<QString>(&inputRight), "name of input video (right)")
        ("stereo-calibration", boost::program_options::value<QString>(&stereoCalibrationFile), "stereo calibration file")
        ("stereo-method-config", boost::program_options::value<QString>(&stereoMethodFile), "stereo method configuration file")
    ;
    commandLineArguments.add(argMandatory);

    boost::program_options::options_description argOptional("Optional arguments");
    argOptional.add_options()
        ("help", "produce help message")
        ("config-file", boost::program_options::value<std::string>(), "config file")
        ("start-frame", boost::program_options::value<int>(&startFrame)->default_value(-1), "start frame")
        ("end-frame", boost::program_options::value<int>(&endFrame)->default_value(-1), "end frame")
        ("combine-rectified-images", boost::program_options::bool_switch(&combineRectifiedImages), "combine rectified images back to a single frame")
        ("output-format-rectified", boost::program_options::value<QString>(&outputFormatRectified), "output format for rectified images. %{f} and %{s} tokens are substituted with frame number and side, respectively.")
        ("output-format-disparity", boost::program_options::value<QString>(&outputFormatDisparity), "output format for disparity images. %{f} token is substituted with frame number.")
        ("output-format-reprojected", boost::program_options::value<QString>(&outputFormatReprojected), "output format for reprojected points. %{f} token is substituted with frame number.")
        ("calibration-alpha", boost::program_options::value<float>(&calibrationAlpha)->default_value(0.0f), "free scaling parameter for stereo pair rectification, ranging from 0 to 1.")
        ("calibration-zero-disparity", boost::program_options::value<bool>(&calibrationZeroDisparity)->default_value(true), "whether to set CALIB_ZERO_DISPARITY flag when setting up stereo calibration or not.")
    ;
    commandLineArguments.add(argOptional);

    // Parse command-line
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, commandLineArguments), optionsMap);

    // Display help?
    if (optionsMap.count("help")) {
        std::cout << commandLineArguments << std::endl;
        return 1;
    }

    // Optionally, parse config file (does not override command line!)
    if (optionsMap.count("config-file")) {
        std::string configFile = optionsMap["config-file"].as<std::string>();
        std::ifstream configFileStream(configFile.c_str());
        boost::program_options::store(boost::program_options::parse_config_file(configFileStream, commandLineArguments), optionsMap);
    }

    // Validate
    boost::program_options::notify(optionsMap);

    // We need either combined video or left/right
    if (inputCombined.isEmpty() && (inputLeft.isEmpty() && inputRight.isEmpty())) {
        throw QString("Either combined or left/right input videos are required!");
    }

    // Stereo calibration is needed for rectified images and for reprojected points
    if (stereoCalibrationFile.isEmpty()) {
        if (!outputFormatRectified.isEmpty()) {
            throw QString("Rectified images output requires stereo calibration!");
        }
        if (!outputFormatReprojected.isEmpty()) {
            throw QString("Reprojected points output requires stereo calibration!");
        }
    }

    // Stereo method is needed for disparity image and for reprojected points
    if (stereoMethodFile.isEmpty()) {
        if (!outputFormatDisparity.isEmpty()) {
            throw QString("Disparity maps output requires stereo method!");
        }
        if (!outputFormatReprojected.isEmpty()) {
            throw QString("Reprojected points output requires stereo method!");
        }
    }

    // *** Setup pipeline ***
    qDebug() << "";
    qDebug() << "Setting up pipeline:";

    if (!inputCombined.isEmpty()) {
        // Using combined video source
        useCombinedSource = true;

        QString suffix = QFileInfo(inputCombined).suffix();
        if (suffix.toLower() == "vrms") {
            useVrmsSource = true;
        } else {
            useVrmsSource = false;
        }
    } else {
        // Using left-right video source
        if (inputLeft.isEmpty() || inputRight.isEmpty()) {
            throw QString("Need both left and right video source!");
        }
        useCombinedSource = false;
    }

    // Create rectitifaction and load stereo calibration
    if (!stereoCalibrationFile.isEmpty()) {
        qDebug() << "Setting up rectification:" << qPrintable(stereoCalibrationFile);
        stereoRectification = new MVL::StereoToolbox::Pipeline::Rectification();
        try {
            stereoRectification->loadStereoCalibration(stereoCalibrationFile);
        } catch (const QString &error) {
            throw QString("Failed to load stereo calibration: %1").arg(error);
        } catch (const std::exception &error) {
            throw QString("Failed to load stereo calibration: %1").arg(error.what());
        }

        qDebug() << " alpha value:" << calibrationAlpha;
        stereoRectification->setAlpha(calibrationAlpha);

        qDebug() << " set zero disparity flag:" << calibrationZeroDisparity;
        stereoRectification->setZeroDisparity(calibrationZeroDisparity);
    }

    // Create stereo method
    if (!stereoMethodFile.isEmpty()) {
        qDebug() << "Setting up stereo method:" << qPrintable(stereoMethodFile);

        // Open config file
        cv::FileStorage storage(stereoMethodFile.toStdString(), cv::FileStorage::READ);
        if (!storage.isOpened()) {
            throw QString("Failed to open OpenCV file storage on '%1'").arg(stereoMethodFile);
        }

        std::string methodName;
        storage["MethodName"] >> methodName;

        // Traverse list of plugins and try to find stereo method based on
        // its name
        MVL::StereoToolbox::Pipeline::PluginManager pluginManager;
        foreach (QObject *pluginFactoryObject, pluginManager.getAvailablePlugins()) {
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
        qDebug() << "Setting up reprojection object...";
        stereoReprojection = new MVL::StereoToolbox::Pipeline::Reprojection();
        stereoReprojection->setReprojectionMatrix(stereoRectification->getReprojectionMatrix());
    }

    // Open videos
    if (useCombinedSource) {
        // Combined
        if (useVrmsSource) {
#ifdef ENABLE_VRMS
            qDebug() << "Opening VRMS video source:" << qPrintable(inputCombined);
            if (!vrmsReader.openFile(inputCombined)) {
                throw QString("Failed to open VRMS video source %1").arg(inputCombined);
            }
#else
            throw QString("VRMS support not available!");
#endif
        } else {
            qDebug() << "Opening combined video source:" << qPrintable(inputCombined);
            captureCombined.open(inputCombined.toStdString());
            if (!captureCombined.isOpened()) {
                throw QString("Failed to open combined video source %1").arg(inputCombined);
            }
        }
    } else {
        // Left
        qDebug() << "Opening left video source:" << qPrintable(inputLeft);
        captureLeft.open(inputLeft.toStdString());
        if (!captureLeft.isOpened()) {
            throw QString("Failed to open left video source %1").arg(inputLeft);
        }

        // Right
        qDebug() << "Opening right video source:" << qPrintable(inputRight);
        captureRight.open(inputRight.toStdString());
        if (!captureRight.isOpened()) {
            throw QString("Failed to open right video source %1").arg(inputRight);
        }
    }

    qDebug() << "";
    qDebug() << "Output formats:";
    qDebug() << " rectified images:" << outputFormatRectified;
    qDebug() << " disparity images:" << outputFormatDisparity;
    qDebug() << " reprojected images:" << outputFormatReprojected;
    qDebug() << "";

    // *** Process ***
    cv::Mat imageIn, imageInL, imageInR;
    cv::Mat imageRectifiedL, imageRectifiedR;
    cv::Mat disparity, reprojection;
    int numDisparities;

    // Debug message about frame range
    {
        QString msg = "Processing from ";
        if (startFrame >= 0) {
            msg += QString("frame %1").arg(startFrame);
        } else {
            msg += "beginning of sequence";
        }
        msg += " to ";

        if (endFrame >= 0) {
            msg += QString("frame %1").arg(endFrame);
        } else {
            msg += "end of sequence";
        }

        msg += "...";
        qDebug() << qPrintable(msg);
    }

    for (int frame = 0; ; frame++) {
        QHash<QString, QVariant> variableMap;
        variableMap["f"] = frame;

        // Read frame
        if (useCombinedSource) {
            if (useVrmsSource) {
#ifdef ENABLE_VRMS
                if (!vrmsReader.grabNextFrame()) {
                    qDebug() << "End of video reached";
                    break;
                }
                vrmsReader.getImages(imageInL, imageInR);
#endif
            } else {
                if (!captureCombined.read(imageIn)) {
                    qDebug() << "End of video reached";
                    break;
                }

                // Split combined frame into left and right
                imageIn(cv::Rect(0, 0, imageIn.cols/2, imageIn.rows)).copyTo(imageInL);
                imageIn(cv::Rect(imageIn.cols/2, 0, imageIn.cols/2, imageIn.rows)).copyTo(imageInR);
            }
        } else {
            if (!captureLeft.read(imageInL) || !captureRight.read(imageInR)) {
                qDebug() << "End of video reached";
                break;
            }
        }

        // Make sure we are within specified range
        if (startFrame >= 0 && frame < startFrame) {
            continue; // Skip
        }

        if (endFrame >= 0 && frame > endFrame) {
            qDebug() << "Specified end frame reached";
            break;
        }

        // Rectification
        if (stereoRectification) {
            stereoRectification->rectifyImagePair(imageInL, imageInR, imageRectifiedL, imageRectifiedR);

            if (!outputFormatRectified.isEmpty()) {
                if (combineRectifiedImages) {
                    // Combine
                    cv::Mat imageRectifiedCombined(imageInL.rows, imageInL.cols*2, imageInL.type());
                    imageRectifiedL.copyTo( imageRectifiedCombined(cv::Rect(0, 0, imageRectifiedCombined.cols/2, imageRectifiedCombined.rows)) );
                    imageRectifiedR.copyTo( imageRectifiedCombined(cv::Rect(imageRectifiedCombined.cols/2, 0, imageRectifiedCombined.cols/2, imageRectifiedCombined.rows)) );

                    // Save
                    QString filename = Utils::formatString(outputFormatRectified, variableMap);
                    cv::imwrite(filename.toStdString(), imageRectifiedCombined);
                } else {
                    // Save left
                    variableMap["s"] = "L";
                    QString filenameLeft = Utils::formatString(outputFormatRectified, variableMap);
                    cv::imwrite(filenameLeft.toStdString(), imageRectifiedL);

                    // Save right
                    variableMap["s"] = "R";
                    QString filenameRight = Utils::formatString(outputFormatRectified, variableMap);
                    cv::imwrite(filenameRight.toStdString(), imageRectifiedR);
                }
            }
        } else {
            // Passthrough
            imageRectifiedL = imageInL;
            imageRectifiedR = imageInR;
        }

        // Stereo method
        if (stereoMethod) {
            // Compute disparity
            qobject_cast<MVL::StereoToolbox::Pipeline::StereoMethod *>(stereoMethod)->computeDisparityImage(imageRectifiedL, imageRectifiedR, disparity, numDisparities);

            if (!outputFormatDisparity.isEmpty()) {
                QString filename = Utils::formatString(outputFormatDisparity, variableMap);
                QString ext = QFileInfo(filename).completeSuffix();

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

        // Reprojection
        if (stereoReprojection) {
            stereoReprojection->reprojectStereoDisparity(disparity, reprojection);

            if (!outputFormatReprojected.isEmpty()) {
                QString filename = Utils::formatString(outputFormatReprojected, variableMap);
                QString ext = QFileInfo(filename).completeSuffix();

                if (ext == "xml" || ext == "yml" || ext == "yaml") {
                    // Save raw matrix in OpenCV storage format
                    try {
                        cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
                        fs << "points" << reprojection;
                    } catch (const cv::Exception &error) {
                        throw QString("Failed to save matrix to file %1: %2").arg(filename).arg(QString::fromStdString(error.what()));
                    }
                } else if (ext == "bin") {
                    // Save raw matrix in custom binary matrix format
                    try {
                        MVL::StereoToolbox::Utils::writeMatrixToBinaryFile(reprojection, filename);
                    } catch (const QString &error) {
                        throw QString("Failed to save binary file %1: %2").arg(filename).arg(error);
                    }
                } else if (ext == "pcd") {
                    try {
                        MVL::StereoToolbox::Utils::writePointCloudToPcdFile(imageRectifiedL, reprojection, filename, true);
                    } catch (const QString &error) {
                        throw QString("Failed to save PCD file %1: %2").arg(filename).arg(error);
                    }
                } else {
                    throw QString("Invalid output format for reprojection: %1").arg(ext);
                }
            }
        }
    }

    return 0;
}

// *********************************************************************
// *                               Main                                *
// *********************************************************************
int main (int argc, char **argv)
{
    try {
        return run_processor(argc, argv);
    } catch (const QString &error) {
        qDebug() << "ERROR:" << qPrintable(error);
        return -1;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return -2;
    } catch (...) {
        std::cerr << "ERROR: unhandled exception!" << std::endl;
        return -3;
    }
}
