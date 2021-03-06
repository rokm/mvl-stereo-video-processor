/*
 * MVL Stereo Processor: main
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


using namespace MVL::StereoProcessor;


int main (int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("MVL Stereo Processor");
    QCoreApplication::setApplicationVersion("1.0");

    qSetMessagePattern("%{message}");

    Processor processor;

    try {
        processor.run();
    } catch (const QString &error) {
        qCWarning(mvlStereoProcessor) << "ERROR:" << qPrintable(error);
        return -1;
    }

    return 0;
}
