/*
 * MVL Stereo Processor: utility functions
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

#ifndef MVL_STEREO_PROCESSOR__UTILS_H
#define MVL_STEREO_PROCESSOR__UTILS_H

#include <QtCore>


namespace MVL {
namespace StereoProcessor {
namespace Utils {

// Universal string formatter
QString formatString (const QString &format, const QHash<QString, QVariant> &dictionary);

// Create parent directory if it does not exist
void ensureParentDirectoryExists (const QString &filename);


} // Utils
} // StereoProcessor
} // MVL


#endif
