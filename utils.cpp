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

#include "utils.h"

namespace MVL {
namespace StereoProcessor {
namespace Utils {


// Universal string formatter
QString formatString (const QString &format, const QHash<QString, QVariant> &dictionary)
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
                    replacement = QString::asprintf(fmt.toLatin1().constData(), d->toString().toUtf8().constData());
                    break;
                }
                case QVariant::UInt:
                case QVariant::Int: {
                    QString fmt = "%d";
                    if (match.lastCapturedIndex() > 1) {
                       fmt = QString("%") + match.capturedRef(2).toString();
                    }
                    replacement = QString::asprintf(fmt.toLatin1().constData(), d->toInt());
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


// Create parent directory if it does not exist
void ensureParentDirectoryExists (const QString &filename)
{
    QDir rootDir = QFileInfo(filename).absoluteDir();
    if (!rootDir.mkpath(".")) {
        throw QString("Failed to create directory '%1'").arg(filename);
    }
}


} // Utils
} // StereoProcessor
} // MVL
