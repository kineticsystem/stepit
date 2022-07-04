/*
 * Copyright (C) 2014 Remigi Giovanni
 * g.remigi@kineticsystem.org
 * www.kineticsystem.org
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <QApplication>
#include <QtCore>
#include <QtDebug>
#include <QThreadPool>
#include <QSerialPort>
#include "MainWindow.h"
#include "CrcUtils.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    app.setOrganizationName("Kineticsystem");
    app.setOrganizationDomain("kineticsystem.org");
    app.setApplicationName("StepIt - Copyright Giovanni Remigi");

    MainWindow controllerScreen;
    controllerScreen.show();

    return app.exec();
}
