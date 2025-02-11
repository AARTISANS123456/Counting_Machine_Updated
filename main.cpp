/*
#include "AA20240615_Counting_Machine.h"
#include <QtWidgets/QApplication>
#include <QCoreApplication>
#include "messagehandler.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    // Install custom message handler
    qInstallMessageHandler(messageHandler);

    // Your application logic
    qDebug() << "Starting application...";
    AA20240615_Counting_Machine w;
    w.show();
    return a.exec();
}*/
///////REV 3: WORKING

#include <QApplication>
#include "MainWindow.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    return app.exec();
}


