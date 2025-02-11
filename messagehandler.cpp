#include "messagehandler.h"

void messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    // Open a log file
    QFile logFile("application.log");
    if (logFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
        QTextStream stream(&logFile);
        // Write message to file
        stream << msg << endl;
    }
    // Also output to standard output (console)
    QTextStream(stdout) << msg << endl;
}
