#pragma once
#ifndef MESSAGEHANDLER_H
#define MESSAGEHANDLER_H

#include <QFile>
#include <QTextStream>
#include <QtDebug>

void messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg);

#endif // MESSAGEHANDLER_H
