////___________________________REV 1: IN WORKING CONDITION______________________________________
/*
* Following works:
*       1.You can set the desired FPS and desired FrameCount from the GUI.
*       2.You can select the required directory for saving the output.
*       3.When you press "Detect" then contour detection is successful and
          the unique contour count gets displayed in the box.
        4.When you press "Display Images" the output images stored in the directory are displayed
*/
/*
#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ImageProcessing.h"
#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QFileDialog>
#include <QTimer>
#include <QMessageBox>
#include <QPixmap>
#include <QObject>
#include <QDir>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QLineEdit>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow* ui;
    QProgressBar progressBar;
    QThread* enqueueThread;
    QThread* processThread1;
    QThread* processThread2;
    QThread* processingThread;
    bool stopThreads;
    bool stopProcessing;
    QString outputDirectory;
    QStringList imageFiles;
    int currentImageIndex;
    QSerialPort* serialPort;
    QString arduinoVendorID;
    QString arduinoProductID;
   
  


    void showImage(const QString& fileName);
    void updateSerialPortInfo();
    bool openSerialPort(const QString& portName);
    void closeSerialPort();
    void sendCommandToArduino(const QByteArray& command);

private slots:
    void startProcessing();
    void stopProcessing_1();
    void selectOutputFolder();
    void displayAndNavigateImages();
    void showPreviousImage();
    void showNextImage();
   // void on_speedSlider_valueChanged(int value);
    void handleSerialError(QSerialPort::SerialPortError error);
   // void on_speedSpinBox_valueChanged(int value);          // Slot for spin box value changed
   

  
};

#endif // MAINWINDOW_H

//____________________REV 2: CONVEYER SPEED CONTROL AND DISPLAYING THE OUTPUT IMAGES INORDER_______________
#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ImageProcessing.h"
#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QFileDialog>
#include <QTimer>
#include <QMessageBox>
#include <QPixmap>
#include <QObject>
#include <QDir>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QLineEdit>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow* ui;
    QProgressBar progressBar;
    QThread* enqueueThread;
    QThread* processThread1;
    QThread* processThread2;
    QThread* processingThread;
    QThread* processThread3;
    bool stopThreads;
    bool stopProcessing;
    QString outputDirectory;
    QStringList imageFiles;
    int currentImageIndex;
    QSerialPort* serialPort;
    QString arduinoVendorID;
    QString arduinoProductID;



    void showImage(const QString& fileName);
    void updateSerialPortInfo();
    bool openSerialPort(const QString& portName);
    void closeSerialPort();
    //void sendCommandToArduino(const QByteArray& command);

private slots:
    void startProcessing();
    void stopProcessing_1();
    void selectOutputFolder();
    void displayAndNavigateImages();
    void showPreviousImage();
    void showNextImage();
    void handleSerialError(QSerialPort::SerialPortError error);
    void on_spinBox_valueChanged(int speedValue);
    void sendCommandToArduino(const QString& command);
    QString convertSpeedValueToCommand(int speedValue);
    //void enableDisplayImageButton();
    void resetApplication();
};

#endif // MAINWINDOW_H
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//____________________REV 3: EXACT CONTOUR COUNT_______________
/*
* * REV 3: EXACT COUNT OF CONTOURS GETTING DISPLAYED
* * Following works:
*   1.You get the exact count of the contours displayed, but the problem is we have we minimize the UI again for ther UI to refresh.
*   2.The stopConveyer gets executed properly but there is delay and thus letting more contours fall off, as the UI is not getting refreshed
*/

#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ImageProcessing.h"
#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QFileDialog>
#include <QTimer>
#include <QMessageBox>
#include <QPixmap>
#include <QObject>
#include <QDir>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QLineEdit>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();
public slots:
    void stopConveyor(); // Function to stop the conveyor
private:
    Ui::MainWindow* ui;
   // QProgressBar progressBar;
    QThread* enqueueThread;
    QThread* processThread1;
    QThread* processThread2;
    QThread* processingThread;
    QThread* processThread3;
    bool stopThreads;
    QString outputDirectory;
    QStringList imageFiles;
    int currentImageIndex;
    QSerialPort* serialPort;
    QString arduinoVendorID;
    QString arduinoProductID;
    QString selectedMode;


    QTimer* updateTimer;
    std::atomic<int> totalcounter; // Use atomic for thread-safe operations
    int desiredCount; // Add this variable
   



    void showImage(const QString& fileName);
    void updateSerialPortInfo();
    bool openSerialPort(const QString& portName);
    void closeSerialPort();
    //void sendCommandToArduino(const QByteArray& command);

private slots:
   // void updateUI();
    void startProcessing();
    void stopProcessing_1();
    void selectOutputFolder();
    void displayAndNavigateImages();
    void showPreviousImage();
    void showNextImage();
    void handleSerialError(QSerialPort::SerialPortError error);
    void on_spinBox_valueChanged(int speedValue);
    void sendCommandToArduino(const QString& command);
    QString convertSpeedValueToCommand(int speedValue);
    //void enableDisplayImageButton();
    void resetApplication();
};

#endif // MAINWINDOW_H


////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//____________________REV 4: STOPPING CONVEYER ON EXACT CONTOUR COUNT_______________
/*
* * * Following works:
*   1.Lookup table implementation works properly but as the UI doesnot refresh consistently , thus many contours fall extra even though they are counted properly.
*   2.updateConveyorSpeed is the function that is been written for the lookup table mechanism.
*   3.The serialport communication functions were shifted to the ImageProcessing.cpp because of the updateConveyorSpeed function initial setup trials.
*/
/*
#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ImageProcessing.h"
#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QLabel>
#include <QString>
#include <QTimer> // Include QTimer header
namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
     MainWindow(QWidget* parent = nullptr);
    ~MainWindow();
    bool stopThreads;
    bool stopProcessing;
private slots:
    void startProcessing();
    void stopProcessing_1();
    void resetApplication();
    void selectOutputFolder();
    void displayAndNavigateImages();
    void showNextImage();
    void showPreviousImage();
    void on_spinBox_valueChanged(int speedValue);
    

private:
    Ui::MainWindow* ui;
    QThread* enqueueThread;
    QThread* processThread1;
    QThread* processThread2;
    QThread* processingThread;
    QThread* processThread3;
    QTimer* refreshTimer; // Timer to refresh UI
    QString outputDirectory;
    QStringList imageFiles;
    int currentImageIndex;

};

#endif // MAINWINDOW_H



*/





















//_____________________TRIALS
/*
#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ImageProcessing.h"
#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QFileDialog>
#include <QTimer>
#include <QMessageBox>
#include <QPixmap>
#include <QObject>
#include <QDir>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QLineEdit>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();
public slots:
    void stopConveyor(); // Function to stop the conveyor
private:
    Ui::MainWindow* ui;
    // QProgressBar progressBar;
    QThread* enqueueThread;
    QThread* processThread3;
    QString outputDirectory;
    QStringList imageFiles;
    int currentImageIndex;
    QSerialPort* serialPort;
    QString arduinoVendorID;
    QString arduinoProductID;
    QString selectedMode;


    QTimer* updateTimer;
    std::atomic<int> totalcounter; // Use atomic for thread-safe operations
    int desiredCount; // Add this variable




    void showImage(const QString& fileName);
    void updateSerialPortInfo();
    bool openSerialPort(const QString& portName);
    void closeSerialPort();
    //void sendCommandToArduino(const QByteArray& command);

private slots:
    // void updateUI();
    void startProcessing();
    void stopProcessing_1();
    void selectOutputFolder();
    void displayAndNavigateImages();
    void showPreviousImage();
    void showNextImage();
    void handleSerialError(QSerialPort::SerialPortError error);
    void on_spinBox_valueChanged(int speedValue);
    void sendCommandToArduino(const QString& command);
    QString convertSpeedValueToCommand(int speedValue);
    //void enableDisplayImageButton();
    void resetApplication();
};

#endif // MAINWINDOW_H
*/
////////////////////////////////////////////////////////////////////
//TRIALS
/*
#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ImageProcessing.h"
#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QFileDialog>
#include <QTimer>
#include <QMessageBox>
#include <QPixmap>
#include <QObject>
#include <QDir>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QLineEdit>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();
public slots:
    void stopConveyor(); // Function to stop the conveyor
private:
    Ui::MainWindow* ui;
    // QProgressBar progressBar;
    QThread* enqueueThread;
    QThread* processThread1;
    QThread* processThread2;
    QThread* processingThread;
    QThread* processThread3;
    bool stopThreads;
    bool stopProcessing;
    QString outputDirectory;
    QStringList imageFiles;
    int currentImageIndex;
    QSerialPort* serialPort;
    QString arduinoVendorID;
    QString arduinoProductID;
    QString selectedMode;



    void showImage(const QString& fileName);
    void updateSerialPortInfo();
    bool openSerialPort(const QString& portName);
    void closeSerialPort();
    //void sendCommandToArduino(const QByteArray& command);

private slots:
    // void updateUI();
    void startProcessing();
    void stopProcessing_1();
    void selectOutputFolder();
    void displayAndNavigateImages();
    void showPreviousImage();
    void showNextImage();
    void handleSerialError(QSerialPort::SerialPortError error);
    void on_spinBox_valueChanged(int speedValue);
    void sendCommandToArduino(const QString& command);
    QString convertSpeedValueToCommand(int speedValue);
    //void enableDisplayImageButton();
    void resetApplication();
};

#endif // MAINWINDOW_H
*/