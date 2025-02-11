
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
#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <QProgressBar>
#include <QLineEdit>
#include <QLabel>
#include <QObject>
using namespace cv;
using namespace std;

extern queue<Mat> buffer1;
extern queue<Mat> buffer2;
extern queue<Mat> buffer3;

extern mutex buffer1_mutex;
extern mutex buffer2_mutex;
extern mutex buffer3_mutex;

extern condition_variable buffer1_cv;
extern condition_variable buffer2_cv;
extern condition_variable buffer3_cv;

extern atomic<bool> stopThreads;
extern atomic<bool> stopProcessing;

struct ContourData {
    int number;
    Rect roi;
    Point2f centroid;
    double area;
};


void setFPSValue(QLineEdit* FPS);        // Function declaration for setting fpsValue
void setFrameCountValue(QLineEdit* FRAME_COUNT); // Function declaration for setting frameCountValue

void enqueueBuffer1(QLineEdit* FPS, QLineEdit* FRAME_COUNT);
void processBuffer1();
void processBuffer2();
void furtherProcessing(const string& outputDirectory,  QLineEdit* countEdit, QLabel* statusLabel);

#endif // IMAGEPROCESSING_H

////////////////////////////////////////////////////////////////////////////////
//____________________REV 2: CONVEYER SPEED CONTROL AND DISPLAYING THE OUTPUT IMAGES INORDER_______________
#pragma once
#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <QProgressBar>
#include <QLineEdit>
#include <QLabel>
#include <QObject>

using namespace cv;
using namespace std;

extern queue<Mat> buffer1;
extern queue<Mat> buffer2;
extern queue<Mat> buffer3;
extern queue<Mat> buffer4;

extern mutex buffer1_mutex;
extern mutex buffer2_mutex;
extern mutex buffer3_mutex;
extern mutex buffer4_mutex;

extern condition_variable buffer1_cv;
extern condition_variable buffer2_cv;
extern condition_variable buffer3_cv;
extern condition_variable buffer4_cv;

extern atomic<bool> stopThreads;
extern atomic<bool> stopProcessing;

struct ContourData {
    int number;
    Rect roi;
    Point2f centroid;
    double area;
    bool wasInTopROI = false; // New flag
    bool wasInBottomROI = false;
};

void setFPSValue(QLineEdit* FPS);        // Function declaration for setting fpsValue
void setFrameCountValue(QLineEdit* FRAME_COUNT); // Function declaration for setting frameCountValue

void enqueueBuffer1(QLineEdit* FPS, QLineEdit* FRAME_COUNT);
void processBuffer1();
void processBuffer2();
void furtherProcessing(QLineEdit* countEdit, QLabel* statusLabel);
void saving(const string& outputDirectory);


#endif // IMAGEPROCESSING_H
*/
//____________________REV 3: EXACT CONTOUR COUNT_______________
/*
* REV 3: EXACT COUNT OF CONTOURS GETTING DISPLAYED
* * Following works:
*   1.You get the exact count of the contours displayed, but the problem is we have we minimize the UI again for ther UI to refresh.
*   2.The stopConveyer gets executed properly but there is delay and thus letting more contours fall off, as the UI is not getting refreshed
*/

#pragma once
#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <QLineEdit>
#include <QLabel>
#include <QObject>

using namespace cv;
using namespace std;

extern queue<Mat> buffer1;
extern queue<Mat> buffer2;
extern queue<Mat> buffer3;
extern queue<Mat> buffer4;

extern mutex buffer1_mutex;
extern mutex buffer2_mutex;
extern mutex buffer3_mutex;
extern mutex buffer4_mutex;

extern condition_variable buffer1_cv;
extern condition_variable buffer2_cv;
extern condition_variable buffer3_cv;
extern condition_variable buffer4_cv;

extern atomic<bool> stopThreads;
extern std::atomic<int> totalcounter;
extern int desiredCount;
extern std::atomic<bool> stopProcessing;
class MainWindow;

struct ContourData {
    int number;
    Rect roi;
    Point2f centroid;
    double area;
    bool wasInTopROI = false; // New flag
    bool wasInBottomROI = false;
};

void setFPSValue(QLineEdit* FPS);        // Function declaration for setting fpsValue
void setFrameCountValue(QLineEdit* FRAME_COUNT); // Function declaration for setting frameCountValue

void enqueueBuffer1(QLineEdit* FPS);
void processBuffer1();
void processBuffer2();
void furtherProcessing(MainWindow* mainWindow, QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT);
void saving(const string& outputDirectory);


#endif // IMAGEPROCESSING_H 



//____________________REV 4: STOPPING THE CONVEYER ON EXACT CONTOUR COUNT_______________
/*
* Following works:
*   1.Lookup table implementation works properly but as the UI doesnot refresh consistently , thus many contours fall extra even though they are counted properly.
*   2.updateConveyorSpeed is the function that is been written for the lookup table mechanism.
*   3.The serialport communication functions were shifted to the ImageProcessing.cpp because of the updateConveyorSpeed function initial setup trials.
*/
/*
#pragma once
#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <QLineEdit>
#include <QLabel>
#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QString>
#include <QDebug>

using namespace cv;
using namespace std;

extern queue<Mat> buffer1;
extern queue<Mat> buffer2;
extern queue<Mat> buffer3;
extern queue<Mat> buffer4;

extern mutex buffer1_mutex;
extern mutex buffer2_mutex;
extern mutex buffer3_mutex;
extern mutex buffer4_mutex;

extern condition_variable buffer1_cv;
extern condition_variable buffer2_cv;
extern condition_variable buffer3_cv;
extern condition_variable buffer4_cv;

extern atomic<bool> stopThreads;
extern std::atomic<int> totalcounter;
extern int desiredCount;
extern std::atomic<bool> stopProcessing;
class MainWindow;

struct ContourData {
    int number;
    Rect roi;
    Point2f centroid;
    double area;
    bool wasInTopROI = false; // New flag
    bool wasInBottomROI = false;
};
void sendCommandToArduino(const QString& command);
bool openSerialPort(const QString& portName);
void closeSerialPort();
void updateSerialPortInfo(QLabel* statusLabel);
QString convertSpeedValueToCommand(int speedValue);
void stopConveyor();

void setFPSValue(QLineEdit* FPS);        // Function declaration for setting fpsValue
void setFrameCountValue(QLineEdit* FRAME_COUNT); // Function declaration for setting frameCountValue
void updateConveyorSpeed(int partCount, int totalParts, QLineEdit* speed_conv, QLineEdit* countEdit, MainWindow* mainWindow);
void enqueueBuffer1(QLineEdit* FPS);
void processBuffer1();
void processBuffer2();
void furtherProcessing(MainWindow* mainWindow, QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT, QLineEdit* speed_conv);
void saving(const string& outputDirectory);



#endif // IMAGEPROCESSING_H

*/
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
#pragma once
#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <QLineEdit>
#include <QLabel>
#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QString>
#include <QDebug>

using namespace cv;
using namespace std;

extern queue<Mat> buffer1;
extern queue<Mat> buffer2;
extern queue<Mat> buffer3;
extern queue<Mat> buffer4;

extern mutex buffer1_mutex;
extern mutex buffer2_mutex;
extern mutex buffer3_mutex;
extern mutex buffer4_mutex;

extern condition_variable buffer1_cv;
extern condition_variable buffer2_cv;
extern condition_variable buffer3_cv;
extern condition_variable buffer4_cv;

extern atomic<bool> stopThreads;
extern std::atomic<int> totalcounter;
extern int desiredCount;
extern std::atomic<bool> stopProcessing;
class MainWindow;

struct ContourData {
    int number;
    Rect roi;
    Point2f centroid;
    double area;
    bool wasInTopROI = false; // New flag
    bool wasInBottomROI = false;
};
void sendCommandToArduino(const std::string& command);

bool openSerialPort(const std::string& portName);
void closeSerialPort();
void updateSerialPortInfo();

std::string convertSpeedValueToCommand(int speedValue);
void stopConveyor();

void setFPSValue(QLineEdit* FPS);        // Function declaration for setting fpsValue
void setFrameCountValue(QLineEdit* FRAME_COUNT); // Function declaration for setting frameCountValue

void enqueueBuffer1(QLineEdit* FPS);
void processBuffer1();
void processBuffer2();
void furtherProcessing( QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT, QLineEdit* speed_conv);
//void saving(const string& outputDirectory);



#endif // IMAGEPROCESSING_H

*/

///////////////////////////////////////////////////////////
//TRIALS
/*
#pragma once
#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <QLineEdit>
#include <QLabel>
#include <QObject>

using namespace cv;
using namespace std;

extern queue<Mat> buffer1;
extern queue<Mat> buffer2;
extern queue<Mat> buffer3;
extern queue<Mat> buffer4;

extern mutex buffer1_mutex;
extern mutex buffer2_mutex;
extern mutex buffer3_mutex;
extern mutex buffer4_mutex;

extern condition_variable buffer1_cv;
extern condition_variable buffer2_cv;
extern condition_variable buffer3_cv;
extern condition_variable buffer4_cv;

extern atomic<bool> stopThreads;
extern std::atomic<int> totalcounter;
extern int desiredCount;
extern atomic<bool> stopProcessing;
class MainWindow;

struct ContourData {
    int number;
    Rect roi;
    Point2f centroid;
    double area;
    bool wasInTopROI = false; // New flag
    bool wasInBottomROI = false;
};

void setFPSValue(QLineEdit* FPS);        // Function declaration for setting fpsValue
void setFrameCountValue(QLineEdit* FRAME_COUNT); // Function declaration for setting frameCountValue

void enqueueBuffer1(QLineEdit* FPS);
void processBuffer1();
void processBuffer2();
void furtherProcessing(MainWindow* mainWindow, QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT);
void saving(const string& outputDirectory);

#endif // IMAGEPROCESSING_H
*/