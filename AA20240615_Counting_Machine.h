/*

#ifndef AA20240615_COUNTING_MACHINE_H
#define AA20240615_COUNTING_MACHINE_H

#include <QMainWindow>
#include <QTextEdit>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QThread>
#include <QDir>
#include <QFileDialog>
#include <QMessageBox>
#include <QScrollBar>
#include <opencv2/opencv.hpp>
#include <queue>
#include <mutex>
#include <condition_variable>
#include "MvCameraControl.h"

#include "AA20240615_Counting_Machine.h"
#include "ui_AA20240615_Counting_Machine.h" // Include the generated UI header

#include "Buffer1Worker.h"
#include "Buffer2Worker.h"
#include "Buffer3Worker.h"
#include "MainProcessingWorker.h"

class AA20240615_Counting_Machine : public QMainWindow{
    Q_OBJECT

public:
    AA20240615_Counting_Machine(QWidget* parent = nullptr);
    ~AA20240615_Counting_Machine();
    void updateDebugLabel(const QString& message); // Make sure this is public


private:
    Ui::AA20240615_Counting_MachineClass ui;

    static int fps;
    static int desired_frames;
    QString outputDirectory;
    QStringList imageFiles;
    int currentImageIndex;
    bool stopProcessing;
    int counter = 0;
    QTextEdit* DEBUG;

    std::queue<cv::Mat> buffer1;
    std::queue<cv::Mat> buffer2;
    std::queue<cv::Mat> buffer3;
    std::mutex buffer1_mutex;
    std::mutex buffer2_mutex;
    std::mutex buffer3_mutex;
    std::condition_variable buffer1_cv;
    std::condition_variable buffer2_cv;
    std::condition_variable buffer3_cv;

    QThread buffer1Thread;
    QThread buffer2Thread;
    QThread buffer3Thread;
    QThread mainProcessingThread;

    Buffer1Worker* buffer1Worker;
    Buffer2Worker* buffer2Worker;
    Buffer3Worker* buffer3Worker;
    MainProcessingWorker* mainProcessingWorker;

    void selectDirectory();
    void detectContours();
    void displayImages();
    void showNextImage();
    void showPreviousImage();
    void runProcessing();
    

    void startWorkers();
    void stopWorkers();

    QString stringToQString(const std::string& str);

private slots:
    // Assuming buffer1Worker is declared as Buffer1Worker* buffer1Worker; in your class

   
   // void on_btnApplySettings_clicked();
};

#endif // AA20240615_COUNTING_MACHINE_H
*/