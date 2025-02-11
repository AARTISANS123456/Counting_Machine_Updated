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
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QDir>
#include <QPixmap>
#include <QTimer>
#include <QFileDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <string>
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , enqueueThread(nullptr)
    , processThread1(nullptr)
    , processThread2(nullptr)
    , processingThread(nullptr)
    , stopThreads(false)
    , currentImageIndex(0)
    , serialPort(new QSerialPort(this))
    , arduinoVendorID("1A86")
    , arduinoProductID("7523")
{
    ui->setupUi(this);

    // Connect buttons to their respective slots
    connect(ui->Detect, &QPushButton::clicked, this, &MainWindow::startProcessing);
    connect(ui->Stop, &QPushButton::clicked, this, &MainWindow::stopProcessing_1);
    connect(ui->Directory, &QPushButton::clicked, this, &MainWindow::selectOutputFolder);
    connect(ui->Display_Output_Images, &QPushButton::clicked, this, &MainWindow::displayAndNavigateImages);
    connect(ui->NEXT, &QPushButton::clicked, this, &MainWindow::showNextImage);
    connect(ui->PREVIOUS, &QPushButton::clicked, this, &MainWindow::showPreviousImage);

    // Serial communication connections
    connect(serialPort, SIGNAL(errorOccurred(QSerialPort::SerialPortError)), this, SLOT(handleSerialError(QSerialPort::SerialPortError)));

    // Disable navigation buttons initially
    ui->NEXT->setEnabled(false);
    ui->PREVIOUS->setEnabled(false);

    // Initialize and select serial port based on vendor ID
    updateSerialPortInfo();
}

MainWindow::~MainWindow()
{
    closeSerialPort(); // Close serial port on destruction
    delete ui;
}


void MainWindow::startProcessing()
{
    int speedValue = ui->spinBox->value(); // Get speed value from spin box

    // Format the speed command for Arduino
    QString speedCommand = QString("$v%1").arg(speedValue, 2, 16, QChar('0'));

    // Send speed command to Arduino
    sendCommandToArduino(speedCommand.toUtf8());
    sendCommandToArduino("$h"); // Send command to start operation
    stopThreads = false; // Reset stop flag

    // Create threads (example placeholders)
    enqueueThread = QThread::create([this] {enqueueBuffer1(ui->FPS, ui->FRAME_COUNT); });
    processThread1 = QThread::create([this] {processBuffer1();  });
    processThread2 = QThread::create([this] {processBuffer2();  });

    enqueueThread->start();
    processThread1->start();
    processThread2->start();

    // Start further processing (example placeholder)
    processingThread = QThread::create([this] {
        furtherProcessing(outputDirectory.toStdString(), ui->countEdit, ui->Status);
        });

    processingThread->start();

    // Disable Display_Output_Images button during processing
    // ui->Display_Output_Images->setEnabled(false);
}

void MainWindow::stopProcessing_1()
{
    sendCommandToArduino("$s"); // Send command to stop operation
    stopThreads = true; // Set stop flag to true
    stopProcessing = false;

    // Stop threads without blocking the main thread
    if (enqueueThread) {
        enqueueThread->quit();
    }
    if (processThread1) {
        processThread1->quit();
    }
    if (processThread2) {
        processThread2->quit();
    }
    if (processingThread) {
        processingThread->quit();
    }

    // The handleThreadFinished slot will handle the GUI update once all threads have finished
}


void MainWindow::selectOutputFolder()
{
    outputDirectory = QFileDialog::getExistingDirectory(this, "Select Output Folder", QDir::homePath());
    if (!outputDirectory.isEmpty()) {
        ui->Status->setText("Status: Folder Selected - " + outputDirectory);
    }
}

void MainWindow::displayAndNavigateImages()
{
    // Stop processing before displaying images
    stopProcessing_1();
    sendCommandToArduino("$s"); // Send command to stop operation
    // Simulate loading and navigating through images
    QTimer::singleShot(1000, this, [this]() {
        ui->Status->setText("Status: Displaying Images");
        // Enable navigation buttons
        ui->PREVIOUS->setEnabled(true);
        ui->NEXT->setEnabled(true);

        // Load and display images (example: load from a directory)
        QDir imageDir(outputDirectory);
        QStringList filters;
        filters << "*.jpg" << "*.png"; // Adjust filters as needed

        imageFiles = imageDir.entryList(filters, QDir::Files, QDir::Name);

        if (!imageFiles.isEmpty()) {
            currentImageIndex = 0; // Start with the first image
            showImage(imageFiles[currentImageIndex]);
        }
        else {
            // Handle case where no images are found
            ui->Status->setText("Status: No images found in selected directory");
        }
        });
}

void MainWindow::showPreviousImage()
{
    if (currentImageIndex > 0 && currentImageIndex < imageFiles.size()) {
        currentImageIndex--;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showNextImage()
{
    if (currentImageIndex >= 0 && currentImageIndex < imageFiles.size() - 1) {
        currentImageIndex++;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showImage(const QString& fileName)
{
    QString imagePath = outputDirectory + "/" + fileName;
    QPixmap pixmap(imagePath);
    if (!pixmap.isNull()) {
        ui->Image->setPixmap(pixmap.scaled(ui->Image->size(), Qt::KeepAspectRatio));
    }
    else {
        ui->Status->setText("Status: Failed to load image - " + fileName);
    }
}

void MainWindow::handleSerialError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError) {
        qDebug() << "Serial port error:" << serialPort->errorString();
        // Handle error appropriately, e.g., show message to user
        ui->Status->setText("Serial port error: " + serialPort->errorString());
    }
}



void MainWindow::updateSerialPortInfo()
{
    bool arduinoDetected = false;

    foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts()) {
        qDebug() << "Port Name:" << info.portName();
        qDebug() << "Vendor ID:" << info.vendorIdentifier();
        qDebug() << "Product ID:" << info.productIdentifier();

        // Convert vendor and product IDs to strings for comparison
        QString vendorIDString = QString::number(info.vendorIdentifier(), 16).toUpper();
        QString productIDString = QString::number(info.productIdentifier(), 16).toUpper();

        qDebug() << "Vendor ID (String):" << vendorIDString;
        qDebug() << "Product ID (String):" << productIDString;

        if (vendorIDString == arduinoVendorID && productIDString == arduinoProductID) {
            qDebug() << "Arduino found on port:" << info.portName();
            if (openSerialPort(info.portName())) {
                ui->Status->setText("Status: Arduino connected on port " + info.portName());
            }
            else {
                ui->Status->setText("Status: Failed to open serial port " + info.portName());
            }
            arduinoDetected = true;
            break;
        }
    }

    if (!arduinoDetected) {
        ui->Status->setText("Status: Arduino not detected");
    }
}



bool MainWindow::openSerialPort(const QString& portName)
{
    serialPort->setPortName(portName);
    serialPort->setBaudRate(QSerialPort::Baud115200); // Set your baud rate
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    return serialPort->open(QIODevice::ReadWrite);
}

void MainWindow::closeSerialPort()
{
    if (serialPort->isOpen()) {
        serialPort->close();
    }
}

void MainWindow::sendCommandToArduino(const QByteArray& command)
{
    std::string commandString = command.constData();  // Convert QByteArray to std::string

    if (serialPort->isOpen() && serialPort->isWritable()) {
        serialPort->write(command);
        serialPort->flush();
        qDebug() << "Sent to Arduino:" << QString::fromStdString(commandString);
    }
    else {
        qDebug() << "Serial port is not open or not writable";
    }
}
*/
////////////////////////////////////////////////////////////////////////////////
//____________________REV 2: CONVEYER SPEED CONTROL AND DISPLAYING THE OUTPUT IMAGES INORDER_______________//
/*
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QDir>
#include <QPixmap>
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <string>
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , enqueueThread(nullptr)
    , processThread1(nullptr)
    , processThread2(nullptr)
    , processingThread(nullptr)
    , stopThreads(false)
    , currentImageIndex(0)
    , serialPort(new QSerialPort(this))
    , arduinoVendorID("1A86")
    , arduinoProductID("7523")
    
{
    ui->setupUi(this);
    // Initialize and select serial port based on vendor ID
    updateSerialPortInfo();
    // Connect buttons to their respective slots
    connect(ui->Detect, &QPushButton::clicked, this, &MainWindow::startProcessing);
    connect(ui->Stop, &QPushButton::clicked, this, &MainWindow::stopProcessing_1);
    connect(ui->Directory, &QPushButton::clicked, this, &MainWindow::selectOutputFolder);
    connect(ui->Display_Output_Images, &QPushButton::clicked, this, &MainWindow::displayAndNavigateImages);
    connect(ui->NEXT, &QPushButton::clicked, this, &MainWindow::showNextImage);
    connect(ui->PREVIOUS, &QPushButton::clicked, this, &MainWindow::showPreviousImage);
    connect(ui->Reset, &QPushButton::clicked, this, &MainWindow::resetApplication);

    // Serial communication connections
    connect(serialPort, SIGNAL(errorOccurred(QSerialPort::SerialPortError)), this, SLOT(handleSerialError(QSerialPort::SerialPortError)));
    connect(ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(on_spinBox_valueChanged(int)));

    // Disable navigation buttons initially
    ui->NEXT->setEnabled(false);
    ui->PREVIOUS->setEnabled(false);
  
   
}

MainWindow::~MainWindow()
{
    closeSerialPort(); // Close serial port on destruction
    delete ui;
}

void MainWindow::startProcessing()
{
    
    QString startCommand = "$h"; // Convert "$h" to QString
    sendCommandToArduino(startCommand); // Send command to start operation
    // Get the speed value from the spin box
    int speedValue = ui->spinBox->value();
    // Convert speed value to a hex string and format it as $vX
    QString speedValueString;
    if (speedValue > 0 && speedValue < 10) {
        speedValueString = QString::number(speedValue);
    }
    else if (speedValue >= 10 && speedValue <= 15) {
        speedValueString = QString(QChar('A' + (speedValue - 10))); // Convert 10-15 to A-F
    }
    else {
        // Handle out of range values if necessary
        return; // or some error handling
    }

    // Construct the speed command in the format $vX
    QString speedCommand = QString("$v%1").arg(speedValueString);

    // Send the formatted command to Arduino
    sendCommandToArduino(speedCommand);


    // Reset stop flag
    stopThreads = false;
    stopProcessing = false;
    // Create and start threads
    enqueueThread = QThread::create([this] { enqueueBuffer1(ui->FPS, ui->FRAME_COUNT); });
    processThread1 = QThread::create([this] { processBuffer1(); });
    processThread2 = QThread::create([this] { processBuffer2(); });
    processingThread = QThread::create([this] {furtherProcessing(ui->countEdit, ui->Status); });
    enqueueThread->start();
    processThread1->start();
    processThread2->start();
    processingThread->start();
    
    processThread3 = QThread::create([this] { saving(outputDirectory.toStdString()); });
    processThread3->start();
    // Disable Display_Output_Images button during processing
    ui->Detect->setEnabled(false);
}

void MainWindow::resetApplication()
{
    // Stop processing if it's ongoing
    stopProcessing_1(); // This function should signal threads to stop and wait for them to finish

    // Wait for threads to finish
    if (enqueueThread && enqueueThread->isRunning()) {
        enqueueThread->quit();
        if (!enqueueThread->wait(2000)) { // Wait for thread to finish with timeout
            qDebug() << "enqueueThread did not finish in time";
        }
        delete enqueueThread;
        enqueueThread = nullptr;
    }

    if (processThread1 && processThread1->isRunning()) {
        processThread1->quit();
        if (!processThread1->wait(2000)) { // Wait for thread to finish with timeout
            qDebug() << "processThread1 did not finish in time";
        }
        delete processThread1;
        processThread1 = nullptr;
    }

    if (processThread2 && processThread2->isRunning()) {
        processThread2->quit();
        if (!processThread2->wait(2000)) { // Wait for thread to finish with timeout
            qDebug() << "processThread2 did not finish in time";
        }
        delete processThread2;
        processThread2 = nullptr;
    }

    if (processingThread && processingThread->isRunning()) {
        processingThread->quit();
        if (!processingThread->wait(2000)) { // Wait for thread to finish with timeout
            qDebug() << "processingThread did not finish in time";
        }
        delete processingThread;
        processingThread = nullptr;
    }

    // Close serial port
    closeSerialPort();

    // Clear any other resources or state
    outputDirectory.clear();
    imageFiles.clear();

    // Re-enable necessary buttons or reset any other state as needed
    ui->Detect->setEnabled(true);
}

void MainWindow::closeSerialPort()
{
    if (serialPort->isOpen()) {
        serialPort->close();
    }
}

void MainWindow::stopProcessing_1()
{
    QString stopCommand = "$s"; // Convert "$s" to QString
    sendCommandToArduino(stopCommand); // Send stop command to Arduino

    stopThreads = true; // Set stop flag to true
    stopProcessing = true;

    // Stop threads without blocking the main thread
    if (enqueueThread) {
        enqueueThread->quit();
    }
    if (processThread1) {
        processThread1->quit();
    }
    if (processThread2) {
        processThread2->quit();
    }
    if (processingThread) {
        processingThread->quit();
    }
}


void MainWindow::selectOutputFolder()
{
    outputDirectory = QFileDialog::getExistingDirectory(this, "Select Output Folder", QDir::homePath());
    if (!outputDirectory.isEmpty()) {
        ui->Status->setText("Status: Folder Selected - " + outputDirectory);
    }
}

void MainWindow::displayAndNavigateImages()
{
    // Stop processing before displaying images
    stopProcessing_1();
    QString stopCommand = "$s"; // Convert "$h" to QString
    sendCommandToArduino(stopCommand); // Send command to start operation

    // Simulate loading and navigating through images
    QTimer::singleShot(1000, this, [this]() {
        ui->Status->setText("Status: Displaying Images");
        // Enable navigation buttons
        ui->PREVIOUS->setEnabled(true);
        ui->NEXT->setEnabled(true);

        // Load and display images (example: load from a directory)
        QDir imageDir(outputDirectory);
        QStringList filters;
        filters << "*.jpg" << "*.png"; // Adjust filters as needed

        imageFiles = imageDir.entryList(filters, QDir::Files, QDir::Name);

        if (!imageFiles.isEmpty()) {
            // Sort the imageFiles based on their names
            std::sort(imageFiles.begin(), imageFiles.end(), [](const QString& file1, const QString& file2) {
                // Extract the numeric part from filenames (e.g., "image_1.jpg" -> "1")
                QString number1 = file1.split("_").last().split(".").first();
                QString number2 = file2.split("_").last().split(".").first();
                return number1.toInt() < number2.toInt();
                });

            currentImageIndex = 0; // Start with the first image
            showImage(imageFiles[currentImageIndex]);
        }
        else {
            // Handle case where no images are found
            ui->Status->setText("Status: No images found in selected directory");
        }
        });
}

void MainWindow::showPreviousImage()
{
    if (currentImageIndex > 0 && currentImageIndex < imageFiles.size()) {
        currentImageIndex--;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showNextImage()
{
    if (currentImageIndex >= 0 && currentImageIndex < imageFiles.size() - 1) {
        currentImageIndex++;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showImage(const QString& fileName)
{
    QString imagePath = outputDirectory + "/" + fileName;
    QPixmap pixmap(imagePath);
    if (!pixmap.isNull()) {
        ui->Image->setPixmap(pixmap.scaled(ui->Image->size(), Qt::KeepAspectRatio));
    }
    else {
        ui->Status->setText("Status: Failed to load image - " + fileName);
    }
}

void MainWindow::handleSerialError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError) {
        qDebug() << "Serial port error:" << serialPort->errorString();
        // Handle error appropriately, e.g., show message to user
        ui->Status->setText("Serial port error: " + serialPort->errorString());
    }
}



void MainWindow::updateSerialPortInfo()
{
    bool arduinoDetected = false;

    foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts()) {
        qDebug() << "Port Name:" << info.portName();
        qDebug() << "Vendor ID:" << info.vendorIdentifier();
        qDebug() << "Product ID:" << info.productIdentifier();

        // Convert vendor and product IDs to strings for comparison
        QString vendorIDString = QString::number(info.vendorIdentifier(), 16).toUpper();
        QString productIDString = QString::number(info.productIdentifier(), 16).toUpper();

        qDebug() << "Vendor ID (String):" << vendorIDString;
        qDebug() << "Product ID (String):" << productIDString;

        if (vendorIDString == arduinoVendorID && productIDString == arduinoProductID) {
            qDebug() << "Arduino found on port:" << info.portName();
            if (openSerialPort(info.portName())) {
                ui->Status->setText("Status: Arduino connected on port " + info.portName());
            }
            else 
            {
                ui->Status->setText("Status: Failed to open serial port " + info.portName());
            }
            arduinoDetected = true;
            break;
        }
    }

    if (!arduinoDetected) {
        ui->Status->setText("Status: Arduino not detected");
    }
}



bool MainWindow::openSerialPort(const QString& portName)
{
    serialPort->setPortName(portName);
    serialPort->setBaudRate(QSerialPort::Baud115200); // Set your baud rate
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    return serialPort->open(QIODevice::ReadWrite);
}



QString MainWindow::convertSpeedValueToCommand(int speedValue) {
    // Assuming speedValue is already a hexadecimal string (e.g., "1" to "F")
    return QString("$v%1").arg(speedValue);
}


// Slot to handle spin box value change
void MainWindow::on_spinBox_valueChanged(int speedValue) {
    QString command = convertSpeedValueToCommand(speedValue);
    if (!command.isEmpty()) {
        sendCommandToArduino(command);
    }
}

void MainWindow::sendCommandToArduino(const QString& command) {
    QByteArray commandBytes = command.toUtf8(); // Convert QString to QByteArray

    if (serialPort->isOpen() && serialPort->isWritable()) {
        serialPort->write(commandBytes);
        serialPort->flush();
        qDebug() << "Sent to Arduino:" << command; // Debug output
    }
    else {
        qDebug() << "Serial port is not open or not writable";
    }
}
*/
////////////////////////////////////////////////////////////////////////////////
//____________________REV 3: EXACT CONTOUR COUNT_______________
/*
* * REV 3: EXACT COUNT OF CONTOURS GETTING DISPLAYED
* * Following works:
*   1.You get the exact count of the contours displayed, but the problem is we have we minimize the UI again for ther UI to refresh.
*   2.The stopConveyer gets executed properly but there is delay and thus letting more contours fall off, as the UI is not getting refreshed
*/

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QDir>
#include <QPixmap>
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <string>
extern std::atomic<int> totalcounter;
extern int desiredCount;
extern std::atomic<bool> stopProcessing;
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , enqueueThread(nullptr)
    , processThread1(nullptr)
    , processThread2(nullptr)
    , processingThread(nullptr)
    , stopThreads(false)
    , currentImageIndex(0)
    , serialPort(new QSerialPort(this))
    , arduinoVendorID("1A86")
    , arduinoProductID("7523")
{
    ui->setupUi(this);
    updateSerialPortInfo();

    connect(ui->Detect, &QPushButton::clicked, this, &MainWindow::startProcessing);
    connect(ui->Stop, &QPushButton::clicked, this, &MainWindow::stopProcessing_1);
    connect(ui->Directory, &QPushButton::clicked, this, &MainWindow::selectOutputFolder);
    connect(ui->Display_Output_Images, &QPushButton::clicked, this, &MainWindow::displayAndNavigateImages);
    connect(ui->NEXT, &QPushButton::clicked, this, &MainWindow::showNextImage);
    connect(ui->PREVIOUS, &QPushButton::clicked, this, &MainWindow::showPreviousImage);
    connect(ui->Reset, &QPushButton::clicked, this, &MainWindow::resetApplication);

    connect(serialPort, SIGNAL(errorOccurred(QSerialPort::SerialPortError)), this, SLOT(handleSerialError(QSerialPort::SerialPortError)));
    connect(ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(on_spinBox_valueChanged(int)));

    ui->NEXT->setEnabled(false);
    ui->PREVIOUS->setEnabled(false);

    
}

MainWindow::~MainWindow()
{
    closeSerialPort();
    delete ui;
}

void MainWindow::startProcessing()
{
    QString startCommand = "$h";
    sendCommandToArduino(startCommand);

    int speedValue = ui->spinBox->value();
    QString speedValueString;
    if (speedValue > 0 && speedValue < 10) {
        speedValueString = QString::number(speedValue);
    }
    else if (speedValue >= 10 && speedValue <= 15) {
        speedValueString = QString(QChar('A' + (speedValue - 10)));
    }
    else {
        return;
    }

    QString speedCommand = QString("$v%1").arg(speedValueString);
    sendCommandToArduino(speedCommand);

    stopThreads = false;
    stopProcessing = false;

    // Get desired count from ui->Frames
    bool ok;
    int framesValue = ui->Frames->text().toInt(&ok);
    if (ok) {
        desiredCount = framesValue;
    }
    else {
        qDebug() << "Invalid number in ui->Frames";
        return;
    }

    enqueueThread = QThread::create([this] { enqueueBuffer1(ui->FPS); });
    processThread1 = QThread::create([this] { processBuffer1(); });
    processThread2 = QThread::create([this] { processBuffer2(); });
    processingThread = QThread::create([this] { furtherProcessing(this,ui->countEdit, ui->Status, ui->Frames); });
    enqueueThread->start();
    processThread1->start();
    processThread2->start();
    processingThread->start();

    processThread3 = QThread::create([this] { saving(outputDirectory.toStdString()); });
    processThread3->start();
    ui->Detect->setEnabled(false);
}



void MainWindow::resetApplication()
{
    // Stop processing if it's ongoing
    stopProcessing_1(); // This function should signal threads to stop and wait for them to finish

    // Wait for threads to finish
    if (enqueueThread && enqueueThread->isRunning()) {
        enqueueThread->quit();
        if (!enqueueThread->wait(2000)) { // Wait for thread to finish with timeout
            qDebug() << "enqueueThread did not finish in time";
        }
        delete enqueueThread;
        enqueueThread = nullptr;
    }

    if (processThread1 && processThread1->isRunning()) {
        processThread1->quit();
        if (!processThread1->wait(2000)) { // Wait for thread to finish with timeout
            qDebug() << "processThread1 did not finish in time";
        }
        delete processThread1;
        processThread1 = nullptr;
    }

    if (processThread2 && processThread2->isRunning()) {
        processThread2->quit();
        if (!processThread2->wait(2000)) { // Wait for thread to finish with timeout
            qDebug() << "processThread2 did not finish in time";
        }
        delete processThread2;
        processThread2 = nullptr;
    }

    if (processingThread && processingThread->isRunning()) {
        processingThread->quit();
        if (!processingThread->wait(2000)) { // Wait for thread to finish with timeout
            qDebug() << "processingThread did not finish in time";
        }
        delete processingThread;
        processingThread = nullptr;
    }

    // Close serial port
    closeSerialPort();

    // Clear any other resources or state
    outputDirectory.clear();
    imageFiles.clear();

    // Re-enable necessary buttons or reset any other state as needed
    ui->Detect->setEnabled(true);
}

void MainWindow::closeSerialPort()
{
    if (serialPort->isOpen()) {
        serialPort->close();
    }
}

void MainWindow::stopProcessing_1()
{
    QString stopCommand = "$s"; // Convert "$s" to QString
    sendCommandToArduino(stopCommand); // Send stop command to Arduino

    stopThreads = true; // Set stop flag to true
    stopProcessing = true;

    // Stop threads without blocking the main thread
    if (enqueueThread) {
        enqueueThread->quit();
    }
    if (processThread1) {
        processThread1->quit();
    }
    if (processThread2) {
        processThread2->quit();
    }
    if (processingThread) {
        processingThread->quit();
    }
}
void MainWindow::stopConveyor()
{
    QString stopCommand = "$s"; // Assuming $s is the stop command for your conveyor
    sendCommandToArduino(stopCommand);
}

void MainWindow::selectOutputFolder()
{
    outputDirectory = QFileDialog::getExistingDirectory(this, "Select Output Folder", QDir::homePath());
    if (!outputDirectory.isEmpty()) {
        ui->Status->setText("Status: Folder Selected - " + outputDirectory);
    }
}

void MainWindow::displayAndNavigateImages()
{
    // Stop processing before displaying images
    stopProcessing_1();
    QString stopCommand = "$s"; // Convert "$h" to QString
    sendCommandToArduino(stopCommand); // Send command to start operation

    // Simulate loading and navigating through images
    QTimer::singleShot(1000, this, [this]() {
        ui->Status->setText("Status: Displaying Images");
        // Enable navigation buttons
        ui->PREVIOUS->setEnabled(true);
        ui->NEXT->setEnabled(true);

        // Load and display images (example: load from a directory)
        QDir imageDir(outputDirectory);
        QStringList filters;
        filters << "*.jpg" << "*.png"; // Adjust filters as needed

        imageFiles = imageDir.entryList(filters, QDir::Files, QDir::Name);

        if (!imageFiles.isEmpty()) {
            // Sort the imageFiles based on their names
            std::sort(imageFiles.begin(), imageFiles.end(), [](const QString& file1, const QString& file2) {
                // Extract the numeric part from filenames (e.g., "image_1.jpg" -> "1")
                QString number1 = file1.split("_").last().split(".").first();
                QString number2 = file2.split("_").last().split(".").first();
                return number1.toInt() < number2.toInt();
                });

            currentImageIndex = 0; // Start with the first image
            showImage(imageFiles[currentImageIndex]);
        }
        else {
            // Handle case where no images are found
            ui->Status->setText("Status: No images found in selected directory");
        }
        });
}

void MainWindow::showPreviousImage()
{
    if (currentImageIndex > 0 && currentImageIndex < imageFiles.size()) {
        currentImageIndex--;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showNextImage()
{
    if (currentImageIndex >= 0 && currentImageIndex < imageFiles.size() - 1) {
        currentImageIndex++;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showImage(const QString& fileName)
{
    QString imagePath = outputDirectory + "/" + fileName;
    QPixmap pixmap(imagePath);
    if (!pixmap.isNull()) {
        ui->Image->setPixmap(pixmap.scaled(ui->Image->size(), Qt::KeepAspectRatio));
    }
    else {
        ui->Status->setText("Status: Failed to load image - " + fileName);
    }
}

void MainWindow::handleSerialError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError) {
        qDebug() << "Serial port error:" << serialPort->errorString();
        // Handle error appropriately, e.g., show message to user
        ui->Status->setText("Serial port error: " + serialPort->errorString());
    }
}



void MainWindow::updateSerialPortInfo()
{
    bool arduinoDetected = false;

    foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts()) {
        qDebug() << "Port Name:" << info.portName();
        qDebug() << "Vendor ID:" << info.vendorIdentifier();
        qDebug() << "Product ID:" << info.productIdentifier();

        // Convert vendor and product IDs to strings for comparison
        QString vendorIDString = QString::number(info.vendorIdentifier(), 16).toUpper();
        QString productIDString = QString::number(info.productIdentifier(), 16).toUpper();

        qDebug() << "Vendor ID (String):" << vendorIDString;
        qDebug() << "Product ID (String):" << productIDString;

        if (vendorIDString == arduinoVendorID && productIDString == arduinoProductID) {
            qDebug() << "Arduino found on port:" << info.portName();
            if (openSerialPort(info.portName())) {
                ui->Status->setText("Status: Arduino connected on port " + info.portName());
            }
            else
            {
                ui->Status->setText("Status: Failed to open serial port " + info.portName());
            }
            arduinoDetected = true;
            break;
        }
    }

    if (!arduinoDetected) {
        ui->Status->setText("Status: Arduino not detected");
    }
}



bool MainWindow::openSerialPort(const QString& portName)
{
    serialPort->setPortName(portName);
    serialPort->setBaudRate(QSerialPort::Baud115200); // Set your baud rate
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    return serialPort->open(QIODevice::ReadWrite);
}



QString MainWindow::convertSpeedValueToCommand(int speedValue) {
    // Assuming speedValue is already a hexadecimal string (e.g., "1" to "F")
    return QString("$v%1").arg(speedValue);
}


// Slot to handle spin box value change
void MainWindow::on_spinBox_valueChanged(int speedValue) {
    QString command = convertSpeedValueToCommand(speedValue);
    if (!command.isEmpty()) {
        sendCommandToArduino(command);
    }
}

void MainWindow::sendCommandToArduino(const QString& command) {
    QByteArray commandBytes = command.toUtf8(); // Convert QString to QByteArray

    if (serialPort->isOpen() && serialPort->isWritable()) {
        serialPort->write(commandBytes);
        serialPort->flush();
        qDebug() << "Sent to Arduino:" << command; // Debug output
    }
    else {
        qDebug() << "Serial port is not open or not writable";
    }
}

////////////////////////////////////////////////////////////////////////////////////////
//___________________REV 4: STOPPING CONVEYER AT SPECIFIC CONTOUR___________________________________
/*
* * Following works:
*   1.Lookup table implementation works properly but as the UI doesnot refresh consistently , thus many contours fall extra even though they are counted properly.
*   2.updateConveyorSpeed is the function that is been written for the lookup table mechanism.
*   3.The serialport communication functions were shifted to the ImageProcessing.cpp because of the updateConveyorSpeed function initial setup trials.
*/ 
/*
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "imageprocessing.h" // Include the header file for image processing functions
#include <QDir>
#include <QPixmap>
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <string>
#include <QThread>
#include <QDateTime>
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , enqueueThread(nullptr)
    , processThread1(nullptr)
    , processThread2(nullptr)
    , processingThread(nullptr)
    , processThread3(nullptr)
    , stopThreads(false)
    , stopProcessing(false)
    , currentImageIndex(0)
{
    ui->setupUi(this);
    updateSerialPortInfo(ui->Status);

    connect(ui->Detect, &QPushButton::clicked, this, &MainWindow::startProcessing);
    connect(ui->Stop, &QPushButton::clicked, this, &MainWindow::stopProcessing_1);
    connect(ui->Directory, &QPushButton::clicked, this, &MainWindow::selectOutputFolder);
    connect(ui->Display_Output_Images, &QPushButton::clicked, this, &MainWindow::displayAndNavigateImages);
    connect(ui->NEXT, &QPushButton::clicked, this, &MainWindow::showNextImage);
    connect(ui->PREVIOUS, &QPushButton::clicked, this, &MainWindow::showPreviousImage);
    connect(ui->Reset, &QPushButton::clicked, this, &MainWindow::resetApplication);
    connect(ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(on_spinBox_valueChanged(int)));
   

    ui->NEXT->setEnabled(false);
    ui->PREVIOUS->setEnabled(false);
}

MainWindow::~MainWindow() {
    closeSerialPort();
    delete ui;
}


void MainWindow::startProcessing()
{
    QString startCommand = "$h";
    sendCommandToArduino(startCommand);

    int speedValue = ui->spinBox->value();
    QString speedValueString;
    if (speedValue > 0 && speedValue < 10) {
        speedValueString = QString::number(speedValue);
    }
    else if (speedValue >= 10 && speedValue <= 15) {
        speedValueString = QString(QChar('A' + (speedValue - 10)));
    }
    else {
        return;
    }

    QString speedCommand = QString("$v%1").arg(speedValueString);
    sendCommandToArduino(speedCommand);

    stopThreads = false;
    stopProcessing = false;

    enqueueThread = QThread::create([this] { enqueueBuffer1(ui->FPS); });
    processThread1 = QThread::create([this] { processBuffer1(); });
    processThread2 = QThread::create([this] { processBuffer2(); });
    processingThread = QThread::create([this] { furtherProcessing(this, ui->countEdit, ui->Status, ui->Frames,ui->speed_conv); });
    enqueueThread->start();
    processThread1->start();
    processThread2->start();
    processingThread->start();

    processThread3 = QThread::create([this] { saving(outputDirectory.toStdString()); });
    processThread3->start();
    ui->Detect->setEnabled(false);
}



void MainWindow::resetApplication() {
    stopProcessing_1();

    if (enqueueThread && enqueueThread->isRunning()) {
        enqueueThread->quit();
        if (!enqueueThread->wait(2000)) {
            qDebug() << "enqueueThread did not finish in time";
        }
        delete enqueueThread;
        enqueueThread = nullptr;
    }

    if (processThread1 && processThread1->isRunning()) {
        processThread1->quit();
        if (!processThread1->wait(2000)) {
            qDebug() << "processThread1 did not finish in time";
        }
        delete processThread1;
        processThread1 = nullptr;
    }

    if (processThread2 && processThread2->isRunning()) {
        processThread2->quit();
        if (!processThread2->wait(2000)) {
            qDebug() << "processThread2 did not finish in time";
        }
        delete processThread2;
        processThread2 = nullptr;
    }

    if (processingThread && processingThread->isRunning()) {
        processingThread->quit();
        if (!processingThread->wait(2000)) {
            qDebug() << "processingThread did not finish in time";
        }
        delete processingThread;
        processingThread = nullptr;
    }

    closeSerialPort();
    outputDirectory.clear();
    imageFiles.clear();
    ui->Detect->setEnabled(true);
}

void MainWindow::stopProcessing_1() {
    QString stopCommand = "$s";
    sendCommandToArduino(stopCommand);

    stopThreads = true;
    stopProcessing = true;

    // Wait for threads to finish before deleting
    if (enqueueThread) {
        enqueueThread->quit();
        enqueueThread->wait();
        delete enqueueThread;
        enqueueThread = nullptr;
    }
    if (processThread1) {
        processThread1->quit();
        processThread1->wait();
        delete processThread1;
        processThread1 = nullptr;
    }
    if (processThread2) {
        processThread2->quit();
        processThread2->wait();
        delete processThread2;
        processThread2 = nullptr;
    }
    if (processingThread) {
        processingThread->quit();
        processingThread->wait();
        delete processingThread;
        processingThread = nullptr;
    }
}


void MainWindow::selectOutputFolder() {
    outputDirectory = QFileDialog::getExistingDirectory(this, "Select Output Folder", QDir::homePath());
    if (!outputDirectory.isEmpty()) {
        ui->Status->setText("Status: Folder Selected - " + outputDirectory);
    }
}

void MainWindow::displayAndNavigateImages() {
    stopProcessing_1();
    QString stopCommand = "$s";
    sendCommandToArduino(stopCommand);

    QTimer::singleShot(1000, this, [this]() {
        ui->Status->setText("Status: Displaying Images");
        ui->PREVIOUS->setEnabled(true);
        ui->NEXT->setEnabled(true);

        QDir imageDir(outputDirectory);
        QStringList filters;
        filters << "*.jpg" << "*.png";
        imageFiles = imageDir.entryList(filters, QDir::Files, QDir::Name);

        if (!imageFiles.isEmpty()) {
            std::sort(imageFiles.begin(), imageFiles.end(), [](const QString& file1, const QString& file2) {
                QString number1 = file1.split("_").last().split(".").first();
                QString number2 = file2.split("_").last().split(".").first();
                return number1.toInt() < number2.toInt();
                });

            currentImageIndex = 0;
            QString imagePath = outputDirectory + "/" + imageFiles[currentImageIndex];
            QPixmap image(imagePath);
            ui->Image->setPixmap(image.scaled(ui->Image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
        });
}

void MainWindow::showNextImage() {
    if (currentImageIndex < imageFiles.size() - 1) {
        ++currentImageIndex;
        QString imagePath = outputDirectory + "/" + imageFiles[currentImageIndex];
        QPixmap image(imagePath);
        ui->Image->setPixmap(image.scaled(ui->Image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void MainWindow::showPreviousImage() {
    if (currentImageIndex > 0) {
        --currentImageIndex;
        QString imagePath = outputDirectory + "/" + imageFiles[currentImageIndex];
        QPixmap image(imagePath);
        ui->Image->setPixmap(image.scaled(ui->Image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void MainWindow::on_spinBox_valueChanged(int speedValue) {
    QString speedCommand = convertSpeedValueToCommand(speedValue);
    if (!speedCommand.isEmpty()) {
        sendCommandToArduino(speedCommand);
    }
}
*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "imageprocessing.h" // Include the header file for image processing functions
#include <QDir>
#include <QPixmap>
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <string>
#include <QThread>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , enqueueThread(nullptr)
    , processThread1(nullptr)
    , processThread2(nullptr)
    , processingThread(nullptr)
    , stopThreads(false)
    , stopProcessing(false)
    , currentImageIndex(0)
{
    ui->setupUi(this);
    updateSerialPortInfo();

    connect(ui->Detect, &QPushButton::clicked, this, &MainWindow::startProcessing);
    connect(ui->Stop, &QPushButton::clicked, this, &MainWindow::stopProcessing_1);
    connect(ui->Directory, &QPushButton::clicked, this, &MainWindow::selectOutputFolder);
    connect(ui->Display_Output_Images, &QPushButton::clicked, this, &MainWindow::displayAndNavigateImages);
    connect(ui->NEXT, &QPushButton::clicked, this, &MainWindow::showNextImage);
    connect(ui->PREVIOUS, &QPushButton::clicked, this, &MainWindow::showPreviousImage);
    connect(ui->Reset, &QPushButton::clicked, this, &MainWindow::resetApplication);
    connect(ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(on_spinBox_valueChanged(int)));

    ui->NEXT->setEnabled(false);
    ui->PREVIOUS->setEnabled(false);
}

MainWindow::~MainWindow() {
    closeSerialPort();
    delete ui;
}


void MainWindow::startProcessing()
{
    
   
    int speedValue = ui->spinBox->value();
    QString speedValueString;
    if (speedValue > 0 && speedValue < 10) {
        speedValueString = QString::number(speedValue);
    }
    else if (speedValue >= 10 && speedValue <= 15) {
        speedValueString = QString(QChar('A' + (speedValue - 10)));
    }
    else {
        return;
    }
  
  

    stopThreads = false;
    stopProcessing = false;

    enqueueThread = QThread::create([this] { enqueueBuffer1(ui->FPS); });
    processThread1 = QThread::create([this] { processBuffer1(); });
    processThread2 = QThread::create([this] { processBuffer2(); });
    processingThread = QThread::create([this] { furtherProcessing( ui->countEdit, ui->Status, ui->Frames,ui->speed_conv); });
    enqueueThread->start();
    processThread1->start();
    processThread2->start();
    processingThread->start();
    
    processThread3 = QThread::create([this] { saving(outputDirectory.toStdString()); });
    processThread3->start();
    
    ui->Detect->setEnabled(false);
}



void MainWindow::resetApplication() {
    stopProcessing_1();

    if (enqueueThread && enqueueThread->isRunning()) {
        enqueueThread->quit();
        if (!enqueueThread->wait(2000)) {
            qDebug() << "enqueueThread did not finish in time";
        }
        delete enqueueThread;
        enqueueThread = nullptr;
    }

    if (processThread1 && processThread1->isRunning()) {
        processThread1->quit();
        if (!processThread1->wait(2000)) {
            qDebug() << "processThread1 did not finish in time";
        }
        delete processThread1;
        processThread1 = nullptr;
    }

    if (processThread2 && processThread2->isRunning()) {
        processThread2->quit();
        if (!processThread2->wait(2000)) {
            qDebug() << "processThread2 did not finish in time";
        }
        delete processThread2;
        processThread2 = nullptr;
    }


    if (processingThread && processingThread->isRunning()) {
        processingThread->quit();
        if (!processingThread->wait(2000)) {
            qDebug() << "processingThread did not finish in time";
        }
        delete processingThread;
        processingThread = nullptr;
    }

    closeSerialPort();
    outputDirectory.clear();
    imageFiles.clear();
    ui->Detect->setEnabled(true);
}

void MainWindow::stopProcessing_1() {
  
    stopThreads = true;
    stopProcessing = true;

    if (enqueueThread) {
        enqueueThread->quit();
    }
    if (processThread1) {
        processThread1->quit();
    }
    if (processThread2) {
        processThread2->quit();
    }

    if (processingThread) {
        processingThread->quit();
    }
 
}

void MainWindow::selectOutputFolder() {
    outputDirectory = QFileDialog::getExistingDirectory(this, "Select Output Folder", QDir::homePath());
    if (!outputDirectory.isEmpty()) {
        ui->Status->setText("Status: Folder Selected - " + outputDirectory);
    }
}

void MainWindow::displayAndNavigateImages() {
    stopProcessing_1();
   

    QTimer::singleShot(1000, this, [this]() {
        ui->Status->setText("Status: Displaying Images");
        ui->PREVIOUS->setEnabled(true);
        ui->NEXT->setEnabled(true);

        QDir imageDir(outputDirectory);
        QStringList filters;
        filters << "*.jpg" << "*.png";
        imageFiles = imageDir.entryList(filters, QDir::Files, QDir::Name);

        if (!imageFiles.isEmpty()) {
            std::sort(imageFiles.begin(), imageFiles.end(), [](const QString& file1, const QString& file2) {
                QString number1 = file1.split("_").last().split(".").first();
                QString number2 = file2.split("_").last().split(".").first();
                return number1.toInt() < number2.toInt();
            });

            currentImageIndex = 0;
            QString imagePath = outputDirectory + "/" + imageFiles[currentImageIndex];
            QPixmap image(imagePath);
            ui->Image->setPixmap(image.scaled(ui->Image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
    });
}

void MainWindow::showNextImage() {
    if (currentImageIndex < imageFiles.size() - 1) {
        ++currentImageIndex;
        QString imagePath = outputDirectory + "/" + imageFiles[currentImageIndex];
        QPixmap image(imagePath);
        ui->Image->setPixmap(image.scaled(ui->Image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void MainWindow::showPreviousImage() {
    if (currentImageIndex > 0) {
        --currentImageIndex;
        QString imagePath = outputDirectory + "/" + imageFiles[currentImageIndex];
        QPixmap image(imagePath);
        ui->Image->setPixmap(image.scaled(ui->Image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}
/*
void MainWindow::on_spinBox_valueChanged(int speedValue) {
    QString speedCommand = convertSpeedValueToCommand(speedValue);
    if (!speedCommand.isEmpty()) {
        sendCommandToArduino(speedCommand);
    }
}

*/


//Trial
/*
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QDir>
#include <QPixmap>
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <string>

extern int desiredCount;

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , enqueueThread(nullptr)
    , currentImageIndex(0)
    , serialPort(new QSerialPort(this))
    , arduinoVendorID("1A86")
    , arduinoProductID("7523")
{
    ui->setupUi(this);
    updateSerialPortInfo();

    connect(ui->Detect, &QPushButton::clicked, this, &MainWindow::startProcessing);
    connect(ui->Stop, &QPushButton::clicked, this, &MainWindow::stopProcessing_1);
    connect(ui->Directory, &QPushButton::clicked, this, &MainWindow::selectOutputFolder);
    connect(ui->Display_Output_Images, &QPushButton::clicked, this, &MainWindow::displayAndNavigateImages);
    connect(ui->NEXT, &QPushButton::clicked, this, &MainWindow::showNextImage);
    connect(ui->PREVIOUS, &QPushButton::clicked, this, &MainWindow::showPreviousImage);
    connect(ui->Reset, &QPushButton::clicked, this, &MainWindow::resetApplication);

    connect(serialPort, SIGNAL(errorOccurred(QSerialPort::SerialPortError)), this, SLOT(handleSerialError(QSerialPort::SerialPortError)));
    connect(ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(on_spinBox_valueChanged(int)));

    ui->NEXT->setEnabled(false);
    ui->PREVIOUS->setEnabled(false);


}

MainWindow::~MainWindow()
{
    closeSerialPort();
    delete ui;
}

void MainWindow::startProcessing()
{
    QString startCommand = "$h";
    sendCommandToArduino(startCommand);

    int speedValue = ui->spinBox->value();
    QString speedValueString;
    if (speedValue > 0 && speedValue < 10) {
        speedValueString = QString::number(speedValue);
    }
    else if (speedValue >= 10 && speedValue <= 15) {
        speedValueString = QString(QChar('A' + (speedValue - 10)));
    }
    else {
        return;
    }

    QString speedCommand = QString("$v%1").arg(speedValueString);
    sendCommandToArduino(speedCommand);

   
    stopProcessing = false;

    // Get desired count from ui->Frames
    bool ok;
    int framesValue = ui->Frames->text().toInt(&ok);
    if (ok) {
        desiredCount = framesValue;
    }
    else {
        qDebug() << "Invalid number in ui->Frames";
        return;
    }

    enqueueThread = QThread::create([this] { run_camera( ui->Frames, this, ui->countEdit, ui->Status); });
    
    enqueueThread->start();
 

    processThread3 = QThread::create([this] { saving(outputDirectory.toStdString()); });
    processThread3->start();
    ui->Detect->setEnabled(false);
}



void MainWindow::resetApplication()
{
    // Stop processing if it's ongoing
    stopProcessing_1(); // This function should signal threads to stop and wait for them to finish

    // Wait for threads to finish
    if (enqueueThread && enqueueThread->isRunning()) {
        enqueueThread->quit();
        if (!enqueueThread->wait(2000)) { // Wait for thread to finish with timeout
            qDebug() << "enqueueThread did not finish in time";
        }
        delete enqueueThread;
        enqueueThread = nullptr;
    }

    // Close serial port
    closeSerialPort();

    // Clear any other resources or state
    outputDirectory.clear();
    imageFiles.clear();

    // Re-enable necessary buttons or reset any other state as needed
    ui->Detect->setEnabled(true);
}

void MainWindow::closeSerialPort()
{
    if (serialPort->isOpen()) {
        serialPort->close();
    }
}

void MainWindow::stopProcessing_1()
{
    QString stopCommand = "$s"; // Convert "$s" to QString
    sendCommandToArduino(stopCommand); // Send stop command to Arduino
    stopProcessing = true;

    // Stop threads without blocking the main thread
    if (enqueueThread) {
        enqueueThread->quit();
    }
   
}
void MainWindow::stopConveyor()
{
    QString stopCommand = "$s"; // Assuming $s is the stop command for your conveyor
    sendCommandToArduino(stopCommand);
}

void MainWindow::selectOutputFolder()
{
    outputDirectory = QFileDialog::getExistingDirectory(this, "Select Output Folder", QDir::homePath());
    if (!outputDirectory.isEmpty()) {
        ui->Status->setText("Status: Folder Selected - " + outputDirectory);
    }
}

void MainWindow::displayAndNavigateImages()
{
    // Stop processing before displaying images
    stopProcessing_1();
    QString stopCommand = "$s"; // Convert "$h" to QString
    sendCommandToArduino(stopCommand); // Send command to start operation

    // Simulate loading and navigating through images
    QTimer::singleShot(1000, this, [this]() {
        ui->Status->setText("Status: Displaying Images");
        // Enable navigation buttons
        ui->PREVIOUS->setEnabled(true);
        ui->NEXT->setEnabled(true);

        // Load and display images (example: load from a directory)
        QDir imageDir(outputDirectory);
        QStringList filters;
        filters << "*.jpg" << "*.png"; // Adjust filters as needed

        imageFiles = imageDir.entryList(filters, QDir::Files, QDir::Name);

        if (!imageFiles.isEmpty()) {
            // Sort the imageFiles based on their names
            std::sort(imageFiles.begin(), imageFiles.end(), [](const QString& file1, const QString& file2) {
                // Extract the numeric part from filenames (e.g., "image_1.jpg" -> "1")
                QString number1 = file1.split("_").last().split(".").first();
                QString number2 = file2.split("_").last().split(".").first();
                return number1.toInt() < number2.toInt();
                });

            currentImageIndex = 0; // Start with the first image
            showImage(imageFiles[currentImageIndex]);
        }
        else {
            // Handle case where no images are found
            ui->Status->setText("Status: No images found in selected directory");
        }
        });
}

void MainWindow::showPreviousImage()
{
    if (currentImageIndex > 0 && currentImageIndex < imageFiles.size()) {
        currentImageIndex--;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showNextImage()
{
    if (currentImageIndex >= 0 && currentImageIndex < imageFiles.size() - 1) {
        currentImageIndex++;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showImage(const QString& fileName)
{
    QString imagePath = outputDirectory + "/" + fileName;
    QPixmap pixmap(imagePath);
    if (!pixmap.isNull()) {
        ui->Image->setPixmap(pixmap.scaled(ui->Image->size(), Qt::KeepAspectRatio));
    }
    else {
        ui->Status->setText("Status: Failed to load image - " + fileName);
    }
}

void MainWindow::handleSerialError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError) {
        qDebug() << "Serial port error:" << serialPort->errorString();
        // Handle error appropriately, e.g., show message to user
        ui->Status->setText("Serial port error: " + serialPort->errorString());
    }
}



void MainWindow::updateSerialPortInfo()
{
    bool arduinoDetected = false;

    foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts()) {
        qDebug() << "Port Name:" << info.portName();
        qDebug() << "Vendor ID:" << info.vendorIdentifier();
        qDebug() << "Product ID:" << info.productIdentifier();

        // Convert vendor and product IDs to strings for comparison
        QString vendorIDString = QString::number(info.vendorIdentifier(), 16).toUpper();
        QString productIDString = QString::number(info.productIdentifier(), 16).toUpper();

        qDebug() << "Vendor ID (String):" << vendorIDString;
        qDebug() << "Product ID (String):" << productIDString;

        if (vendorIDString == arduinoVendorID && productIDString == arduinoProductID) {
            qDebug() << "Arduino found on port:" << info.portName();
            if (openSerialPort(info.portName())) {
                ui->Status->setText("Status: Arduino connected on port " + info.portName());
            }
            else
            {
                ui->Status->setText("Status: Failed to open serial port " + info.portName());
            }
            arduinoDetected = true;
            break;
        }
    }

    if (!arduinoDetected) {
        ui->Status->setText("Status: Arduino not detected");
    }
}



bool MainWindow::openSerialPort(const QString& portName)
{
    serialPort->setPortName(portName);
    serialPort->setBaudRate(QSerialPort::Baud115200); // Set your baud rate
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    return serialPort->open(QIODevice::ReadWrite);
}



QString MainWindow::convertSpeedValueToCommand(int speedValue) {
    // Assuming speedValue is already a hexadecimal string (e.g., "1" to "F")
    return QString("$v%1").arg(speedValue);
}


// Slot to handle spin box value change
void MainWindow::on_spinBox_valueChanged(int speedValue) {
    QString command = convertSpeedValueToCommand(speedValue);
    if (!command.isEmpty()) {
        sendCommandToArduino(command);
    }
}

void MainWindow::sendCommandToArduino(const QString& command) {
    QByteArray commandBytes = command.toUtf8(); // Convert QString to QByteArray

    if (serialPort->isOpen() && serialPort->isWritable()) {
        serialPort->write(commandBytes);
        serialPort->flush();
        qDebug() << "Sent to Arduino:" << command; // Debug output
    }
    else {
        qDebug() << "Serial port is not open or not writable";
    }
}
*/
////////////////////////////////////////
//TRIALS
/*
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QDir>
#include <QPixmap>
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <string>
extern std::atomic<int> totalcounter;

//extern std::atomic<bool> stopProcessing;
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , enqueueThread(nullptr)
    , processThread1(nullptr)
    , processThread2(nullptr)
    , processingThread(nullptr)
    , stopThreads(false)
    , stopProcessing(false)
    , currentImageIndex(0)
    , serialPort(new QSerialPort(this))
    , arduinoVendorID("1A86")
    , arduinoProductID("7523")
{
    ui->setupUi(this);
    updateSerialPortInfo();

    connect(ui->Detect, &QPushButton::clicked, this, &MainWindow::startProcessing);
    connect(ui->Stop, &QPushButton::clicked, this, &MainWindow::stopProcessing_1);
    connect(ui->Directory, &QPushButton::clicked, this, &MainWindow::selectOutputFolder);
    connect(ui->Display_Output_Images, &QPushButton::clicked, this, &MainWindow::displayAndNavigateImages);
    connect(ui->NEXT, &QPushButton::clicked, this, &MainWindow::showNextImage);
    connect(ui->PREVIOUS, &QPushButton::clicked, this, &MainWindow::showPreviousImage);
    connect(ui->Reset, &QPushButton::clicked, this, &MainWindow::resetApplication);

    connect(serialPort, SIGNAL(errorOccurred(QSerialPort::SerialPortError)), this, SLOT(handleSerialError(QSerialPort::SerialPortError)));
    connect(ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(on_spinBox_valueChanged(int)));

    ui->NEXT->setEnabled(false);
    ui->PREVIOUS->setEnabled(false);


}

MainWindow::~MainWindow()
{
    closeSerialPort();
    delete ui;
}

void MainWindow::startProcessing()
{
    QString startCommand = "$h";
    sendCommandToArduino(startCommand);

    int speedValue = ui->spinBox->value();
    QString speedValueString;
    if (speedValue > 0 && speedValue < 10) {
        speedValueString = QString::number(speedValue);
    }
    else if (speedValue >= 10 && speedValue <= 15) {
        speedValueString = QString(QChar('A' + (speedValue - 10)));
    }
    else {
        return;
    }

    QString speedCommand = QString("$v%1").arg(speedValueString);
    sendCommandToArduino(speedCommand);

    stopThreads = false;
    stopProcessing = false;

    enqueueThread = QThread::create([this] { enqueueBuffer1(ui->FPS); });
    processThread1 = QThread::create([this] { processBuffer1(); });
    processThread2 = QThread::create([this] { processBuffer2(); });
    processingThread = QThread::create([this] { furtherProcessing(this, ui->countEdit, ui->Status, ui->Frames); });
    enqueueThread->start();
    processThread1->start();
    processThread2->start();
    processingThread->start();

    processThread3 = QThread::create([this] { saving(outputDirectory.toStdString()); });
    processThread3->start();
    ui->Detect->setEnabled(false);
}



void MainWindow::resetApplication()
{
    // Stop processing if it's ongoing
    stopProcessing_1(); // Signal threads to stop and wait for them to finish

    // Wait for threads to finish
    auto waitForThread = [](QThread* thread, const char* threadName) {
        if (thread && thread->isRunning()) {
            thread->quit();
            if (!thread->wait(2000)) { // Wait for thread to finish with timeout
                qDebug() << QString("%1 did not finish in time").arg(threadName);
            }
            delete thread;
            thread = nullptr;
        }
        };

    waitForThread(enqueueThread, "enqueueThread");
    waitForThread(processThread1, "processThread1");
    waitForThread(processThread2, "processThread2");
    waitForThread(processingThread, "processingThread");

    // Clear resources or state
    outputDirectory.clear();
    imageFiles.clear();
    ui->countEdit->clear();
    ui->Status->clear();
    ui->Frames->clear();

    // Re-enable necessary buttons or reset any other state as needed
    ui->Detect->setEnabled(true);
}


void MainWindow::closeSerialPort()
{
    if (serialPort->isOpen()) {
        serialPort->close();
    }
}

void MainWindow::stopProcessing_1()
{
    QString stopCommand = "$s"; // Convert "$s" to QString
    sendCommandToArduino(stopCommand); // Send stop command to Arduino

    stopThreads = true; // Set stop flag to true
    stopProcessing = true;

    // Stop threads without blocking the main thread
    if (enqueueThread) {
        enqueueThread->quit();
    }
    if (processThread1) {
        processThread1->quit();
    }
    if (processThread2) {
        processThread2->quit();
    }
    if (processingThread) {
        processingThread->quit();
    }
}
void MainWindow::stopConveyor()
{
    QString stopCommand = "$s"; // Assuming $s is the stop command for your conveyor
    sendCommandToArduino(stopCommand);
}

void MainWindow::selectOutputFolder()
{
    outputDirectory = QFileDialog::getExistingDirectory(this, "Select Output Folder", QDir::homePath());
    if (!outputDirectory.isEmpty()) {
        ui->Status->setText("Status: Folder Selected - " + outputDirectory);
    }
}

void MainWindow::displayAndNavigateImages()
{
    // Stop processing before displaying images
    stopProcessing_1();
    QString stopCommand = "$s"; // Convert "$h" to QString
    sendCommandToArduino(stopCommand); // Send command to start operation

    // Simulate loading and navigating through images
    QTimer::singleShot(1000, this, [this]() {
        ui->Status->setText("Status: Displaying Images");
        // Enable navigation buttons
        ui->PREVIOUS->setEnabled(true);
        ui->NEXT->setEnabled(true);

        // Load and display images (example: load from a directory)
        QDir imageDir(outputDirectory);
        QStringList filters;
        filters << "*.jpg" << "*.png"; // Adjust filters as needed

        imageFiles = imageDir.entryList(filters, QDir::Files, QDir::Name);

        if (!imageFiles.isEmpty()) {
            // Sort the imageFiles based on their names
            std::sort(imageFiles.begin(), imageFiles.end(), [](const QString& file1, const QString& file2) {
                // Extract the numeric part from filenames (e.g., "image_1.jpg" -> "1")
                QString number1 = file1.split("_").last().split(".").first();
                QString number2 = file2.split("_").last().split(".").first();
                return number1.toInt() < number2.toInt();
                });

            currentImageIndex = 0; // Start with the first image
            showImage(imageFiles[currentImageIndex]);
        }
        else {
            // Handle case where no images are found
            ui->Status->setText("Status: No images found in selected directory");
        }
        });
}

void MainWindow::showPreviousImage()
{
    if (currentImageIndex > 0 && currentImageIndex < imageFiles.size()) {
        currentImageIndex--;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showNextImage()
{
    if (currentImageIndex >= 0 && currentImageIndex < imageFiles.size() - 1) {
        currentImageIndex++;
        showImage(imageFiles[currentImageIndex]);
    }
}

void MainWindow::showImage(const QString& fileName)
{
    QString imagePath = outputDirectory + "/" + fileName;
    QPixmap pixmap(imagePath);
    if (!pixmap.isNull()) {
        ui->Image->setPixmap(pixmap.scaled(ui->Image->size(), Qt::KeepAspectRatio));
    }
    else {
        ui->Status->setText("Status: Failed to load image - " + fileName);
    }
}

void MainWindow::handleSerialError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError) {
        qDebug() << "Serial port error:" << serialPort->errorString();
        // Handle error appropriately, e.g., show message to user
        ui->Status->setText("Serial port error: " + serialPort->errorString());
    }
}



void MainWindow::updateSerialPortInfo()
{
    bool arduinoDetected = false;

    foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts()) {
        qDebug() << "Port Name:" << info.portName();
        qDebug() << "Vendor ID:" << info.vendorIdentifier();
        qDebug() << "Product ID:" << info.productIdentifier();

        // Convert vendor and product IDs to strings for comparison
        QString vendorIDString = QString::number(info.vendorIdentifier(), 16).toUpper();
        QString productIDString = QString::number(info.productIdentifier(), 16).toUpper();

        qDebug() << "Vendor ID (String):" << vendorIDString;
        qDebug() << "Product ID (String):" << productIDString;

        if (vendorIDString == arduinoVendorID && productIDString == arduinoProductID) {
            qDebug() << "Arduino found on port:" << info.portName();
            if (openSerialPort(info.portName())) {
                ui->Status->setText("Status: Arduino connected on port " + info.portName());
            }
            else
            {
                ui->Status->setText("Status: Failed to open serial port " + info.portName());
            }
            arduinoDetected = true;
            break;
        }
    }

    if (!arduinoDetected) {
        ui->Status->setText("Status: Arduino not detected");
    }
}



bool MainWindow::openSerialPort(const QString& portName)
{
    serialPort->setPortName(portName);
    serialPort->setBaudRate(QSerialPort::Baud115200); // Set your baud rate
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    return serialPort->open(QIODevice::ReadWrite);
}



QString MainWindow::convertSpeedValueToCommand(int speedValue) {
    // Assuming speedValue is already a hexadecimal string (e.g., "1" to "F")
    return QString("$v%1").arg(speedValue);
}


// Slot to handle spin box value change
void MainWindow::on_spinBox_valueChanged(int speedValue) {
    QString command = convertSpeedValueToCommand(speedValue);
    if (!command.isEmpty()) {
        sendCommandToArduino(command);
    }
}

void MainWindow::sendCommandToArduino(const QString& command) {
    QByteArray commandBytes = command.toUtf8(); // Convert QString to QByteArray

    if (serialPort->isOpen() && serialPort->isWritable()) {
        serialPort->write(commandBytes);
        serialPort->flush();
        qDebug() << "Sent to Arduino:" << command; // Debug output
    }
    else {
        qDebug() << "Serial port is not open or not writable";
    }
}
*/