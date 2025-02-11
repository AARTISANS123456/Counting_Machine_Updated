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
#include "ImageProcessing.h"
#include "MvCameraControl.h"
#include <iostream>
//#include <QLineEdit>  // Include QLineEdit header for using QLineEdit
//#include <QString>    // Include QString header for using QString
//#include <QObject>

queue<Mat> buffer1;
queue<Mat> buffer2;
queue<Mat> buffer3;

mutex buffer1_mutex;
mutex buffer2_mutex;
mutex buffer3_mutex;

condition_variable buffer1_cv;
condition_variable buffer2_cv;
condition_variable buffer3_cv;

atomic<bool> stopThreads(false);
atomic<bool> stopProcessing(false);

using namespace std;
using namespace cv;
int totalcounter = 0;
int buff2 = 0;

// Global variables to store FPS and Frame Count
int fpsValue = 0;
int FrameCountvalue = 0;

void setFPSValue(QLineEdit* FPS) {

        QString fpsText = FPS->text();
        fpsValue = fpsText.toInt();
}

void setFrameCountValue(QLineEdit* FRAME_COUNT) {
    QString FrameCount = FRAME_COUNT->text();
    FrameCountvalue = FrameCount.toInt();
}

void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
    if (pFrameInfo) {
        static unsigned int counter = 0;
        counter++;
        cout << "Frame Number: " << pFrameInfo->nFrameNum << endl;

        Mat image(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
        {
            lock_guard<mutex> lock(buffer1_mutex);
            buffer1.push(image.clone());
            cout << "Image enqueued in buffer1" << endl;
            cout << "Frame Count: " << counter << endl;
            cout << "Host Timestamp: " << pFrameInfo->nHostTimeStamp << endl;
            buffer1_cv.notify_one();
        }

        if (counter >= FrameCountvalue) {
            stopThreads = true;
        }
    }
}

void enqueueBuffer1(QLineEdit* FPS, QLineEdit* FRAME_COUNT) {
    int nRet = MV_OK;
    void* handle = NULL;
    setFPSValue(FPS);
    setFrameCountValue(FRAME_COUNT);
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
    if (stDeviceList.nDeviceNum == 0) {
        cerr << "No devices found!" << endl;
        return;
    }

    unsigned int nIndex = 0;
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
        cerr << "Create handle failed! nRet " << nRet << endl;
        return;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        cerr << "Open device failed! nRet " << nRet << endl;
        return;
    }

    nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fpsValue);
    if (MV_OK != nRet) {
        cerr << "Failed to set frame rate! Error code: " << nRet << endl;
        return;
    }

    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
    if (MV_OK != nRet) {
        cerr << "Register image callback failed! nRet " << nRet << endl;
        return;
    }

    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        cerr << "Start grabbing failed! nRet " << nRet << endl;
        return;
    }

    while (!stopThreads);

    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
}

void processBuffer1() {
    while (!stopThreads) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer1_mutex);
            buffer1_cv.wait(lock, [] { return !buffer1.empty() || stopThreads; });
            if (stopThreads && buffer1.empty()) break;

            image = buffer1.front();
            buffer1.pop();
        }

        {
            lock_guard<mutex> lock(buffer2_mutex);
            buffer2.push(image);
            cout << "Image moved from buffer1 to buffer2" << endl;
            buffer2_cv.notify_one();
        }
    }
}

// Function to process images in buffer2 and detect contours
void processBuffer2() {
    while (!stopProcessing) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer2_mutex);
            buffer2_cv.wait(lock, [] { return !buffer2.empty() || stopProcessing; });
            if (stopProcessing && buffer2.empty()) break;

            image = buffer2.front();
            buffer2.pop();
        }

        // Process the image and find contours
        vector<vector<Point>> contours;
        Mat thresholded_image;
        threshold(image, thresholded_image, 60, 255, THRESH_BINARY_INV);
        findContours(thresholded_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Number of contours detected: " << contours.size() << endl;

        // If contours are detected, move the image to buffer3
        if (!contours.empty()) {
            lock_guard<mutex> lock(buffer3_mutex);
            buffer3.push(image);
            buff2++;
            cout << "Image with contours moved from buffer2 to buffer3" << endl;
            buffer3_cv.notify_one();
        }
    }
}



void clearNumberAndAdjustCounts(int imageIndex, int contourIndex, vector<vector<ContourData>>& prevContourData) {
    prevContourData[imageIndex][contourIndex].number = 0;
}
Rect defineROI(const vector<Point>& contour, int imageWidth, int imageHeight, int horizontalExpand) {
    Rect boundingBox = boundingRect(contour);
    boundingBox.x = max(0, boundingBox.x);
    boundingBox.y = max(0, boundingBox.y);
    boundingBox.width = min(imageWidth - boundingBox.x, boundingBox.width);
    boundingBox.height = min(imageHeight - boundingBox.y, boundingBox.height);

    int expandedWidth = boundingBox.width + 2 * horizontalExpand;
    int centerX = boundingBox.x + boundingBox.width / 2;
    int expandedX = centerX - expandedWidth / 2;
    expandedX = max(0, expandedX);
    expandedWidth = min(imageWidth - expandedX, expandedWidth);

    return Rect(expandedX, 0, expandedWidth, imageHeight);
}



// Function to calculate centroid of a contour
Point2f calculateCentroid(const vector<Point>& contour) {
    Moments m = moments(contour);
    return Point2f(m.m10 / m.m00, m.m01 / m.m00);
}



bool areRectanglesApproxEqual(const Rect& rect1, const Rect& rect2, int threshold) {
    return abs(rect1.x - rect2.x) <= threshold && abs(rect1.y - rect2.y) <= threshold &&
        abs(rect1.width - rect2.width) <= threshold && abs(rect1.height - rect2.height) <= threshold;
}

void processUniqueContour(const ContourData& contourData, Mat& inputImage, const vector<vector<Point>>& contours, int contourIndex) {

    drawContours(inputImage, contours, contourIndex, Scalar(255), 2);
    rectangle(inputImage, contourData.roi, Scalar(255), 2);
}




    void furtherProcessing(const string & outputDirectory, QLineEdit * countEdit, QLabel * statusLabel) {
        int counter = 0;
        int maxUniqueInFirstImage = 0;
        int uniqueCountInPreviousImage = 0;

        vector<vector<ContourData>> prevContourData; // Vector to store contour data from previous image
        vector<vector<ContourData>> changedContourData; // Vector to store changed contour data for next image

        while (!stopProcessing) {
            Mat inputImage;

            {
                unique_lock<mutex> lock(buffer3_mutex);
                buffer3_cv.wait(lock, [] { return !buffer3.empty() || stopProcessing; });
                if (buffer3.empty())
                {
                    QString statusText = QString("Processed all Images,Now Open the Output Directory %1").arg(counter);
                    statusLabel->setText(statusText);

                }
                if (buffer3.empty()) break;

                inputImage = buffer3.front();
                buffer3.pop();
            }

            counter++;
            vector<ContourData> currentContourData;
            vector<double> currentAreas;
            vector<Point2f> currentCentroids;
            vector<int> currentNumbers;
            int uniqueCount = 0;

            Mat grayImage, thresholded;
            threshold(inputImage, thresholded, 60, 255, THRESH_BINARY_INV);
            vector<vector<Point>> contours;
            findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
           //////////////////////////_____FOR BOTTOM TO TOP_____///////////////////////
            sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
                Rect r1 = boundingRect(c1);
                Rect r2 = boundingRect(c2);
                return r1.y + r1.height > r2.y + r2.height; // Sort from bottom to top
                });
            ////////////////////////______FOR TOP TO BOTTOM_______///////////////////
            sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
                Rect r1 = boundingRect(c1);
                Rect r2 = boundingRect(c2);
                return (r1.y + r1.height) < (r2.y + r2.height); // Sort from top to bottom
                });

            // Determine number of unique contours in the first image
            if (counter == 1) {
                maxUniqueInFirstImage = contours.size();
            }

            // Assign numbers and gather data for each contour
            for (size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
                const vector<Point>& contour = contours[contourIndex];
                Rect roi = defineROI(contour, inputImage.cols, inputImage.rows, 25);

                Point2f centroid = calculateCentroid(contour);
                double area = contourArea(contour);

                ContourData data;
                data.number = uniqueCountInPreviousImage + contourIndex + 1;
                data.roi = roi;
                data.centroid = centroid;
                data.area = area;

                currentContourData.push_back(data);
                currentAreas.push_back(area);
                currentCentroids.push_back(centroid);
                currentNumbers.push_back(data.number);

            }

            // Store current data for comparison in next image
            changedContourData.push_back(currentContourData);

            // Process each contour in the current image
            for (size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
                const ContourData& currentData = currentContourData[contourIndex];
                bool isUnique = true;

                // Compare current contour with previous contours
                for (size_t i = 0; i < prevContourData.size(); ++i) {
                    for (size_t j = 0; j < prevContourData[i].size(); ++j) {
                        const ContourData& prevData = prevContourData[i][j];

                        int tolerance = 500;

                        if (areRectanglesApproxEqual(prevData.roi, currentData.roi, tolerance)) {
                            if (((prevData.roi & currentData.roi).width > 0) && (prevData.roi.contains(currentData.centroid))) {
                                if (prevData.centroid.y < currentData.centroid.y) {
                                    clearNumberAndAdjustCounts(i, j, prevContourData);
                                    isUnique = false;
                                    break;
                                }
                            }
                        }
                    }
                    if (!isUnique) break;
                }

                // If unique, process the contour and update data
                if (isUnique) {
                    processUniqueContour(currentData, inputImage, contours, contourIndex);
                    uniqueCount++;
                }
            }
            totalcounter += uniqueCount;
            // Clear previous data and update with changed data for the next iteration
            prevContourData.clear();
            prevContourData = changedContourData;
            changedContourData.clear(); // Clear changedContourData for next iteration

            // Output total unique contours count for current image
            cout << "Total unique in image " << counter << ": " << uniqueCount << endl;
            QString countText = QString::number(totalcounter);
            countEdit->setText(countText);

            QString statusText = QString("Processing Image %1").arg(counter);
            statusLabel->setText(statusText);

            // Save current image data for future comparison
            string imageName = outputDirectory + "/image_" + to_string(counter) + ".jpg";
            imwrite(imageName, inputImage);
            cout << "Total:" << totalcounter << endl;

            // int progressValue = static_cast<int>((static_cast<double>(counter) / buff2 ) * 100);
            // progressBar->setValue(progressValue);

        }
    }
    */

    //____________________REV 2: CONVEYER SPEED CONTROL AND DISPLAYING THE OUTPUT IMAGES INORDER_______________
/*
*  Following Failed:
*   furtherProcessing function's core logic failed due to a different approach in detecting the unique contours 
* 
void furtherProcessing(QLineEdit* countEdit, QLabel* statusLabel) {
    int counter = 0;
    int maxUniqueInFirstImage = 0;
    int uniqueCountInPreviousImage = 0;
    int totalcounter = 0;

    vector<vector<ContourData>> prevContourData; // Vector to store contour data from previous image
    vector<vector<ContourData>> changedContourData; // Vector to store changed contour data for next image

    while (!stopProcessing) {
        Mat croppedImage;

        {
            unique_lock<mutex> lock(buffer3_mutex);
            buffer3_cv.wait(lock, [] { return !buffer3.empty() || stopProcessing; });

            if (stopProcessing && buffer3.empty()) break;

            croppedImage = buffer3.front();
            buffer3.pop();
        }

        counter++;
        vector<ContourData> currentContourData;

        // Rectangles for top and bottom ROIs
        Rect topROI(0, 0, croppedImage.cols, 150);
        Rect bottomROI(0, croppedImage.rows - 150, croppedImage.cols, 150);
        rectangle(croppedImage, topROI, Scalar(255), 2);
        rectangle(croppedImage, bottomROI, Scalar(255), 2);

        // Thresholding and finding contours
        Mat thresholded;
        threshold(croppedImage, thresholded, 60, 255, THRESH_BINARY_INV);
        vector<vector<Point>> contours;
        findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Sort contours from top to bottom
        sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
            Rect r1 = boundingRect(c1);
            Rect r2 = boundingRect(c2);
            return (r1.y + r1.height) < (r2.y + r2.height); // Sort from top to bottom
            });

        // Determine number of unique contours in the first image
        if (counter == 1) {
            maxUniqueInFirstImage = contours.size();
        }

        // Assign numbers and gather data for each contour
        for (size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
            const vector<Point>& contour = contours[contourIndex];

            // Calculate ROI, centroid, and area
            Rect roi = defineROI(contour, croppedImage.cols, croppedImage.rows, 25);
            Point2f centroid = calculateCentroid(contour);
            double area = contourArea(contour);


            ContourData data;
            data.number = uniqueCountInPreviousImage + contourIndex + 1;
            data.roi = roi;
            data.centroid = centroid;
            data.area = area;
            data.wasInTopOrBottomROI = false;

            currentContourData.push_back(data);
        }

        // Handle resetting `wasInTopOrBottomROI` flag for subsequent images based on area in topROI
        if (counter >= 1) {
            for (auto& contour : currentContourData) {
                // Calculate intersection area with topROI using the contour's bounding box
                Rect contourBoundingBox = boundingRect(contour.area);
                Rect intersectRect = contourBoundingBox & topROI;
                double intersectArea = intersectRect.area();

                // Calculate percentage of contour area in topROI
                double percentInTopROI = (intersectArea / contour.area) * 100.0;

                // Determine if contour is not unique based on area percentage in topROI
                if (percentInTopROI > 50.0) {  // Adjust threshold as needed
                    contour.wasInTopOrBottomROI = true;
                }
            }
        }

        // Store current data for comparison in next image
        changedContourData.push_back(currentContourData);

        // Process each contour in the current image
        int uniqueCount = 0;
        for (size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
            ContourData& currentData = currentContourData[contourIndex];
            bool isUnique = true;

            // Calculate intersection area with topROI using the contour's bounding box
            Rect contourBoundingBox = boundingRect(currentData.area);
            Rect intersectRect = contourBoundingBox & topROI;
            double intersectArea = intersectRect.area();

            // Calculate percentage of contour area in topROI
            double percentInTopROI = (intersectArea / currentData.area) * 100.0;

            // Determine if contour is not unique based on area percentage in topROI
            if (counter >= 1 && percentInTopROI > 50.0) {
                currentData.wasInTopOrBottomROI = true;
                continue; // Skip to next contour
            }

            // Compare current contour with previous contours
            for (size_t i = 0; i < prevContourData.size(); ++i) {
                for (size_t j = 0; j < prevContourData[i].size(); ++j) {
                    ContourData& prevData = prevContourData[i][j];
                    int tolerance = 500;

                    if (areRectanglesApproxEqual(prevData.roi, currentData.roi, tolerance)) {
                        if ((prevData.roi & currentData.roi).width > 0 && prevData.roi.contains(currentData.centroid)) {
                            if (prevData.centroid.y < currentData.centroid.y) {
                                if (prevData.wasInTopOrBottomROI == false) {
                                    clearNumberAndAdjustCounts(i, j, prevContourData);
                                    isUnique = false;
                                    break; // Exit the inner loop
                                }
                            }
                        }
                    }
                }
                if (!isUnique) break; // Exit the outer loop if current contour is not unique
            }

            if (isUnique) {
                processUniqueContour(currentData, croppedImage, contours, contourIndex);
                uniqueCount++;
            }
        }

        totalcounter += uniqueCount;

        // Clear previous data and update with changed data for the next iteration
        prevContourData.clear();
        prevContourData = changedContourData;
        changedContourData.clear(); // Clear changedContourData for next iteration

        // Output total unique contours count for current image
        cout << "Total unique in image " << counter << ": " << uniqueCount << endl;
        QString countText = QString::number(totalcounter);
        countEdit->setText(countText);

        QString statusText = QString("Processing Image %1").arg(counter);
        statusLabel->setText(statusText);

        // Push processed image to buffer4 for further processing/display
        lock_guard<mutex> lock(buffer4_mutex);
        buffer4.push(croppedImage);
        cout << "Image with contours moved from buffer3 to buffer4" << endl;
        buffer4_cv.notify_one();
    }
}


void saving(const string& outputDirectory)
{
    int count = 0;
    while (!stopProcessing) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer4_mutex);
            buffer4_cv.wait(lock, [] { return !buffer4.empty() || stopProcessing; });
            if (stopProcessing && buffer4.empty()) break;

            image = buffer4.front();
            buffer4.pop();
        }
        count++;
        // Save current image data for future comparison
        string imageName = outputDirectory + "/image_" + to_string(count) + ".jpg";
        imwrite(imageName, image);
        cout << "Total:" << totalcounter << endl;
    }
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* REV 3: EXACT COUNT OF CONTOURS GETTING DISPLAYED
* * Following works:
*   1.You get the exact count of the contours displayed, but the problem is we have we minimize the UI again for ther UI to refresh.
*   2.The stopConveyer gets executed properly but there is delay and thus letting more contours fall off, as the UI is not getting refreshed

*/

#include "ImageProcessing.h"
#include "MvCameraControl.h"
#include <iostream>
#include <QTimer>
#include "mainwindow.h" // Include this to recognize MainWindow
#include <QApplication> // Include this to use qApp
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QString>
#include <QDebug>

//#include <QLineEdit>  // Include QLineEdit header for using QLineEdit
//#include <QString>    // Include QString header for using QString
//#include <QObject>
using namespace std;
using namespace cv;
queue<Mat> buffer1;
queue<Mat> buffer2;
queue<Mat> buffer3;
queue<Mat> buffer4;
mutex buffer1_mutex;
mutex buffer2_mutex;
mutex buffer3_mutex;
mutex buffer4_mutex;
condition_variable buffer1_cv;
condition_variable buffer2_cv;
condition_variable buffer3_cv;
condition_variable buffer4_cv;
std::atomic<bool> stopThreads(false);
std::atomic<int> totalcounter(0);
std::atomic<bool> stopProcessing(false);
//int totalcounter = 0;
int counter = 0;
// Global variables to store FPS and Frame Count
int fpsValue = 0;
int FrameCountvalue = 0;


void setFPSValue(QLineEdit* FPS) {

    QString fpsText = FPS->text();
    fpsValue = fpsText.toInt();
}

void setFrameCountValue(QLineEdit* FRAME_COUNT) {
    QString FrameCount = FRAME_COUNT->text();
    FrameCountvalue = FrameCount.toInt();
}

void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo) {
        static unsigned int counter = 0;
        counter++;
        cout << "Frame Number: " << pFrameInfo->nFrameNum << endl;

        Mat image(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
        {
            lock_guard<mutex> lock(buffer1_mutex);
            buffer1.push(image.clone());
            cout << "Image enqueued in buffer1" << endl;
            cout << "Frame Count: " << counter << endl;
            cout << "Host Timestamp: " << pFrameInfo->nHostTimeStamp << endl;
            buffer1_cv.notify_one();
        }
    }
}

void enqueueBuffer1(QLineEdit* FPS)
{
    int nRet = MV_OK;
    void* handle = NULL;
    setFPSValue(FPS);

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));



        MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
        if (stDeviceList.nDeviceNum == 0) {
            cerr << "No devices found!" << endl;
            return;
        }

        unsigned int nIndex = 0;
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet) {
            cerr << "Create handle failed with error code: " << nRet
                << ". Possible reasons: device not found, or insufficient permissions." << endl; //20240720 the code snippet you have provided is used for error handling when a function or opration fails 
            return;
        }

        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet) {
            cerr << "Open device failed! nRet " << nRet << endl;
            return;
        }

        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fpsValue);
        if (MV_OK != nRet) {
            cerr << "Failed to set frame rate! Error code: " << nRet << endl;
            return;
        }

        nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
        if (MV_OK != nRet) {
            cerr << "Register image callback failed! nRet " << nRet << endl;
            return;
        }

        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet) {
            cerr << "Start grabbing failed! nRet " << nRet << endl;
            return;
        }
        while (!stopThreads);

    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);

}
void processBuffer1()
{
    while (!stopThreads.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); //AA20240720 Avoid busy-waiting
        Mat image;
        {
            unique_lock<mutex> lock(buffer1_mutex);
            buffer1_cv.wait(lock, [] { return !buffer1.empty() || stopThreads; });
            if (stopThreads) break;

            image = buffer1.front();
            buffer1.pop();
        }

        {
            lock_guard<mutex> lock(buffer2_mutex);
            buffer2.push(image);
            cout << "Image moved from buffer1 to buffer2" << endl;
            buffer2_cv.notify_one();
        }
    }
}
// Function to process images in buffer2 and detect contours
void processBuffer2()
{

    while (!stopProcessing) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer2_mutex);
            buffer2_cv.wait(lock, [] { return !buffer2.empty() || stopProcessing; });
            if (stopProcessing ) break;

            image = buffer2.front();
            buffer2.pop();
        }
        Rect x_ROI(0, 0, image.cols, image.rows); // Define your ROI (adjust as needed)
        Mat croppedImage = image(x_ROI); // Crop the image using the ROI
        // Process the image and find contours
        vector<vector<Point>> contours;
        Mat thresholded_image;
        threshold(croppedImage, thresholded_image, 60, 255, THRESH_BINARY_INV);
        findContours(thresholded_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Number of contours detected: " << contours.size() << endl;

        // If contours are detected, move the image to buffer3
        if (!contours.empty()) {
            lock_guard<mutex> lock(buffer3_mutex);
            buffer3.push(croppedImage);

            cout << "Image with contours moved from buffer2 to buffer3" << endl;
            buffer3_cv.notify_one();
        }
    }
}
void clearNumberAndAdjustCounts(int imageIndex, int contourIndex, vector<vector<ContourData>>& prevContourData) {
    prevContourData[imageIndex][contourIndex].number = 0;
}
Rect defineROI(const vector<Point>& contour, int imageWidth, int imageHeight, int horizontalExpand) {
    Rect boundingBox = boundingRect(contour);
    boundingBox.x = max(0, boundingBox.x);
    boundingBox.y = max(0, boundingBox.y);
    boundingBox.width = min(imageWidth - boundingBox.x, boundingBox.width);
    boundingBox.height = min(imageHeight - boundingBox.y, boundingBox.height);

    int expandedWidth = boundingBox.width + 2 * horizontalExpand;
    int centerX = boundingBox.x + boundingBox.width / 2;
    int expandedX = centerX - expandedWidth / 2;
    expandedX = max(0, expandedX);
    expandedWidth = min(imageWidth - expandedX, expandedWidth);

    return Rect(expandedX, 0, expandedWidth, imageHeight);
}
// Function to calculate centroid of a contour
Point2f calculateCentroid(const vector<Point>& contour) {
    Moments m = moments(contour);
    return Point2f(m.m10 / m.m00, m.m01 / m.m00);
}
bool areRectanglesApproxEqual(const Rect& rect1, const Rect& rect2, int threshold) {
    return abs(rect1.x - rect2.x) <= threshold && abs(rect1.y - rect2.y) <= threshold &&
        abs(rect1.width - rect2.width) <= threshold && abs(rect1.height - rect2.height) <= threshold;
}
void processUniqueContour(const ContourData& contourData, Mat& inputImage, const vector<vector<Point>>& contours, int contourIndex) {

    drawContours(inputImage, contours, contourIndex, Scalar(255), 2);
    rectangle(inputImage, contourData.roi, Scalar(255), 2);
}



// Add logging to track counter values and processing
void furtherProcessing(MainWindow* mainWindow, QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT)
{
    int counter_2 = 0;
    int maxUniqueInFirstImage = 0;
    int uniqueCountInPreviousImage = 0;
    vector<vector<ContourData>> prevContourData; // Vector to store contour data from previous image
    vector<vector<ContourData>> changedContourData; // Vector to store changed contour data for next image
    setFrameCountValue(FRAME_COUNT);
    while (!stopProcessing) {

        Mat croppedImage;
        {
            unique_lock<mutex> lock(buffer3_mutex);
            buffer3_cv.wait(lock, [] { return !buffer3.empty() || stopProcessing; });
            if (stopProcessing) break;
            croppedImage = buffer3.front();
            buffer3.pop();
        }

        counter_2++;
        vector<ContourData> currentContourData;
        vector<double> currentAreas;
        vector<Point2f> currentCentroids;
        vector<int> currentNumbers;
        int uniqueCount = 0;

        Mat grayImage, thresholded;
        threshold(croppedImage, thresholded, 60, 255, THRESH_BINARY_INV);
        vector<vector<Point>> contours;
        findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Number of contours detected: " << contours.size() << endl;

        sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
            Rect r1 = boundingRect(c1);
            Rect r2 = boundingRect(c2);
            return (r1.y + r1.height) < (r2.y + r2.height); // Sort from top to bottom
            });

        if (counter_2 == 1) {
            maxUniqueInFirstImage = contours.size();
        }

        for (size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
            const vector<Point>& contour = contours[contourIndex];

            Rect roi = defineROI(contour, croppedImage.cols, croppedImage.rows, 25);
            Point2f centroid = calculateCentroid(contour);
            double area = contourArea(contour);

            if (area > 500) {
                ContourData data;
                data.number = uniqueCountInPreviousImage + contourIndex + 1;
                data.roi = roi;
                data.centroid = centroid;
                data.area = area;

                currentContourData.push_back(data);
                currentAreas.push_back(area);
                currentCentroids.push_back(centroid);
                currentNumbers.push_back(data.number);
            }
        }

        changedContourData.push_back(currentContourData);

        for (size_t contourIndex = 0; contourIndex < currentContourData.size(); ++contourIndex) {
            const ContourData& currentData = currentContourData[contourIndex];
            bool isUnique = true;

            Rect topROI(0, 0, croppedImage.cols, 150);
            Rect bottomROI(0, croppedImage.rows - 150, croppedImage.cols, 150);

            for (size_t i = 0; i < prevContourData.size(); ++i) {
                for (size_t j = 0; j < prevContourData[i].size(); ++j) {
                    const ContourData& prevData = prevContourData[i][j];
                    int tolerance = 500;

                    if (areRectanglesApproxEqual(prevData.roi, currentData.roi, tolerance)) {
                        if ((prevData.roi & currentData.roi).width > 0 && prevData.roi.contains(currentData.centroid)) {
                            if (prevData.centroid.y < currentData.centroid.y) {
                                clearNumberAndAdjustCounts(i, j, prevContourData);
                                isUnique = false;
                                break;
                            }
                        }
                    }
                }
                if (!isUnique) break;
            }

            if (isUnique) {
                processUniqueContour(currentData, croppedImage, contours, contourIndex);
                uniqueCount++;
            }
        }

        totalcounter += uniqueCount;

        prevContourData.clear();
        prevContourData = changedContourData;
        changedContourData.clear();

        lock_guard<mutex> lock(buffer4_mutex);
        buffer4.push(croppedImage);
        cout << "Image with contours moved from buffer3 to buffer4" << endl;
        buffer4_cv.notify_one();

        cout << "Total unique in image " << counter_2 << ": " << uniqueCount << endl;
        QString countText = QString::number(totalcounter);
        countEdit->setText(countText);

        QString statusText = QString("Processing Image %1").arg(counter_2);
        statusLabel->setText(statusText);


        cout << "Total counter value: " << totalcounter << " | Frame Count value: " << FrameCountvalue << endl;
        
        if (totalcounter >= FrameCountvalue)
        {
            QMetaObject::invokeMethod(mainWindow, "stopConveyor", Qt::QueuedConnection);
            cout << "Conveyor stopped: Desired count reached" << endl;
            stopThreads = true;
            stopProcessing = true;
            cout << "Conveyor stopped: Desired count reached" << endl;
            break;
        }
      

    }
}


void saving(const string& outputDirectory)
{
    int count = 0;
    while (!stopProcessing)
    {
        Mat image;
        {
            unique_lock<mutex> lock(buffer4_mutex);
            buffer4_cv.wait(lock, [] { return !buffer4.empty() || stopProcessing; });
            if (stopProcessing && buffer4.empty()) break;

            image = buffer4.front();
            buffer4.pop();
        }
        count++;
        // Save current image data for future comparison
        string imageName = outputDirectory + "/image_" + to_string(count) + ".jpg";
        imwrite(imageName, image);
        cout << "Total:" << totalcounter << endl;
    }
}




////////////////////////////////////////////////////////////////////////////////////////////////////////*
// REV 4: STOPPING CONVEYER AFTER EXACT COUNT AND IMPLEMENTING A LOOKUP UP TABLE
/*
* Following works:
*   1.Lookup table implementation works properly but as the UI doesnot refresh consistently , thus many contours fall extra even though they are counted properly.
*   2.updateConveyorSpeed is the function that is been written for the lookup table mechanism.
*   3.The serialport communication functions were shifted to the ImageProcessing.cpp because of the updateConveyorSpeed function initial setup trials.
*/ 
/*
#include "ImageProcessing.h" 
#include "MvCameraControl.h"
#include <iostream>
#include <QTimer>
#include "mainwindow.h" // Include this to recognize MainWindow
#include <QApplication> // Include this to use qApp
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QString>
#include <QDebug>

//#include <QLineEdit>  // Include QLineEdit header for using QLineEdit
//#include <QString>    // Include QString header for using QString
//#include <QObject>
using namespace std;
using namespace cv;
queue<Mat> buffer1;
queue<Mat> buffer2;
queue<Mat> buffer3;
queue<Mat> buffer4;
mutex buffer1_mutex;
mutex buffer2_mutex;
mutex buffer3_mutex;
mutex buffer4_mutex;
condition_variable buffer1_cv;
condition_variable buffer2_cv;
condition_variable buffer3_cv;
condition_variable buffer4_cv;
atomic<bool> stopThreads(false);
std::atomic<int> totalcounter(0);
std::atomic<bool> stopProcessing(false);
//int totalcounter = 0;
int counter = 0;
// Global variables to store FPS and Frame Count
int fpsValue = 0;
int FrameCountvalue = 0;
static QSerialPort serialPort;
static const QString arduinoVendorID = "1A86";
static const QString arduinoProductID = "7523";
void sendCommandToArduino(const QString& command) {
    QByteArray commandBytes = command.toUtf8();

    if (serialPort.isOpen() && serialPort.isWritable()) {
        serialPort.write(commandBytes);
        serialPort.flush();
        QThread::msleep(1000); // Short delay to ensure the command is sent
        qDebug() << "Sent to Arduino:" << command;
    }
    else {
        qDebug() << "Serial port is not open or not writable";
    }
}

bool openSerialPort(const QString& portName) {
    serialPort.setPortName(portName);
    serialPort.setBaudRate(QSerialPort::Baud115200);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);

    return serialPort.open(QIODevice::ReadWrite);
}

void closeSerialPort() {
    if (serialPort.isOpen()) {
        serialPort.close();
    }
}

void updateSerialPortInfo(QLabel* statusLabel) {
    bool arduinoDetected = false;

    foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts()) {
        QString vendorIDString = QString::number(info.vendorIdentifier(), 16).toUpper();
        QString productIDString = QString::number(info.productIdentifier(), 16).toUpper();

        if (vendorIDString == arduinoVendorID && productIDString == arduinoProductID) {
            if (openSerialPort(info.portName())) {
                statusLabel->setText("Status: Arduino connected on port " + info.portName());
            }
            else {
                statusLabel->setText("Status: Failed to open serial port " + info.portName());
            }
            arduinoDetected = true;
            break;
        }
    }

    if (!arduinoDetected) {
        statusLabel->setText("Status: Arduino not detected");
    }
}

QString convertSpeedValueToCommand(int speedValue) {
    QString speedValueString;
    if (speedValue > 0 && speedValue < 10) {
        speedValueString = QString::number(speedValue);
    }
    else if (speedValue >= 10 && speedValue <= 15) {
        speedValueString = QString(QChar('A' + (speedValue - 10)));
    }
    else {
        return QString(); // Invalid speed value
    }
    return QString("$v%1").arg(speedValueString);
}

void stopConveyor()
{
    QString stopCommand = "$s";
    sendCommandToArduino(stopCommand);
    QThread::msleep(1000); // Short delay to ensure the command is sent
    cout << "Stop command sent to conveyor" << endl;
}

void setFPSValue(QLineEdit* FPS) {

    QString fpsText = FPS->text();
    fpsValue = fpsText.toInt();
}

void setFrameCountValue(QLineEdit* FRAME_COUNT) {
    QString FrameCount = FRAME_COUNT->text();
    FrameCountvalue = FrameCount.toInt();
}

void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
    if (pFrameInfo) {
        static unsigned int counter = 0;
        counter++;
        cout << "Frame Number: " << pFrameInfo->nFrameNum << endl;

        Mat image(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
        {
            lock_guard<mutex> lock(buffer1_mutex);
            buffer1.push(image.clone());
            cout << "Image enqueued in buffer1" << endl;
            cout << "Frame Count: " << counter << endl;
            cout << "Host Timestamp: " << pFrameInfo->nHostTimeStamp << endl;
            buffer1_cv.notify_one();
        }
    }
}

void enqueueBuffer1(QLineEdit* FPS)
{
    int nRet = MV_OK;
    void* handle = NULL;
    setFPSValue(FPS);

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
    if (stDeviceList.nDeviceNum == 0) {
        cerr << "No devices found!" << endl;
        return;
    }

    unsigned int nIndex = 0;
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
        cerr << "Create handle failed! nRet " << nRet << endl;
        return;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        cerr << "Open device failed! nRet " << nRet << endl;
        return;
    }

    nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fpsValue);
    if (MV_OK != nRet) {
        cerr << "Failed to set frame rate! Error code: " << nRet << endl;
        return;
    }

    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
    if (MV_OK != nRet) {
        cerr << "Register image callback failed! nRet " << nRet << endl;
        return;
    }

    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        cerr << "Start grabbing failed! nRet " << nRet << endl;
        return;
    }
    while (!stopThreads);
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);

}
void processBuffer1() {
    while (!stopThreads) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer1_mutex);
            buffer1_cv.wait(lock, [] { return !buffer1.empty() || stopThreads; });
            if (stopThreads && buffer1.empty()) break;

            image = buffer1.front();
            buffer1.pop();
        }

        {
            lock_guard<mutex> lock(buffer2_mutex);
            buffer2.push(image);
            cout << "Image moved from buffer1 to buffer2" << endl;
            buffer2_cv.notify_one();
        }
    }
}
// Function to process images in buffer2 and detect contours
void processBuffer2() {

    while (!stopProcessing) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer2_mutex);
            buffer2_cv.wait(lock, [] { return !buffer2.empty() || stopProcessing; });
            if (stopProcessing && buffer2.empty()) break;

            image = buffer2.front();
            buffer2.pop();
        }
        Rect x_ROI(0, 0, image.cols, image.rows); // Define your ROI (adjust as needed)
        Mat croppedImage = image(x_ROI); // Crop the image using the ROI
        // Process the image and find contours
        vector<vector<Point>> contours;
        Mat thresholded_image;
        threshold(croppedImage, thresholded_image, 60, 255, THRESH_BINARY_INV);
        findContours(thresholded_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Number of contours detected: " << contours.size() << endl;

        // If contours are detected, move the image to buffer3
        if (!contours.empty()) {
            lock_guard<mutex> lock(buffer3_mutex);
            buffer3.push(croppedImage);

            cout << "Image with contours moved from buffer2 to buffer3" << endl;
            buffer3_cv.notify_one();
        }
    }
}
void clearNumberAndAdjustCounts(int imageIndex, int contourIndex, vector<vector<ContourData>>& prevContourData) {
    prevContourData[imageIndex][contourIndex].number = 0;
}
Rect defineROI(const vector<Point>& contour, int imageWidth, int imageHeight, int horizontalExpand) {
    Rect boundingBox = boundingRect(contour);
    boundingBox.x = max(0, boundingBox.x);
    boundingBox.y = max(0, boundingBox.y);
    boundingBox.width = min(imageWidth - boundingBox.x, boundingBox.width);
    boundingBox.height = min(imageHeight - boundingBox.y, boundingBox.height);

    int expandedWidth = boundingBox.width + 2 * horizontalExpand;
    int centerX = boundingBox.x + boundingBox.width / 2;
    int expandedX = centerX - expandedWidth / 2;
    expandedX = max(0, expandedX);
    expandedWidth = min(imageWidth - expandedX, expandedWidth);

    return Rect(expandedX, 0, expandedWidth, imageHeight);
}
// Function to calculate centroid of a contour
Point2f calculateCentroid(const vector<Point>& contour) {
    Moments m = moments(contour);
    return Point2f(m.m10 / m.m00, m.m01 / m.m00);
}
bool areRectanglesApproxEqual(const Rect& rect1, const Rect& rect2, int threshold) {
    return abs(rect1.x - rect2.x) <= threshold && abs(rect1.y - rect2.y) <= threshold &&
        abs(rect1.width - rect2.width) <= threshold && abs(rect1.height - rect2.height) <= threshold;
}
void processUniqueContour(const ContourData& contourData, Mat& inputImage, const vector<vector<Point>>& contours, int contourIndex) {

    drawContours(inputImage, contours, contourIndex, Scalar(255), 2);
    rectangle(inputImage, contourData.roi, Scalar(255), 2);
}

// Update the conveyor speed and count display
void updateConveyorSpeed(int partCount, int totalParts, QLineEdit* speed_conv, QLineEdit* countEdit, MainWindow* mainWindow) {
    const int maxSpeed = 9;
    const int numberOfSpeedLevels = 9;

    int partsPerLevel = std::ceil(static_cast<double>(totalParts) / numberOfSpeedLevels);

    int speed = maxSpeed - (partCount - 1) / partsPerLevel;
    speed = std::max(speed, 1);  // Ensure speed doesn't go below 1

    QString speedText = QString::number(speed);
    speed_conv->setText(speedText);

    QString countText = QString::number(partCount);
    countEdit->setText(countText);

    if (mainWindow) {
        QString command = convertSpeedValueToCommand(speed);
        sendCommandToArduino(command);
    }
}

// Add logging to track counter values and processing
void furtherProcessing(MainWindow* mainWindow, QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT, QLineEdit* speed_conv)
{
    int counter_2 = 0;
    int maxUniqueInFirstImage = 0;
    int uniqueCountInPreviousImage = 0;
    vector<vector<ContourData>> prevContourData; // Vector to store contour data from previous image
    vector<vector<ContourData>> changedContourData; // Vector to store changed contour data for next image
    setFrameCountValue(FRAME_COUNT);
    while (!stopProcessing) {

        Mat croppedImage;
        {
            unique_lock<mutex> lock(buffer3_mutex);
            buffer3_cv.wait(lock, [] { return !buffer3.empty() || stopProcessing; });
            if (stopProcessing && buffer3.empty()) break;
            croppedImage = buffer3.front();
            buffer3.pop();
        }

        counter_2++;
        vector<ContourData> currentContourData;
        vector<double> currentAreas;
        vector<Point2f> currentCentroids;
        vector<int> currentNumbers;
        int uniqueCount = 0;

        Mat grayImage, thresholded;
        threshold(croppedImage, thresholded, 60, 255, THRESH_BINARY_INV);
        vector<vector<Point>> contours;
        findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Number of contours detected: " << contours.size() << endl;

        sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
            Rect r1 = boundingRect(c1);
            Rect r2 = boundingRect(c2);
            return (r1.y + r1.height) < (r2.y + r2.height); // Sort from top to bottom
            });

        if (counter_2 == 1) {
            maxUniqueInFirstImage = contours.size();
        }

        for (size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
            const vector<Point>& contour = contours[contourIndex];

            Rect roi = defineROI(contour, croppedImage.cols, croppedImage.rows, 25);
            Point2f centroid = calculateCentroid(contour);
            double area = contourArea(contour);

            if (area > 500) {
                ContourData data;
                data.number = uniqueCountInPreviousImage + contourIndex + 1;
                data.roi = roi;
                data.centroid = centroid;
                data.area = area;

                currentContourData.push_back(data);
                currentAreas.push_back(area);
                currentCentroids.push_back(centroid);
                currentNumbers.push_back(data.number);
            }
        }

        changedContourData.push_back(currentContourData);

        for (size_t contourIndex = 0; contourIndex < currentContourData.size(); ++contourIndex) {
            const ContourData& currentData = currentContourData[contourIndex];
            bool isUnique = true;

            Rect topROI(0, 0, croppedImage.cols, 150);
            Rect bottomROI(0, croppedImage.rows - 150, croppedImage.cols, 150);

            for (size_t i = 0; i < prevContourData.size(); ++i) {
                for (size_t j = 0; j < prevContourData[i].size(); ++j) {
                    const ContourData& prevData = prevContourData[i][j];
                    int tolerance = 500;

                    if (areRectanglesApproxEqual(prevData.roi, currentData.roi, tolerance)) {
                        if ((prevData.roi & currentData.roi).width > 0 && prevData.roi.contains(currentData.centroid)) {
                            if (prevData.centroid.y < currentData.centroid.y) {
                                clearNumberAndAdjustCounts(i, j, prevContourData);
                                isUnique = false;
                                break;
                            }
                        }
                    }
                }
                if (!isUnique) break;
            }

            if (isUnique) {
                processUniqueContour(currentData, croppedImage, contours, contourIndex);
                uniqueCount++;
            }
        }

        totalcounter += uniqueCount;

        prevContourData.clear();
        prevContourData = changedContourData;
        changedContourData.clear();

        lock_guard<mutex> lock(buffer4_mutex);
        buffer4.push(croppedImage);
        cout << "Image with contours moved from buffer3 to buffer4" << endl;
        buffer4_cv.notify_one();

        cout << "Total unique in image " << counter_2 << ": " << uniqueCount << endl;
        QString countText = QString::number(totalcounter);
        countEdit->setText(countText);

        QString statusText = QString("Processing Image %1").arg(counter_2);
        statusLabel->setText(statusText);


        cout << "Total counter value: " << totalcounter << " | Frame Count value: " << FrameCountvalue << endl;

        if (totalcounter >= FrameCountvalue)
        {
            stopConveyor();
            stopThreads = true;
            stopProcessing = true;
            cout << "Conveyor stopped: Desired count reached" << endl;
            break;
        }
        else
        {
            updateConveyorSpeed(totalcounter, FrameCountvalue, speed_conv, countEdit, mainWindow);

        }
}
}


void saving(const string& outputDirectory)
{
    int count = 0;
    while (!stopProcessing)
    {
        Mat image;
        {
            unique_lock<mutex> lock(buffer4_mutex);
            buffer4_cv.wait(lock, [] { return !buffer4.empty() || stopProcessing; });
            if (stopProcessing && buffer4.empty()) break;

            image = buffer4.front();
            buffer4.pop();
        }
        count++;
        // Save current image data for future comparison
        string imageName = outputDirectory + "/image_" + to_string(count) + ".jpg";
        imwrite(imageName, image);
        cout << "Total:" << totalcounter << endl;
    }
}


*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// FAILED DUE TO SERIAL LIBRARY
/*
#include "ImageProcessing.h"
#include "MvCameraControl.h"
#include <iostream>
#include <QTimer>
#include "mainwindow.h" // Include this to recognize MainWindow
#include <QApplication> // Include this to use qApp

#include <QThread>
#include <QString>
#include <QDebug>
#include <cmath>

#include <string>
#include <thread>
#include <chrono>

#include <serial/serial.h>

//#include <QLineEdit>  // Include QLineEdit header for using QLineEdit
//#include <QString>    // Include QString header for using QString
//#include <QObject>
using namespace std;
using namespace cv;
queue<Mat> buffer1;
queue<Mat> buffer2;
queue<Mat> buffer3;
queue<Mat> buffer4;
mutex buffer1_mutex;
mutex buffer2_mutex;
mutex buffer3_mutex;
mutex buffer4_mutex;
condition_variable buffer1_cv;
condition_variable buffer2_cv;
condition_variable buffer3_cv;
condition_variable buffer4_cv;
atomic<bool> stopThreads(false);
std::atomic<int> totalcounter(0);
std::atomic<bool> stopProcessing(false);
//int totalcounter = 0;
int counter = 0;
// Global variables to store FPS and Frame Count
int fpsValue = 0;
int FrameCountvalue = 0;
// Initialize serial port
static serial::Serial serialPort;
static const std::string arduinoVendorID = "1A86";
static const std::string arduinoProductID = "7523";

void sendCommandToArduino(const std::string& command) {
    if (serialPort.isOpen()) {
        serialPort.write(command);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Short delay to ensure the command is sent
        std::cout << "Sent to Arduino: " << command << std::endl;
    }
    else {
        std::cout << "Serial port is not open" << std::endl;
    }
}

bool openSerialPort(const std::string& portName) {
    serialPort.setPort(portName);
    serialPort.setBaudrate(115200);
    serialPort.setBytesize(serial::eightbits); // Use correct enumeration
    serialPort.setParity(serial::parity_none); // Correct usage
    serialPort.setStopbits(serial::stopbits_one); // Correct usage
    serialPort.setFlowcontrol(serial::flowcontrol_none); // Correct usage

    try {
        serialPort.open();
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
        return false;
    }
}

void closeSerialPort() {
    if (serialPort.isOpen()) {
        serialPort.close();
    }
}

void updateSerialPortInfo() {
    bool arduinoDetected = false;
    auto ports = serial::list_ports();

    for (const auto& port : ports) {
        std::string portName = port.port;
        std::string description = port.description;
        std::string hwid = port.hardware_id;

        // Note: `vendor_id` and `product_id` might not be available directly.
        // Use `hardware_id` for identification
        std::cout << "Port: " << portName << ", Description: " << description << ", Hardware ID: " << hwid << std::endl;

        // Example logic to identify Arduino by hardware ID
        if (hwid.find(arduinoVendorID) != std::string::npos && hwid.find(arduinoProductID) != std::string::npos) {
            if (openSerialPort(portName)) {
                std::cout << "Status: Arduino connected on port " << portName << std::endl;
            }
            else {
                std::cout << "Status: Failed to open serial port " << portName << std::endl;
            }
            arduinoDetected = true;
            break;
        }
    }

    if (!arduinoDetected) {
        std::cout << "Status: Arduino not detected" << std::endl;
    }
}


std::string convertSpeedValueToCommand(int speedValue) {
    std::string speedValueString;
    if (speedValue > 0 && speedValue < 10) {
        speedValueString = std::to_string(speedValue);
    }
    else if (speedValue >= 10 && speedValue <= 15) {
        speedValueString = static_cast<char>('A' + (speedValue - 10));
    }
    else {
        return ""; // Invalid speed value
    }
    return "$v" + speedValueString;
}

void stopConveyor() {
    std::string stopCommand = "$s";
    sendCommandToArduino(stopCommand);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Short delay to ensure the command is sent
    std::cout << "Stop command sent to conveyor" << std::endl;
}


void setFPSValue(QLineEdit* FPS) 
{

    QString fpsText = FPS->text();
    fpsValue = fpsText.toInt();
}
void setFrameCountValue(QLineEdit* FRAME_COUNT) {
    QString FrameCount = FRAME_COUNT->text();
    FrameCountvalue = FrameCount.toInt();
}
void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
    if (pFrameInfo) {

        static unsigned int counter = 0;
        counter++;
        cout << "Frame Number: " << pFrameInfo->nFrameNum << endl;

        Mat image(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
        {
            lock_guard<mutex> lock(buffer1_mutex);
            buffer1.push(image.clone());
            cout << "Image enqueued in buffer1" << endl;
            cout << "Frame Count: " << counter << endl;
            cout << "Host Timestamp: " << pFrameInfo->nHostTimeStamp << endl;
            buffer1_cv.notify_one();
        }
    }
}
void enqueueBuffer1(QLineEdit* FPS)
{
    std::string stopCommand = "$h";
    sendCommandToArduino(stopCommand);
        int nRet = MV_OK;
        void* handle = NULL;
        setFPSValue(FPS);
 
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
        if (stDeviceList.nDeviceNum == 0) {
            cerr << "No devices found!" << endl;
            return;
        }

        unsigned int nIndex = 0;
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet) {
            cerr << "Create handle failed! nRet " << nRet << endl;
            return;
        }

        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet) {
            cerr << "Open device failed! nRet " << nRet << endl;
            return;
        }

        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fpsValue);
        if (MV_OK != nRet) {
            cerr << "Failed to set frame rate! Error code: " << nRet << endl;
            return;
        }

        nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
        if (MV_OK != nRet) {
            cerr << "Register image callback failed! nRet " << nRet << endl;
            return;
        }

        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet) {
            cerr << "Start grabbing failed! nRet " << nRet << endl;
            return;
        }
        while (!stopThreads);
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);

}
void processBuffer1() {
    while (!stopThreads) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer1_mutex);
            buffer1_cv.wait(lock, [] { return !buffer1.empty() || stopThreads; });
            if (stopThreads && buffer1.empty()) break;

            image = buffer1.front();
            buffer1.pop();
        }

        {
            lock_guard<mutex> lock(buffer2_mutex);
            buffer2.push(image);
            cout << "Image moved from buffer1 to buffer2" << endl;
            buffer2_cv.notify_one();
        }
    }
}
void processBuffer2() {

    while (!stopProcessing) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer2_mutex);
            buffer2_cv.wait(lock, [] { return !buffer2.empty() || stopProcessing; });
            if (stopProcessing && buffer2.empty()) break;

            image = buffer2.front();
            buffer2.pop();
        }
        Rect x_ROI(0, 0, image.cols, image.rows); // Define your ROI (adjust as needed)
        Mat croppedImage = image(x_ROI); // Crop the image using the ROI
        // Process the image and find contours
        vector<vector<Point>> contours;
        Mat thresholded_image;
        threshold(croppedImage, thresholded_image, 60, 255, THRESH_BINARY_INV);
        findContours(thresholded_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Number of contours detected: " << contours.size() << endl;

        // If contours are detected, move the image to buffer3
        if (!contours.empty()) {
            lock_guard<mutex> lock(buffer3_mutex);
            buffer3.push(croppedImage);

            cout << "Image with contours moved from buffer2 to buffer3" << endl;
            buffer3_cv.notify_one();
        }
    }
}
void clearNumberAndAdjustCounts(int imageIndex, int contourIndex, vector<vector<ContourData>>& prevContourData) {
    prevContourData[imageIndex][contourIndex].number = 0;
}
Rect defineROI(const vector<Point>& contour, int imageWidth, int imageHeight, int horizontalExpand) {
    Rect boundingBox = boundingRect(contour);
    boundingBox.x = max(0, boundingBox.x);
    boundingBox.y = max(0, boundingBox.y);
    boundingBox.width = min(imageWidth - boundingBox.x, boundingBox.width);
    boundingBox.height = min(imageHeight - boundingBox.y, boundingBox.height);

    int expandedWidth = boundingBox.width + 2 * horizontalExpand;
    int centerX = boundingBox.x + boundingBox.width / 2;
    int expandedX = centerX - expandedWidth / 2;
    expandedX = max(0, expandedX);
    expandedWidth = min(imageWidth - expandedX, expandedWidth);

    return Rect(expandedX, 0, expandedWidth, imageHeight);
}
Point2f calculateCentroid(const vector<Point>& contour) {
    Moments m = moments(contour);
    return Point2f(m.m10 / m.m00, m.m01 / m.m00);
}
bool areRectanglesApproxEqual(const Rect& rect1, const Rect& rect2, int threshold) {
    return abs(rect1.x - rect2.x) <= threshold && abs(rect1.y - rect2.y) <= threshold &&
        abs(rect1.width - rect2.width) <= threshold && abs(rect1.height - rect2.height) <= threshold;
}
void processUniqueContour(const ContourData& contourData, Mat& inputImage, const vector<vector<Point>>& contours, int contourIndex) {

    drawContours(inputImage, contours, contourIndex, Scalar(255), 2);
    rectangle(inputImage, contourData.roi, Scalar(255), 2);
}
void updateConveyorSpeed(int partCount, int totalParts, QLineEdit* speed_conv, QLineEdit* countEdit) 
{
    // Define constants
    const int maxSpeed = 6;
    const int numberOfSpeedLevels = 6;

    // Calculate parts per level
    int partsPerLevel = std::ceil(static_cast<double>(totalParts) / numberOfSpeedLevels);

    // Calculate speed
    int speed = maxSpeed - (partCount - 1) / partsPerLevel;
    speed = std::max(speed, 1);  // Ensure speed doesn't go below 1

    QString speedText = QString::number(speed);
    speed_conv->setText(speedText);

    QString countText = QString::number(partCount);
    countEdit->setText(countText);
    sendCommandToArduino(convertSpeedValueToCommand(speed));
}
void furtherProcessing( QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT, QLineEdit* speed_conv)
{
    int counter_2 = 0;
    int maxUniqueInFirstImage = 0;
    int uniqueCountInPreviousImage = 0;
    vector<vector<ContourData>> prevContourData; // Vector to store contour data from previous image
    vector<vector<ContourData>> changedContourData; // Vector to store changed contour data for next image
    setFrameCountValue(FRAME_COUNT);
    while (!stopProcessing) {

        Mat croppedImage;
        {
            unique_lock<mutex> lock(buffer3_mutex);
            buffer3_cv.wait(lock, [] { return !buffer3.empty() || stopProcessing; });
            if (stopProcessing && buffer3.empty()) break;
            croppedImage = buffer3.front();
            buffer3.pop();
        }

        counter_2++;
        vector<ContourData> currentContourData;
        vector<double> currentAreas;
        vector<Point2f> currentCentroids;
        vector<int> currentNumbers;
        int uniqueCount = 0;

        Mat grayImage, thresholded;
        threshold(croppedImage, thresholded, 60, 255, THRESH_BINARY_INV);
        vector<vector<Point>> contours;
        findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Number of contours detected: " << contours.size() << endl;

        sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
            Rect r1 = boundingRect(c1);
            Rect r2 = boundingRect(c2);
            return (r1.y + r1.height) < (r2.y + r2.height); // Sort from top to bottom
            });

        if (counter_2 == 1) {
            maxUniqueInFirstImage = contours.size();
        }

        for (size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
            const vector<Point>& contour = contours[contourIndex];

            Rect roi = defineROI(contour, croppedImage.cols, croppedImage.rows, 25);
            Point2f centroid = calculateCentroid(contour);
            double area = contourArea(contour);

            if (area > 500) {
                ContourData data;
                data.number = uniqueCountInPreviousImage + contourIndex + 1;
                data.roi = roi;
                data.centroid = centroid;
                data.area = area;

                currentContourData.push_back(data);
                currentAreas.push_back(area);
                currentCentroids.push_back(centroid);
                currentNumbers.push_back(data.number);
            }
        }

        changedContourData.push_back(currentContourData);

        for (size_t contourIndex = 0; contourIndex < currentContourData.size(); ++contourIndex) {
            const ContourData& currentData = currentContourData[contourIndex];
            bool isUnique = true;

            Rect topROI(0, 0, croppedImage.cols, 150);
            Rect bottomROI(0, croppedImage.rows - 150, croppedImage.cols, 150);

            for (size_t i = 0; i < prevContourData.size(); ++i) {
                for (size_t j = 0; j < prevContourData[i].size(); ++j) {
                    const ContourData& prevData = prevContourData[i][j];
                    int tolerance = 500;

                    if (areRectanglesApproxEqual(prevData.roi, currentData.roi, tolerance)) {
                        if ((prevData.roi & currentData.roi).width > 0 && prevData.roi.contains(currentData.centroid)) {
                            if (prevData.centroid.y < currentData.centroid.y) {
                                clearNumberAndAdjustCounts(i, j, prevContourData);
                                isUnique = false;
                                break;
                            }
                        }
                    }
                }
                if (!isUnique) break;
            }

            if (isUnique) {
                processUniqueContour(currentData, croppedImage, contours, contourIndex);
                uniqueCount++;
            }
        }

        totalcounter += uniqueCount;
        
        prevContourData.clear();
        prevContourData = changedContourData;
        changedContourData.clear();

        lock_guard<mutex> lock(buffer4_mutex);
        buffer4.push(croppedImage);
        cout << "Image with contours moved from buffer3 to buffer4" << endl;
        buffer4_cv.notify_one();

        cout << "Total unique in image " << counter_2 << ": " << uniqueCount << endl;
       
       //
        QString statusText = QString("Processing Image %1").arg(counter_2);
        statusLabel->setText(statusText);
        
       
      
        if (totalcounter > FrameCountvalue)
        {
            stopConveyor();
            stopThreads = true;
            stopProcessing = true;
            cout << "Conveyor stopped: Desired count reached" << endl;
            break;
        }
        else
        {
            updateConveyorSpeed(totalcounter, FrameCountvalue, speed_conv, countEdit);
        }
      
    }
}
/*
void saving(const string& outputDirectory)
{
    int count = 0;
    while (!stopProcessing) 
    {
        Mat image;
        {
            unique_lock<mutex> lock(buffer4_mutex);
            buffer4_cv.wait(lock, [] { return !buffer4.empty() || stopProcessing; });
            if (stopProcessing && buffer4.empty()) break;

            image = buffer4.front();
            buffer4.pop();
        }
        count++;
        // Save current image data for future comparison
        string imageName = outputDirectory + "/image_" + to_string(count) + ".jpg";
        imwrite(imageName, image);
        cout << "Total:" << totalcounter << endl;
    }
}
*/




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
#include "ImageProcessing.h"
#include "mainwindow.h" // Include this to recognize MainWindow
#include <QApplication> // Include this to use qApp
#include <chrono>
#include <thread>
#include "MvCameraControl.h"
#include <iostream>
#include <atomic>
#include <iostream>
#include <QTimer>
#include <QDebug>

//#include <QLineEdit>  // Include QLineEdit header for using QLineEdit
//#include <QString>    // Include QString header for using QString
//#include <QObject>
using namespace std;
using namespace cv;
queue<Mat> buffer1;
queue<Mat> buffer2;
queue<Mat> buffer3;
queue<Mat> buffer4;
mutex buffer1_mutex;
mutex buffer2_mutex;
mutex buffer3_mutex;
mutex buffer4_mutex;
condition_variable buffer1_cv;
condition_variable buffer2_cv;
condition_variable buffer3_cv;
condition_variable buffer4_cv;


int desiredCount = 0;
// Define the external variables
bool stopProcessing = false;
int totalcounter = 0;
int FrameCountvalue = 0;

std::mutex counter_mutex;
//int totalcounter = 0;
int counter = 0;




void setFrameCountValue(QLineEdit* FRAME_COUNT) {
    QString FrameCount = FRAME_COUNT->text();
    FrameCountvalue = FrameCount.toInt();
}

//const int DESIRED_COUNT = 100; // Set your desired count here

void processbuffer1();
void processbuffer2();
void furtherProcessing(MainWindow* mainWindow, QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT);
void print_frame_info(const MV_FRAME_OUT& frame)
{
    const MV_FRAME_OUT_INFO_EX& info = frame.stFrameInfo;

    std::cout << "Frame Info:" << std::endl;
  //  std::cout << "Width: " << info.nWidth << std::endl;
  //  std::cout << "Height: " << info.nHeight << std::endl;
  //  std::cout << "Pixel Type: " << info.enPixelType << std::endl;
    std::cout << "Frame Number: " << info.nFrameNum << std::endl;
   // std::cout << "Device Timestamp High: " << info.nDevTimeStampHigh << std::endl;
//std::cout << "Device Timestamp Low: " << info.nDevTimeStampLow << std::endl;
    std::cout << "Host Timestamp: " << info.nHostTimeStamp << std::endl;
  //  std::cout << "Frame Length: " << info.nFrameLen << std::endl;
  //  std::cout << "Average Brightness: " << info.nAverageBrightness << std::endl;
  //  std::cout << "Red: " << info.nRed << std::endl;
  //  std::cout << "Green: " << info.nGreen << std::endl;
  //  std::cout << "Blue: " << info.nBlue << std::endl;
    std::cout << "Frame Counter: " << info.nFrameCounter << std::endl;
  //  std::cout << "Trigger Index: " << info.nTriggerIndex << std::endl;
  //  std::cout << "Input: " << info.nInput << std::endl;
  //  std::cout << "Output: " << info.nOutput << std::endl;
  //  std::cout << "Lost Packet: " << info.nLostPacket << std::endl;
  //  std::cout << "Offset X: " << info.nOffsetX << std::endl;
  //  std::cout << "Offset Y: " << info.nOffsetY << std::endl;
}

// Define the function here
void run_camera(QLineEdit* FRAME_COUNT, MainWindow* mainWindow, QLineEdit* countEdit, QLabel* statusLabel)
{
    setFrameCountValue(FRAME_COUNT);
    int nRet = MV_OK;
    void* handle = NULL;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
    if (stDeviceList.nDeviceNum == 0) {
        qDebug() << "No devices found!";
        return;
    }

    unsigned int nIndex = 0;
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
        qDebug() << "Create handle failed! nRet" << nRet;
        return;
    }

    // Open device
    nRet = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
    if (MV_OK != nRet)
    {
        qDebug() << "Error: OpenDevice fail [" << nRet << "]";
        return;
    }

    // Start grabbing images
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        qDebug() << "Error: StartGrabbing fail [" << nRet << "]";
        MV_CC_DestroyHandle(handle);
        return;
    }

    MV_FRAME_OUT stOutFrame;
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));

    // Buffer management loop
    while ( totalcounter < FrameCountvalue)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
        if (MV_OK == nRet)
        {
            // Create threads to process the image data
            std::thread buffer1Thread([]() {
                processbuffer1();
                });

            std::thread buffer2Thread([]() {
                processbuffer2();
                });

            std::thread furtherProcessingThread([&]() {
                furtherProcessing(mainWindow, countEdit, statusLabel, FRAME_COUNT);
                });

            // Print frame information
            print_frame_info(stOutFrame);

            nRet = MV_CC_FreeImageBuffer(handle, &stOutFrame);
            if (nRet != MV_OK)
            {
                qDebug() << "Free Image Buffer fail! nRet [0x" << std::hex << nRet << "]";
            }

            // Join threads to ensure they complete before proceeding
            if (buffer1Thread.joinable()) buffer1Thread.join();
            if (buffer2Thread.joinable()) buffer2Thread.join();
            if (furtherProcessingThread.joinable()) furtherProcessingThread.join();
        }
        {
            std::lock_guard<std::mutex> lock(counter_mutex);
            if (totalcounter >= FrameCountvalue)
            {
                QMetaObject::invokeMethod(mainWindow, "stopConveyor", Qt::QueuedConnection);
                std::cout << "Conveyor stopped: Desired count reached" << std::endl;
                break;
            }
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
        auto sleep_duration = std::chrono::milliseconds(16) - elapsed_time;

        if (sleep_duration > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleep_duration);
        }
    }

    // Stop image acquisition
    nRet = MV_CC_StopGrabbing(handle);
    if (nRet != MV_OK)
    {
        qDebug() << "Error: StopGrabbing fail [" << nRet << "]";
    }

    // Close device and release resources
    nRet = MV_CC_CloseDevice(handle);
    if (nRet != MV_OK)
    {
        qDebug() << "Error: CloseDevice fail [" << nRet << "]";
    }

    nRet = MV_CC_DestroyHandle(handle);
    if (nRet != MV_OK)
    {
        qDebug() << "Error: DestroyHandle fail [" << nRet << "]";
    }
}


void processbuffer1()
{
    // Implement buffer 1 processing logic here
    while (!stopProcessing) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer1_mutex);
            buffer1_cv.wait(lock, [] { return !buffer1.empty() || stopProcessing; });
            if (stopProcessing && buffer1.empty()) break;

            image = buffer1.front();
            buffer1.pop();
        }

        {
            lock_guard<mutex> lock(buffer2_mutex);
            buffer2.push(image);
            qDebug() << "Image moved from buffer1 to buffer2";
            buffer2_cv.notify_one();
        }
    }
}


void processbuffer2()
{
    // Implement buffer 2 processing logic here

    while (!stopProcessing) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer2_mutex);
            buffer2_cv.wait(lock, [] { return !buffer2.empty() || stopProcessing; });
            if (stopProcessing && buffer2.empty()) break;

            image = buffer2.front();
            buffer2.pop();
        }
        Rect x_ROI(0, 0, image.cols, image.rows); // Define your ROI (adjust as needed)
        Mat croppedImage = image(x_ROI); // Crop the image using the ROI
        // Process the image and find contours
        vector<vector<Point>> contours;
        Mat thresholded_image;
        threshold(croppedImage, thresholded_image, 60, 255, THRESH_BINARY_INV);
        findContours(thresholded_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        qDebug() << "Number of contours detected:" << contours.size();

        // If contours are detected, move the image to buffer3
        if (!contours.empty()) {
            lock_guard<mutex> lock(buffer3_mutex);
            buffer3.push(croppedImage);

            qDebug() << "Image with contours moved from buffer2 to buffer3";
            buffer3_cv.notify_one();
        }
    }
}

void clearNumberAndAdjustCounts(int imageIndex, int contourIndex, vector<vector<ContourData>>& prevContourData) {
    prevContourData[imageIndex][contourIndex].number = 0;
}
Rect defineROI(const vector<Point>& contour, int imageWidth, int imageHeight, int horizontalExpand) {
    Rect boundingBox = boundingRect(contour);
    boundingBox.x = max(0, boundingBox.x);
    boundingBox.y = max(0, boundingBox.y);
    boundingBox.width = min(imageWidth - boundingBox.x, boundingBox.width);
    boundingBox.height = min(imageHeight - boundingBox.y, boundingBox.height);

    int expandedWidth = boundingBox.width + 2 * horizontalExpand;
    int centerX = boundingBox.x + boundingBox.width / 2;
    int expandedX = centerX - expandedWidth / 2;
    expandedX = max(0, expandedX);
    expandedWidth = min(imageWidth - expandedX, expandedWidth);

    return Rect(expandedX, 0, expandedWidth, imageHeight);
}
// Function to calculate centroid of a contour
Point2f calculateCentroid(const vector<Point>& contour) {
    Moments m = moments(contour);
    return Point2f(m.m10 / m.m00, m.m01 / m.m00);
}
bool areRectanglesApproxEqual(const Rect& rect1, const Rect& rect2, int threshold) {
    return abs(rect1.x - rect2.x) <= threshold && abs(rect1.y - rect2.y) <= threshold &&
        abs(rect1.width - rect2.width) <= threshold && abs(rect1.height - rect2.height) <= threshold;
}
void processUniqueContour(const ContourData& contourData, Mat& inputImage, const vector<vector<Point>>& contours, int contourIndex) {

    drawContours(inputImage, contours, contourIndex, Scalar(255), 2);
    rectangle(inputImage, contourData.roi, Scalar(255), 2);
}

void furtherProcessing(MainWindow* mainWindow, QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT) {
    setFrameCountValue(FRAME_COUNT);
    int counter_2 = 0;
    int maxUniqueInFirstImage = 0;
    int uniqueCountInPreviousImage = 0;
    vector<vector<ContourData>> prevContourData; // Vector to store contour data from previous image
    vector<vector<ContourData>> changedContourData; // Vector to store changed contour data for next image

    while (!stopProcessing) {
        Mat croppedImage;
        {
            unique_lock<mutex> lock(buffer3_mutex);
            buffer3_cv.wait(lock, [] { return !buffer3.empty() || stopProcessing; });
            if (stopProcessing && buffer3.empty()) break;
            croppedImage = buffer3.front();
            buffer3.pop();
        }

        counter_2++;
        vector<ContourData> currentContourData;
        vector<double> currentAreas;
        vector<Point2f> currentCentroids;
        vector<int> currentNumbers;
        int uniqueCount = 0;

        Mat grayImage, thresholded;
        threshold(croppedImage, thresholded, 60, 255, THRESH_BINARY_INV);
        vector<vector<Point>> contours;
        findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
            Rect r1 = boundingRect(c1);
            Rect r2 = boundingRect(c2);
            return (r1.y + r1.height) < (r2.y + r2.height); // Sort from top to bottom
            });

        if (counter_2 == 1) {
            maxUniqueInFirstImage = contours.size();
        }

        for (size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
            const vector<Point>& contour = contours[contourIndex];

            Rect roi = defineROI(contour, croppedImage.cols, croppedImage.rows, 25);
            Point2f centroid = calculateCentroid(contour);
            double area = contourArea(contour);

            if (area > 500) {
                ContourData data;
                data.number = uniqueCountInPreviousImage + contourIndex + 1;
                data.roi = roi;
                data.centroid = centroid;
                data.area = area;

                currentContourData.push_back(data);
                currentAreas.push_back(area);
                currentCentroids.push_back(centroid);
                currentNumbers.push_back(data.number);
            }
        }

        changedContourData.push_back(currentContourData);

        for (size_t contourIndex = 0; contourIndex < currentContourData.size(); ++contourIndex) {
            const ContourData& currentData = currentContourData[contourIndex];
            bool isUnique = true;

            Rect topROI(0, 0, croppedImage.cols, 150);
            Rect bottomROI(0, croppedImage.rows - 150, croppedImage.cols, 150);

            for (size_t i = 0; i < prevContourData.size(); ++i) {
                for (size_t j = 0; j < prevContourData[i].size(); ++j) {
                    const ContourData& prevData = prevContourData[i][j];
                    int tolerance = 500;

                    if (areRectanglesApproxEqual(prevData.roi, currentData.roi, tolerance)) {
                        if ((prevData.roi & currentData.roi).width > 0 && prevData.roi.contains(currentData.centroid)) {
                            if (prevData.centroid.y < currentData.centroid.y) {
                                clearNumberAndAdjustCounts(i, j, prevContourData);
                                isUnique = false;
                                break;
                            }
                        }
                    }
                }
                if (!isUnique) break;
            }

            if (isUnique) {
                processUniqueContour(currentData, croppedImage, contours, contourIndex);
                uniqueCount++;
            }
        }

        {
            std::lock_guard<std::mutex> lock(counter_mutex);
            totalcounter += uniqueCount;
        }
       // totalCounter += uniqueCount;
        prevContourData.clear();
        prevContourData = changedContourData;
        changedContourData.clear();

        cout << "Total unique in image " << counter_2 << ": " << uniqueCount << endl;
        QString countText = QString::number(totalcounter);
        countEdit->setText(countText);

        QString statusText = QString("Processing Image %1").arg(counter_2);
        statusLabel->setText(statusText);

        lock_guard<mutex> lock(buffer4_mutex);
        buffer4.push(croppedImage);
        cout << "Image with contours moved from buffer3 to buffer4" << endl;
        buffer4_cv.notify_one();
    }
}
void saving(const string& outputDirectory)
{
    int count = 0;
    while (!stopProcessing) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer4_mutex);
            buffer4_cv.wait(lock, [] { return !buffer4.empty() || stopProcessing; });
            if (stopProcessing && buffer4.empty()) break;

            image = buffer4.front();
            buffer4.pop();
        }
        count++;
        // Save current image data for future comparison
        string imageName = outputDirectory + "/image_" + to_string(count) + ".jpg";
        imwrite(imageName, image);
        cout << "Total:" << totalcounter << endl;
    }
}
*/




////////////////////////////////////////////////////////////////////////////////
// Trials
/*
#include "ImageProcessing.h"
#include "MvCameraControl.h"
#include <iostream>
#include <QTimer>
#include "mainwindow.h" // Include this to recognize MainWindow
#include <QApplication> // Include this to use qApp
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QString>
#include <QDebug>

//#include <QLineEdit>  // Include QLineEdit header for using QLineEdit
//#include <QString>    // Include QString header for using QString
//#include <QObject>
using namespace std;
using namespace cv;
queue<Mat> buffer1;
queue<Mat> buffer2;
queue<Mat> buffer3;
queue<Mat> buffer4;
mutex buffer1_mutex;
mutex buffer2_mutex;
mutex buffer3_mutex;
mutex buffer4_mutex;
condition_variable buffer1_cv;
condition_variable buffer2_cv;
condition_variable buffer3_cv;
condition_variable buffer4_cv;
std::atomic<bool> stopThreads(false);
std::atomic<int> totalcounter(0);
std::atomic<bool> stopProcessing(false);
//int totalcounter = 0;
int counter = 0;
// Global variables to store FPS and Frame Count
int fpsValue = 0;
int FrameCountvalue = 0;


void setFPSValue(QLineEdit* FPS) {

    QString fpsText = FPS->text();
    fpsValue = fpsText.toInt();
}

void setFrameCountValue(QLineEdit* FRAME_COUNT) {
    QString FrameCount = FRAME_COUNT->text();
    FrameCountvalue = FrameCount.toInt();
}

void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
    if (pFrameInfo) {
        static unsigned int counter = 0;
        counter++;
        cout << "Frame Number: " << pFrameInfo->nFrameNum << endl;

        Mat image(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
        {
            lock_guard<mutex> lock(buffer1_mutex);
            buffer1.push(image.clone());
            cout << "Image enqueued in buffer1" << endl;
            cout << "Frame Count: " << counter << endl;
            cout << "Host Timestamp: " << pFrameInfo->nHostTimeStamp << endl;
            buffer1_cv.notify_one();
        }
    }
}

void enqueueBuffer1(QLineEdit* FPS)
{
    int nRet = MV_OK;
    void* handle = NULL;
    setFPSValue(FPS);

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
 


        MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
        if (stDeviceList.nDeviceNum == 0) {
            cerr << "No devices found!" << endl;
            return;
        }

        unsigned int nIndex = 0;
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet) {
            cerr << "Create handle failed! nRet " << nRet << endl;
            return;
        }

        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet) {
            cerr << "Open device failed! nRet " << nRet << endl;
            return;
        }

        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fpsValue);
        if (MV_OK != nRet) {
            cerr << "Failed to set frame rate! Error code: " << nRet << endl;
            return;
        }

        nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
        if (MV_OK != nRet) {
            cerr << "Register image callback failed! nRet " << nRet << endl;
            return;
        }

        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet) {
            cerr << "Start grabbing failed! nRet " << nRet << endl;
            return;
        }
        while (!stopThreads);
    
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);

}
void processBuffer1() {
    while (!stopThreads) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer1_mutex);
            buffer1_cv.wait(lock, [] { return !buffer1.empty() || stopThreads; });
            if (stopThreads) break;

            image = buffer1.front();
            buffer1.pop();
        }

        {
            lock_guard<mutex> lock(buffer2_mutex);
            buffer2.push(image);
            cout << "Image moved from buffer1 to buffer2" << endl;
            buffer2_cv.notify_one();
        }
    }
}
// Function to process images in buffer2 and detect contours
void processBuffer2() {

    while (!stopProcessing) {
        Mat image;
        {
            unique_lock<mutex> lock(buffer2_mutex);
            buffer2_cv.wait(lock, [] { return !buffer2.empty() || stopProcessing; });
            if (stopProcessing ) break;

            image = buffer2.front();
            buffer2.pop();
        }
        Rect x_ROI(0, 0, image.cols, image.rows); // Define your ROI (adjust as needed)
        Mat croppedImage = image(x_ROI); // Crop the image using the ROI
        // Process the image and find contours
        vector<vector<Point>> contours;
        Mat thresholded_image;
        threshold(croppedImage, thresholded_image, 60, 255, THRESH_BINARY_INV);
        findContours(thresholded_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Number of contours detected: " << contours.size() << endl;

        // If contours are detected, move the image to buffer3
        if (!contours.empty()) {
            lock_guard<mutex> lock(buffer3_mutex);
            buffer3.push(croppedImage);

            cout << "Image with contours moved from buffer2 to buffer3" << endl;
            buffer3_cv.notify_one();
        }
    }
}
void clearNumberAndAdjustCounts(int imageIndex, int contourIndex, vector<vector<ContourData>>& prevContourData) {
    prevContourData[imageIndex][contourIndex].number = 0;
}
Rect defineROI(const vector<Point>& contour, int imageWidth, int imageHeight, int horizontalExpand) {
    Rect boundingBox = boundingRect(contour);
    boundingBox.x = max(0, boundingBox.x);
    boundingBox.y = max(0, boundingBox.y);
    boundingBox.width = min(imageWidth - boundingBox.x, boundingBox.width);
    boundingBox.height = min(imageHeight - boundingBox.y, boundingBox.height);

    int expandedWidth = boundingBox.width + 2 * horizontalExpand;
    int centerX = boundingBox.x + boundingBox.width / 2;
    int expandedX = centerX - expandedWidth / 2;
    expandedX = max(0, expandedX);
    expandedWidth = min(imageWidth - expandedX, expandedWidth);

    return Rect(expandedX, 0, expandedWidth, imageHeight);
}
// Function to calculate centroid of a contour
Point2f calculateCentroid(const vector<Point>& contour) {
    Moments m = moments(contour);
    return Point2f(m.m10 / m.m00, m.m01 / m.m00);
}
bool areRectanglesApproxEqual(const Rect& rect1, const Rect& rect2, int threshold) {
    return abs(rect1.x - rect2.x) <= threshold && abs(rect1.y - rect2.y) <= threshold &&
        abs(rect1.width - rect2.width) <= threshold && abs(rect1.height - rect2.height) <= threshold;
}
void processUniqueContour(const ContourData& contourData, Mat& inputImage, const vector<vector<Point>>& contours, int contourIndex) {

    drawContours(inputImage, contours, contourIndex, Scalar(255), 2);
    rectangle(inputImage, contourData.roi, Scalar(255), 2);
}



// Add logging to track counter values and processing
void furtherProcessing(MainWindow* mainWindow, QLineEdit* countEdit, QLabel* statusLabel, QLineEdit* FRAME_COUNT)
{
    int counter_2 = 0;
    int maxUniqueInFirstImage = 0;
    int uniqueCountInPreviousImage = 0;
    vector<vector<ContourData>> prevContourData; // Vector to store contour data from previous image
    vector<vector<ContourData>> changedContourData; // Vector to store changed contour data for next image
    setFrameCountValue(FRAME_COUNT);
    while (!stopProcessing) {

        Mat croppedImage;
        {
            unique_lock<mutex> lock(buffer3_mutex);
            buffer3_cv.wait(lock, [] { return !buffer3.empty() || stopProcessing; });
            if (stopProcessing) break;
            croppedImage = buffer3.front();
            buffer3.pop();
        }

        counter_2++;
        vector<ContourData> currentContourData;
        vector<double> currentAreas;
        vector<Point2f> currentCentroids;
        vector<int> currentNumbers;
        int uniqueCount = 0;

        Mat grayImage, thresholded;
        threshold(croppedImage, thresholded, 60, 255, THRESH_BINARY_INV);
        vector<vector<Point>> contours;
        findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "Number of contours detected: " << contours.size() << endl;

        sort(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
            Rect r1 = boundingRect(c1);
            Rect r2 = boundingRect(c2);
            return (r1.y + r1.height) < (r2.y + r2.height); // Sort from top to bottom
            });

        if (counter_2 == 1) {
            maxUniqueInFirstImage = contours.size();
        }

        for (size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
            const vector<Point>& contour = contours[contourIndex];

            Rect roi = defineROI(contour, croppedImage.cols, croppedImage.rows, 25);
            Point2f centroid = calculateCentroid(contour);
            double area = contourArea(contour);

            if (area > 500) {
                ContourData data;
                data.number = uniqueCountInPreviousImage + contourIndex + 1;
                data.roi = roi;
                data.centroid = centroid;
                data.area = area;

                currentContourData.push_back(data);
                currentAreas.push_back(area);
                currentCentroids.push_back(centroid);
                currentNumbers.push_back(data.number);
            }
        }

        changedContourData.push_back(currentContourData);

        for (size_t contourIndex = 0; contourIndex < currentContourData.size(); ++contourIndex) {
            const ContourData& currentData = currentContourData[contourIndex];
            bool isUnique = true;

            Rect topROI(0, 0, croppedImage.cols, 150);
            Rect bottomROI(0, croppedImage.rows - 150, croppedImage.cols, 150);

            for (size_t i = 0; i < prevContourData.size(); ++i) {
                for (size_t j = 0; j < prevContourData[i].size(); ++j) {
                    const ContourData& prevData = prevContourData[i][j];
                    int tolerance = 500;

                    if (areRectanglesApproxEqual(prevData.roi, currentData.roi, tolerance)) {
                        if ((prevData.roi & currentData.roi).width > 0 && prevData.roi.contains(currentData.centroid)) {
                            if (prevData.centroid.y < currentData.centroid.y) {
                                clearNumberAndAdjustCounts(i, j, prevContourData);
                                isUnique = false;
                                break;
                            }
                        }
                    }
                }
                if (!isUnique) break;
            }

            if (isUnique) {
                processUniqueContour(currentData, croppedImage, contours, contourIndex);
                uniqueCount++;
            }
        }

        totalcounter += uniqueCount;

        prevContourData.clear();
        prevContourData = changedContourData;
        changedContourData.clear();

        lock_guard<mutex> lock(buffer4_mutex);
        buffer4.push(croppedImage);
        cout << "Image with contours moved from buffer3 to buffer4" << endl;
        buffer4_cv.notify_one();

        cout << "Total unique in image " << counter_2 << ": " << uniqueCount << endl;
        QString countText = QString::number(totalcounter);
        countEdit->setText(countText);

        QString statusText = QString("Processing Image %1").arg(counter_2);
        statusLabel->setText(statusText);


        cout << "Total counter value: " << totalcounter << " | Frame Count value: " << FrameCountvalue << endl;

        if (totalcounter >= FrameCountvalue)
        {
            QMetaObject::invokeMethod(mainWindow, "stopConveyor", Qt::QueuedConnection);
            cout << "Conveyor stopped: Desired count reached" << endl;
            stopThreads = true;
            stopProcessing = true;
            cout << "Conveyor stopped: Desired count reached" << endl;
            break;
        }
       
    }
}


void saving(const string& outputDirectory)
{
    int count = 0;
    while (!stopProcessing)
    {
        Mat image;
        {
            unique_lock<mutex> lock(buffer4_mutex);
            buffer4_cv.wait(lock, [] { return !buffer4.empty() || stopProcessing; });
            if (stopProcessing && buffer4.empty()) break;

            image = buffer4.front();
            buffer4.pop();
        }
        count++;
        // Save current image data for future comparison
        string imageName = outputDirectory + "/image_" + to_string(count) + ".jpg";
        imwrite(imageName, image);
        cout << "Total:" << totalcounter << endl;
    }
}

*/