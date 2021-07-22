#include <opencv2/viz/vizcore.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <vector>

using namespace cv;
using namespace std;
using namespace viz;

void AddCamera(viz::Viz3d window, String CamName, Point3d shiftP, Vec3f rotateP, bool enable)
{
    double x = rotateP[0];
    double y = rotateP[1];
    double z = rotateP[2];

    Point3d pf(cos(y) * cos(z), cos(y) * sin(z), -sin(y));
    y += CV_PI / 2.0;
    Point3d pd(cos(y) * cos(z), cos(y) * sin(z), -sin(y));

    Affine3f cam_pose = viz::makeCameraPose(shiftP, shiftP + pf, pd);

    if (enable)
    {
        window.setViewerPose(cam_pose);
    }
    else
    {
        WArrow forward(shiftP, shiftP + pf, 0.008, viz::Color::red());
        window.showWidget(CamName + "Forward", forward);
        WArrow downward(shiftP, shiftP + pd, 0.008, viz::Color::blue());
        window.showWidget(CamName + "Downward", downward);

        viz::WCameraPosition camFrame(Vec2f(1, 1), 0.5);
        window.showWidget(CamName + "Frame", camFrame, cam_pose);
        viz::WCameraPosition camAxis(0.5);
        window.showWidget(CamName + "Axis", camAxis, cam_pose);
    }
}

void inintVizs(vector<Viz3d> windows)
{
    for (int i = 0; i < windows.size(); ++i)
    {
        windows[i].setBackgroundColor(Color::black());

        //        WLine xaxis(Point3f(0,0,0),Point3f(2,0,0),Color::red());
        //        xaxis.setRenderingProperty(LINE_WIDTH,5);
        //        windows[i].showWidget("xaxis", xaxis);

        //        WLine yaxis(Point3f(0,0,0),Point3f(0,2,0),Color::green());
        //        yaxis.setRenderingProperty(LINE_WIDTH,5);
        //        windows[i].showWidget("yaxis", yaxis);

        //        WLine zaxis(Point3f(0,0,0),Point3f(0,0,2),Color::blue());
        //        zaxis.setRenderingProperty(LINE_WIDTH,5);
        //        windows[i].showWidget("zaxis", zaxis);

        WGrid grid(Vec2i::all(50), Vec2d::all(1), Color::white());
        windows[i].showWidget("grid", grid);

        //---------------------------------------------

        //        WLine l1(Point3f(0,-5,0),Point3f(0,5,0),Color::green());
        //        l1.setRenderingProperty(LINE_WIDTH,2);
        //        windows[i].showWidget("l1", l1);

        WLine l3(Point3f(10, -5, 0), Point3f(10, 5, 0), Color::red());
        l3.setRenderingProperty(LINE_WIDTH, 2);
        windows[i].showWidget("l3", l3);

        WLine l2(Point3f(0, -5, 0), Point3f(10, -5, 0), Color::green());
        l2.setRenderingProperty(LINE_WIDTH, 2);
        windows[i].showWidget("l2", l2);

        WLine l4(Point3f(0, 5, 0), Point3f(10, 5, 0), Color::green());
        l4.setRenderingProperty(LINE_WIDTH, 2);
        windows[i].showWidget("l4", l4);
    }
}

void updateVizsOnce(vector<Viz3d> windows)
{
    for (int i = 0; i < windows.size(); ++i)
    {
        windows[i].spinOnce();
    }
}
int i = 0;
void addLine(Viz3d Gwin, String Name, Point3d a, Point3d b, Color colo)
{
    WLine l3(a, b, colo);
    l3.setRenderingProperty(LINE_WIDTH, 2);
    Gwin.showWidget(Name, l3);
}

void alineR(Viz3d Gwin, Mat pic, Point3d pos)
{
    Mat imgHSV;
    cvtColor(pic, imgHSV, COLOR_BGR2HSV);

    Mat imTH;
    inRange(imgHSV, Scalar(0, 100, 100), Scalar(1, 255, 255), imTH);
    cv::imshow("Thresholded ImageR", imTH);

    int x = 0;
    int t = 0;

    for (int i = 0; i < imTH.rows; i++)
    {
        for (int j = 0; j < imTH.cols; ++j)
        {
            if (imTH.ptr<uchar>(i)[j] == 255)
            {
                x += j;
                ++t;
            }
        }
    }
    double Abig = atan2(10.0 - pos.x, -pos.y - 5);
    double Asmall = atan2(-pos.x, 5.0 - pos.y);

    double Amiddle = Abig - Asmall;

    double ang = (double)x / (double)t;

    double rang = (CV_PI - (Asmall + Amiddle / imTH.cols * ang));
    double ta = tan(rang);

    double b = pos.x + ta * pos.y;

    addLine(Gwin, "rr", pos, Point3d(ta * -20 + b, 20, 0), Color::yellow());

    waitKey(1);
}

void alineL(Viz3d Gwin, Mat pic, Point3d pos)
{
    Mat imgHSV;
    cvtColor(pic, imgHSV, COLOR_BGR2HSV);

    Mat imTH;
    inRange(imgHSV, Scalar(0, 100, 100), Scalar(1, 255, 255), imTH);
    cv::imshow("Thresholded ImageL", imTH);

    int x = 0;
    int t = 0;

    for (int i = 0; i < imTH.rows; i++)
    {
        for (int j = 0; j < imTH.cols; ++j)
        {
            if (imTH.ptr<uchar>(i)[j] == 255)
            {
                x += j;
                ++t;
            }
        }
    }

    double Abig = atan2(10.0 - pos.x, pos.y - 5);
    double Asmall = atan2(-pos.x, 5.0 + pos.y);

    double Amiddle = Abig - Asmall;

    double ang = (double)x / (double)t;

    double rang = (Asmall + Amiddle / imTH.cols * (imTH.cols - ang));
    double ta = tan(rang);

    double b = pos.x + ta * pos.y;

    addLine(Gwin, "ll", pos, Point3d(ta * 20 + b, -20, 0), Color::blue());

    waitKey(1);
}

int main()
{
    vector<Viz3d> Wins;

    Viz3d Gwin("Gwin");
    Wins.push_back(Gwin);
    Viz3d Lwin("Lwin");
    Wins.push_back(Lwin);
    Viz3d Rwin("Rwin");
    Wins.push_back(Rwin);

    inintVizs(Wins);

    Point3d PositionCamL = Point3d(-6.67, 11.67, 0);
    Point3d PositionCamR = Point3d(-6.67, -11.67, 0);

    AddCamera(Gwin, "lcam", PositionCamL, Vec3f(0, 0, CV_PI / 180.0 * -45.0), false);
    AddCamera(Gwin, "rcam", PositionCamR, Vec3f(0, 0, CV_PI / 180.0 * 45.0), false);

    AddCamera(Lwin, "Lwin", PositionCamL, Vec3f(0, 0, CV_PI / 180.0 * -45.0), true);
    AddCamera(Rwin, "Rwin", PositionCamR, Vec3f(0, 0, CV_PI / 180.0 * 45.0), true);

    while (true)
    {
        for (double i = 0; i < 10; i += 0.1)
        {
            WCylinder cylinder(Point3d(i, -5, 0), Point3d(i, -5, 1), 0.1, 30, Color::red());
            Gwin.showWidget("cylinder", cylinder);
            Lwin.showWidget("cylinder", cylinder);
            Rwin.showWidget("cylinder", cylinder);

            addLine(Gwin, "l", PositionCamL, Point3d(i, -5, 0), Color::green());
            addLine(Gwin, "r", PositionCamR, Point3d(i, -5, 0), Color::green());

            Mat picr = Rwin.getScreenshot();
            alineR(Gwin, picr, PositionCamR);
            Mat picl = Lwin.getScreenshot();
            alineL(Gwin, picl, PositionCamL);

            updateVizsOnce(Wins);
        }

        for (double i = -5; i < 5; i += 0.1)
        {
            WCylinder cylinder(Point3d(10, i, 0), Point3d(10, i, 1), 0.1, 30, Color::red());
            Gwin.showWidget("cylinder", cylinder);
            Lwin.showWidget("cylinder", cylinder);
            Rwin.showWidget("cylinder", cylinder);

            addLine(Gwin, "l", PositionCamL, Point3d(10, i, 0), Color::green());
            addLine(Gwin, "r", PositionCamR, Point3d(10, i, 0), Color::green());

            Mat picr = Rwin.getScreenshot();
            alineR(Gwin, picr, PositionCamR);
            Mat picl = Lwin.getScreenshot();
            alineL(Gwin, picl, PositionCamL);

            updateVizsOnce(Wins);
        }

        for (double i = 10; i > 0; i -= 0.1)
        {
            WCylinder cylinder(Point3d(i, 5, 0), Point3d(i, 5, 1), 0.1, 30, Color::red());
            Gwin.showWidget("cylinder", cylinder);
            Lwin.showWidget("cylinder", cylinder);
            Rwin.showWidget("cylinder", cylinder);

            addLine(Gwin, "l", PositionCamL, Point3d(i, 5, 0), Color::green());
            addLine(Gwin, "r", PositionCamR, Point3d(i, 5, 0), Color::green());

            Mat picr = Rwin.getScreenshot();
            alineR(Gwin, picr, PositionCamR);
            Mat picl = Lwin.getScreenshot();
            alineL(Gwin, picl, PositionCamL);

            updateVizsOnce(Wins);
        }

        for (double i = 5; i > -5; i -= 0.1)
        {
            WCylinder cylinder(Point3d(0, i, 0), Point3d(0, i, 1), 0.1, 30, Color::red());
            Gwin.showWidget("cylinder", cylinder);
            Lwin.showWidget("cylinder", cylinder);
            Rwin.showWidget("cylinder", cylinder);

            addLine(Gwin, "l", PositionCamL, Point3d(0, i, 0), Color::green());
            addLine(Gwin, "r", PositionCamR, Point3d(0, i, 0), Color::green());

            Mat picr = Rwin.getScreenshot();
            alineR(Gwin, picr, PositionCamR);
            Mat picl = Lwin.getScreenshot();
            alineL(Gwin, picl, PositionCamL);

            updateVizsOnce(Wins);
        }
    }
}
