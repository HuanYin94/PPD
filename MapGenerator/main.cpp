#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    CV_Assert(argc == 2);

    Mat image = imread(argv[1]);
    CV_Assert(image.empty() == false);
    imshow("image", image);

    Mat gray;
    cvtColor(image, gray, CV_BGR2GRAY);

    Mat canny;
    Canny(gray, canny, 100, 200, 3);
    imshow("canny", canny);

    Mat inv;
    inv = 255 - gray;

    vector<vector<Point> > contours;
    findContours(inv, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    ofstream ofs;
    ofs.open("tmp.map");
    CV_Assert (ofs.is_open() == true);

    ofs << "      " << endl;
    vector<vector<Point> > polygons;
    for (int c = 0; c < contours.size(); ++c)
    {
        vector<Point> polygon;
#if 1
        approxPolyDP(contours[c], polygon, 3, true);
#else
        polygon = contours[c];
#endif
        if (polygon.size() > 2)
        {
            ofs << polygon.size() << " ";
            for (int p = 0; p < polygon.size(); ++p)
                ofs << polygon[p].x << " " << polygon[p].y << " ";
            ofs << endl;
            polygons.push_back(polygon);
        }
    }
    ofs << endl;
    ofs.seekp(0);
    ofs << polygons.size();
    ofs.close();

    drawContours(image, polygons, -1, Scalar(255, 0, 0));
    imshow("polygons", image);

    waitKey();

    return 0;
}

