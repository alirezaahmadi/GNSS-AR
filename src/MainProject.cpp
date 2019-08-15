/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>

#include <include/TxtExtractor.h>

#include <eigen3/Eigen/Dense>
#include <vector>

#include <cstdio>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace Eigen;


int watershedCluster(Mat& Input_Image, Mat& WEMask);
void addGrid(Mat& img,uint step);
static void onMouse(int event, int x, int y, int flags, void*);
vector<Vec4i> EdgeDetector(Mat& Input_Image, Mat& Output_Image);
std::vector<Point2i> ElevationMap(Mat& src, ofstream& file,vector<Vec4i> Lines, double field_of_view);
bool addSatellite(Mat& img, vector<cv::Point2d> SatPosition, double SatPrn, vector<Point2i> boarders, Vector3d color);
std::vector<cv::Point3d> getSatPos3D(uint StartPos_index, uint EndPos_index,uint SatNum);
void addCircle(Mat& img, cv::Point cp, int radius, Vector3d color,int thickness);

float Scale = 0.3;

#define CAMERA_ROLL_OFFSET    180
#define CAMERA_PAN_OFFSET     0   // with respect to magnetic north not true north
#define CAMERA_TILTE_OFFSET   255 

ofstream file;

vector<Vec4i> DefLine;

Mat markerMask, Image;
Point prevPt(-1, -1);
vector<vector<int> > boarders;

vector<Vec4i> MaskLines;

std::vector<Point2i> WM_boarders;

int SatInImage =0;
bool SatInPic_b = false;

int img_width,img_height;

// typedef Matrix<long double, 3, 3> Matrix3ld;
// Matrix3ld a ;
// typedef Matrix<long double, Dynamic, Dynamic> MatrixXld;

int main(int argc, char** argv) {

  file.open ("../ElevationMap.txt");

  double roll = CAMERA_ROLL_OFFSET * M_PI / 180;
  double pitch = CAMERA_TILTE_OFFSET * M_PI / 180;
  double yaw = CAMERA_PAN_OFFSET * M_PI / 180;
  

  // 50.727246, 7.086934
  double c = -2696.13738 * Scale;
  double m = 0.995130917 * Scale;
  double xh = -18.2157641 * Scale;
  double yh = 10.5762745 * Scale;

  cv::Mat intrisicMat(3, 3, cv::DataType<double>::type);  // Intrisic matrix
  intrisicMat.at<double>(0, 0) = c;
  intrisicMat.at<double>(1, 0) = 0;
  intrisicMat.at<double>(2, 0) = 0;

  intrisicMat.at<double>(0, 1) = 0;
  intrisicMat.at<double>(1, 1) = c ;
  intrisicMat.at<double>(2, 1) = 0;

  intrisicMat.at<double>(0, 2) = xh;
  intrisicMat.at<double>(1, 2) = yh;
  intrisicMat.at<double>(2, 2) = 1;

  cv::Mat rVec(3, 1, cv::DataType<double>::type);  // Rotation vector
  rVec.at<double>(0) = roll;
  rVec.at<double>(1) = pitch;
  rVec.at<double>(2) = yaw;

  cv::Mat rVecR;
  cv::Rodrigues(rVec, rVecR); 

  cv::Mat tVec(3, 1, cv::DataType<double>::type);  // Translation vector
  tVec.at<double>(0) = 4014758.6504083;
  tVec.at<double>(1) = 498948.480408127;
  tVec.at<double>(2) = 4914331.43590426;

  cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);  // Distortion vector
  distCoeffs.at<double>(0) = 0;
  distCoeffs.at<double>(1) = 0;
  distCoeffs.at<double>(2) = 0;
  distCoeffs.at<double>(3) = 0;
  distCoeffs.at<double>(4) = 0;

  std::cout << "Intrisic matrix: " << intrisicMat << std::endl << std::endl;
  std::cout << "Rotation vector: " << rVec << std::endl << std::endl;
  std::cout << "Translation vector: " << tVec << std::endl << std::endl;
  std::cout << "Distortion coef: " << distCoeffs << std::endl << std::endl;

  cout.precision(numeric_limits<double>::digits10 + 1);
  string filename;
  Mat Input_Image,src,WEdgesMask;
  TxtExtr Extractor;

  std::vector<int> SatNumInPic;

  // vector<vector<double>> GPS_time = Extractor.TxTtoVector("../data/GPS_time.txt"); 
  // vector<vector<double>> GPS_Az = Extractor.TxTtoVector("../data/GPS_Az.txt");
  // vector<vector<double>> GPS_El = Extractor.TxTtoVector("../data/GPS_El.txt");
  vector< vector<double> > GPS_Prn = Extractor.TxTtoVector("../data/GPS_Prn.txt");

  if (atoi(argv[1]) == 1 || atoi(argv[1]) == 2 || atoi(argv[1]) == 3 ) {
    filename = argc == 3 ? argv[2] : "../img/DSC_0594.JPG";
    Input_Image = imread(filename, 1);
    if (Input_Image.empty()) {
      cout << "Image is Empty !!!" << endl;
      return -1;
    }
    cv::resize(Input_Image, src, cv::Size(), Scale, Scale);
    img_width = src.cols;
    img_height = src.rows;
  }

  DefLine.resize(2,0);
  DefLine[0][0] = 0;
  DefLine[0][1] = 0;
  DefLine[0][2] = img_width;
  DefLine[0][3] = 0;

  DefLine[1][0] = 0;
  DefLine[1][1] = img_height;
  DefLine[1][2] = img_width;
  DefLine[1][3] = img_height;

  if (atoi(argv[1]) == 1) {

    Mat cdst;
    vector<Vec4i> RoofEdge = EdgeDetector(src, cdst);

    Vector3d RGB;
    RGB << 255, 0, 0;
    for (int i = 0; i < 1; i++) {
      std::vector<cv::Point3d> SatPos3D = getSatPos3D(650, 651, 20);

      std::vector<cv::Point2d> SatPos2D;
      cv::projectPoints(SatPos3D, rVecR, tVec, intrisicMat, distCoeffs, SatPos2D);

      RGB(1) += 5;
      RGB(2) += 10;
      addSatellite(cdst, SatPos2D, GPS_Prn[0][0],WM_boarders, RGB);
    }
    RGB << 30, 230, 125;
    addGrid(cdst,50);
    imshow("detected lines", cdst);
  } else if (atoi(argv[1]) == 2) {

    Mat final_markers;
    watershedCluster(src,WEdgesMask);
    cout << "done !!!" << endl;
  } else if (atoi(argv[1]) == 3) {
    Mat cdst,WatershedEdgesMask;
    MaskLines = EdgeDetector(src, cdst);

    for(int i=0; i<2;i++){
      cout << MaskLines[i] << endl;
    }

    imshow("detected lines", cdst);

    Mat final_markers;
    DefLine[1][0] = 0;
    DefLine[1][1] = (MaskLines[0][1] + MaskLines[1][1])/2;
    DefLine[1][2] = img_width;
    DefLine[1][3] = (MaskLines[0][3] + MaskLines[1][3])/2;
    watershedCluster(src, WEdgesMask);

    WM_boarders = ElevationMap(WEdgesMask, file, MaskLines, 65);


    Mat LMask(src.rows, src.cols, CV_8UC1, Scalar(0,0, 100));
    Mat WLMask(src.rows, src.cols, CV_8UC1, Scalar(0,0, 100));

    line(LMask, Point(MaskLines[0][0], MaskLines[0][1]), Point(MaskLines[0][2], MaskLines[0][3]), 255, 3,
           CV_AA);
    line(LMask, Point(MaskLines[1][0], MaskLines[1][1]), Point(MaskLines[1][2], MaskLines[1][3]), 255, 3,
           CV_AA);

    imshow("WM",WEdgesMask);
    imshow("LM",LMask);
    for (int i = 0; i < src.rows; i++)
    {
      for (int j = 0; j < src.cols; j++)
      {
        WLMask.at<uint8_t>(i,j)= (uint8_t)(WEdgesMask.at<uint8_t>(i,j) + LMask.at<uint8_t>(i,j));
      }
    }
    imshow("WLM",WLMask);


    Mat WSresult;
    cvtColor(WLMask,WSresult,CV_GRAY2BGR,3);;

    Vector3d RGB;
    RGB << 255, 0, 0;
    int start_line = 0;
    int End_line = start_line + 10000;
    for (int i = 0; i < 24 ; i++) {
      std::vector<cv::Point3d> SatPos3D = getSatPos3D(start_line, End_line, i);
      std::vector<cv::Point2d> SatPos2D;
      cv::projectPoints(SatPos3D, rVecR, tVec, intrisicMat, distCoeffs, SatPos2D);

      RGB(0) = theRNG().uniform(0, 255);
      RGB(1) = theRNG().uniform(0, 255);
      RGB(2) = theRNG().uniform(0, 255);
      cout << i << endl;

      for (int j = start_line; j < End_line; j++) {
          SatInPic_b = addSatellite(src, SatPos2D, GPS_Prn[start_line + j][i], WM_boarders, RGB);     
      }
      if(SatInPic_b){
        SatInImage++;
      } 
    }

    imshow("Block",src);
    cout << "Satellites in this image: " << SatInImage << "  " << endl;

    // ******** Mask Combination done *************************
    
    cout << "done !!!" << endl;
  } else if (atoi(argv[1]) == 4) {
    // VideoCapture cap(0);  // open the default camera
    // if (!cap.isOpened()) {
    //   cout << "Cant open the camera!!" << endl;  // check if we succeeded
    //   return -1;
    // }
    // cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    // cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    // namedWindow("edges", 1);
    // for (;;) {
    //   Mat frame;
    //   Mat edges;
    //   Mat dst, cdst, blur;
    //   cap >> frame;  // get a new frame from camera
    //   GaussianBlur(frame, blur, Size(5, 5), 0, 0);
    //   int lowThreshold = 0;
    //   Canny(blur, dst, lowThreshold, lowThreshold * 3, 3);
    //   cvtColor(dst, edges, CV_GRAY2BGR);
    //   // imshow("cvtColor", dst);
    //   // imshow("edges", edges);
    //   // Create the images that will use to extract the horizontal and
    //   // vertical lines
    //   Mat horizontal = dst.clone();
    //   Mat vertical = dst.clone();
    //   // Specify size on horizontal axis
    //   int horizontalsize = horizontal.cols / 100;
    //   // Create structure element for extracting horizontal lines through
    //   // morphology
    //   // operations
    //   Mat horizontalStructure =
    //       getStructuringElement(MORPH_RECT, Size(horizontalsize, 1));
    //   // Apply morphology operations
    //   erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
    //   dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1));
    //   // Show extracted horizontal lines
    //   imshow("horizontal", horizontal);
    //   Vec4i l;
    //   HoughLinesP(horizontal, lines, 1, CV_PI / 180, 100, 100, 150);
    //   l = lines[0];
    //   // cout << lines[0] << endl;
    //   for (size_t i = 1; i < lines.size(); i++) {
    //     if (l[1] > lines[i][1]) {
    //       l = lines[i + 1];
    //     }
    //     // cout << lines[i] << endl;
    //     // cout << lines[i][0] << endl;
    //     // line(edges, Point(lines[i][0], lines[i][1]), Point(lines[i][2],
    //     // lines[i][3]),
    //     //     Scalar(255, 0, 255), 2, CV_AA);
    //   }
    //   line(edges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 255),
    //        3, CV_AA);
    //   cout << "Roof's edge-line: " << l << endl;
    //   line(edges, Point(edges.cols, edges.rows), Point(0, edges.rows),
    //        Scalar(0, 0, 255), 3, CV_AA);
    //   // imshow("source", frame);
    //   imshow("detectedlines", edges);
    //   if (waitKey(30) >= 0) break;
    // }
  }
  waitKey(0);
  file.close();
  return 0;
}

std::vector<Point2i> ElevationMap(Mat& src, ofstream& file,vector<Vec4i> Lines, double field_of_view = 45) {
  // vertical line in center of the 2image  
  int width = img_width/2;
  // in case of using central line az horizon 
  int height = Lines[1][3];

  std::vector<Point2i> boarders;
  boarders.resize(img_width);

  for (int i = 0; i < img_width; i++){
    for (int j = 0; j < img_height; j++){
      if((int)src.at<uint8_t>(j,i) > 0){
        //cout  << i << " : " << j << endl;
        boarders[i] = Point2i(i,j);
      }
    }
  }

  // for (int i = 0; i < img_width; ++i)
  // {
  //   cout << boarders[i] << endl;
  // }

  float Hyp = sqrt(width*width + height*height);
  float angle_per_pixel = field_of_view/Hyp;

  for (int i = 0; i < img_width; ++i){
    float Elv_x = -width + boarders[i].x;
    float Elv_y = height - boarders[i].y;
    //cout << i << "  EL-y: "<< Elv_x  << "  y: " << boarders[i].x<< endl;
    double Angle_x = Elv_x * angle_per_pixel;
    double Angle_y = Elv_y * angle_per_pixel;  // correct it !!!!!! assumption - > image is square
    if(i==0) file << "PixNum  Edges.x  Edges.y  Long  Lat" << endl;
    file << i << "  " << boarders[i].x << "  " << boarders[i].y << "  " << 
         Angle_x << "  " << Angle_y << endl;
  }
  cout << endl;
  file.close();
  return boarders;
} 

void addGrid(Mat& img,uint step = 65) {

  int width = img.size().width;
  int height = img.size().height;

  for (int i = 0; i<height; i += step) 
      cv::line(img, Point(0, i), Point(width, i), cv::Scalar(30, 230, 125));

  for (int i = 0; i<width; i += step)
      cv::line(img, Point(i, 0), Point(i, height), cv::Scalar(30, 230, 125));
}

void addCircle(Mat& img, cv::Point cp, int radius, Vector3d color,int thickness = 1) {
  std::string win_name = "circle";
  cv::Scalar black((uint8_t)color(0), (uint8_t)color(1), (uint8_t)color(2));
  cv::circle(img, cp, radius, black, thickness);
}

std::vector<cv::Point3d> getSatPos3D(uint StartPos_index, uint EndPos_index,uint SatNum) {
  TxtExtr Extractor;
  vector<vector<double>> GPS_x = Extractor.TxTtoVector("../data/GPS_x.txt");
  vector<vector<double>> GPS_y = Extractor.TxTtoVector("../data/GPS_y.txt");
  vector<vector<double>> GPS_z = Extractor.TxTtoVector("../data/GPS_z.txt");

  //cout << "Data x : "<<GPS_x.size() << endl;

  std::vector<cv::Point3d> points;

  // if (EndPos_index==0)
  // {
  //   EndPos_index = (GPS_x[0].size())-2;
  // }

  for (uint i = StartPos_index; i < EndPos_index; ++i) {
    points.push_back(
        cv::Point3d(GPS_x[i][SatNum], GPS_y[i][SatNum], GPS_z[i][SatNum]));
  }
  return points;
}

bool addSatellite(Mat& img, vector<cv::Point2d> SatPosition, double SatPrn, vector<Point2i> boarders,Vector3d color) {
  bool SatinImage_bool = false;
  for (size_t i = 0; i < SatPosition.size() ; i++) {
    if (SatPosition[i].x < img.cols && SatPosition[i].y < img.rows &&
        SatPosition[i].x >= 0 && SatPosition[i].y >= 0) {

      if(boarders[SatPosition[i].x].y < SatPosition[i].y){
        color << 0, 0, 255;
        //cout << "here....  " << boarders[SatPosition[i].x].y << "  "<< SatPosition[i].y << "  " << SatPosition[i].x <<  endl;
      }
      //cout << "SatPrn"
      //     << " " << SatPosition[i].x << ", " << SatPosition[i].y << endl;
      addCircle(img, Point(SatPosition[i].x, SatPosition[i].y), 2, color, 1);
      if ((boarders[SatPosition[i].x].y < 1 +  SatPosition[i].y && boarders[SatPosition[i].x].y >  SatPosition[i].y -1) || i == 0) {
        cv::putText(img, 
                "Prn:"+to_string((int)SatPrn),
                Point(SatPosition[i].x, SatPosition[i].y), // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                0.5, // Scale. 2.0 = 2x bigger
                cv::Scalar(theRNG().uniform(0, 255),theRNG().uniform(0, 255),theRNG().uniform(0, 255)), // BGR Color
                1, // Line Thickness (Optional)
                81); // Anti-alias (Optional)
      }
      SatinImage_bool = true;
    } 
  }
  return SatinImage_bool;
}

vector<Vec4i> EdgeDetector(Mat& Input_Image, Mat& Output_Image) {
  vector<Vec4i> lines;
  cout << "Cols: " << Input_Image.cols << ", Rows: " << Input_Image.rows  << " channelNum: " << Input_Image.channels()<< endl;

  Mat dst, cdst, blur,gray;

  cvtColor(Input_Image, gray, CV_BGR2GRAY);
  //imshow("gray", gray);
  GaussianBlur(Input_Image, blur, Size(5, 5), 0, 0);
  //imshow("blur", blur);
  Canny(blur, dst, 50, 0, 3);
  //imshow("canny", dst);
  cvtColor(dst, cdst, CV_GRAY2BGR);
  
  // imshow("canny", dst);
  // Create the images that will use to extract the horizontal and vertical
  // lines
  Mat horizontal = dst.clone();
  Mat vertical = dst.clone();
  // Specify size on horizontal axis
  int horizontalsize = horizontal.cols / 100;
  // Create structure element for extracting horizontal lines through
  // morphology
  // operations
  Mat horizontalStructure =
      getStructuringElement(MORPH_RECT, Size(horizontalsize, 1));
  // Apply morphology operations
  erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
  //imshow("erode", horizontal);
  dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1));
  //imshow("dilate", horizontal);
  // Show extracted horizontal lines
  // imshow("horizontal", horizontal);
  vector<Vec4i> MaskLines;
  MaskLines.resize(2,0);
  HoughLinesP(horizontal, lines, 1, CV_PI / 180, 100, 100, 150);
  MaskLines[0] = lines[0];
  MaskLines[1] = lines[0];
  for (size_t i = 1; i < lines.size(); i++){
    if (MaskLines[0][1] > lines[i][1]) {
      MaskLines[0] = lines[i];
    }
    if (MaskLines[1][1] < lines[i][1]) {
      MaskLines[1] = lines[i];
    }
    cout << lines[i] << endl;
    line(cdst, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]),
         Scalar(255, 0, 255), 2, CV_AA);
    if (lines.size()-1 == i) {
      line(cdst, Point(MaskLines[0][0], MaskLines[0][1]), Point(MaskLines[0][2], MaskLines[0][3]), Scalar(0, 255, 255), 3,
           CV_AA);
      line(cdst, Point(MaskLines[1][0], MaskLines[1][1]), Point(MaskLines[1][2], MaskLines[1][3]), Scalar(250, 100, 100), 3,
           CV_AA);
      cout << "Top Edge line: " << MaskLines[0] << endl;
      cout << "Horizon line: " << MaskLines[1] << endl;
    }
  }
  Output_Image = cdst;
  return MaskLines;
}

static void onMouse(int event, int x, int y, int flags, void*) {
  if (x < 0 || x >= Image.cols || y < 0 || y >= Image.rows) return;
  if (event == EVENT_LBUTTONUP || !(flags & EVENT_FLAG_LBUTTON))
    prevPt = Point(-1, -1);
  else if (event == EVENT_LBUTTONDOWN)
    prevPt = Point(x, y);
  else if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON)) {
    Point pt(x, y);
    if (prevPt.x < 0) prevPt = pt;
    line(markerMask, prevPt, pt, Scalar::all(255), 5, 8, 0);
    line(Image, prevPt, pt, Scalar::all(255), 5, 8, 0);
    prevPt = pt;
    imshow("image", Image);

  }
  line(markerMask, Point(DefLine[0][0], DefLine[0][1]), Point(DefLine[0][2], DefLine[0][3]), Scalar::all(255), 5, 8, 0);
  line(markerMask, Point(DefLine[1][0], DefLine[1][1]), Point(DefLine[1][2], DefLine[1][3]), Scalar::all(255), 5, 8, 0);
  line(Image, Point(DefLine[0][0], DefLine[0][1]), Point(DefLine[0][2], DefLine[0][3]), Scalar::all(255), 5, 8, 0);
  line(Image, Point(DefLine[1][0], DefLine[1][1]), Point(DefLine[1][2], DefLine[1][3]), Scalar::all(255), 5, 8, 0);
  imshow("image", Image);
}

int watershedCluster(Mat& Input_Image, Mat& WEMask) {
  Mat imgGray;

  cout << "Cols: " << Input_Image.cols << ", Rows: " << Input_Image.rows << endl;
  if( Input_Image.empty() )
  {
      cout << "Couldn'g open image " << ". Usage: watershed <image_name>\n";
      return 0;
  }
  namedWindow( "image", 1 );

  Input_Image.copyTo(Image);

  cvtColor(Image, markerMask, COLOR_BGR2GRAY);
  cvtColor(markerMask, imgGray, COLOR_GRAY2BGR);
  markerMask = Scalar::all(0);
  imshow("image", Image);
  setMouseCallback("image", onMouse, 0);

  for (;;){
    char c = (char)waitKey(0);

    if (c == 27) break;

    if (c == 'r') {
      markerMask = Scalar::all(0);
      Input_Image.copyTo(Image);
      imshow("image", Image);
    }

    if (c == 'w' || c == ' ') {
      int i, j, compCount = 0;
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;

      findContours(markerMask, contours, hierarchy, RETR_CCOMP,
                   CHAIN_APPROX_SIMPLE);

      if (contours.empty()) continue;
      Mat markers(markerMask.size(), CV_32S);
      markers = Scalar::all(0);
      
      for (int idx = 0; idx >= 0; idx = hierarchy[idx][0], compCount++)
        drawContours(markers, contours, idx, Scalar::all(compCount + 1), -1, 8,
                     hierarchy, INT_MAX);

      if (compCount == 0) continue;

      vector<Vec3b> colorTab;
      for (i = 0; i < compCount; i++) {
        int b = theRNG().uniform(0, 255);
        int g = theRNG().uniform(0, 255);
        int r = theRNG().uniform(0, 255);

        colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
      }

      double t = (double)getTickCount();
      watershed(Input_Image, markers);
      t = (double)getTickCount() - t;
      printf("execution time = %gms\n", t * 1000. / getTickFrequency());

      Mat wshed(markers.size(), CV_8UC3);
      Mat WMask(Input_Image.rows, Input_Image.cols, CV_8UC1, Scalar(0,0, 100));

      // paint the watershed image
      for (i = 0; i < markers.rows; i++)
        for (j = 0; j < markers.cols; j++) {
          int index = markers.at<int>(i, j);
          if (index == -1){
            wshed.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
            if(i != markers.rows-1 && j != markers.cols -1 && i !=0 && j !=0 )
            WMask.at<uint8_t>(i,j) = 255;
          }
          else if (index <= 0 || index > compCount){
            wshed.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
            WMask.at<uint8_t>(i,j) = 0;
          }else{
            wshed.at<Vec3b>(i, j) = colorTab[index - 1];
            WMask.at<uint8_t>(i,j) = 0;
          }
        }
      //imshow("test", WMask);
      WMask.copyTo(WEMask);
      wshed = wshed * 0.5 + imgGray * 0.5;
      addGrid(wshed,65) ;
      imshow("watershed transform", wshed);
      break;
    }
  }
}
