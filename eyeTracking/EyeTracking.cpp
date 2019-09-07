// openTest.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.

// Eye Tracker based on Starburst Algorithm

// implement by Minhyuk, Kwon



#include "stdafx.h"

#include "StarBurst.h"

#include "Ransac.h"

#include <time.h>

#include <windows.h>

#include <conio.h>

#include <sys/timeb.h>

#include "opencv2/core/traits.hpp"

#include "opencv2/core.hpp"



using namespace cv;



HANDLE hfMemMap;

static LPSTR lpAddress1, lpAddress2;



struct timeb timebuffer;

struct tm *now;

time_t ltime;

int milisec;



Mat orignal_image;         // Original Image

Mat gray_image;         // Gray Image for Processing

Mat cr_image;            // Only Corneal Reflection Image without Background

Mat pupil_image;         // Only Pupil Image without Background

Mat overay_image;      // Merge All of Images of Result by Starburst (CR & Pupil Location)

Mat empty_image;      //칼리브 좌표 확인용



vector<Point2f> eye_point;

vector<Point2f> screen_point;



int threshold = 0;

Point pupil = { 0,0 };            // Pupil Center

Point start_point = { 0,0 };   // Starting Point of Reflex

Point end_point = { 0,0 };

int flag = 0;



/**

/* Window Title

/* open CV Handle with window title

 */

const char * calib_original_wnd = "칼리브용 뷰";

const char * original_wnd = "Original Image View";

const char * cr_wnd = "Corneal Reflection View";

const char * pupil_wnd = "Pupil View";

const char * result_wnd = "Result View";

const char * calib = "칼리브 후";



/**

/* Functions

 */

void Draw_Cross(Mat image, int centerx, int centery, int x_cross_length, int y_cross_length, Scalar color);

void init();

void refreshViews();

void unSetGUI();

void setGUI();



//여기서 부터 변수 추가

StarBurst *starburst_calib;

StarBurstInfo *info_calib;



Mat lambda;

Point2f screen;

Point2f myEye;

Mat scene_image;

Mat scene_draw_image;

Mat calib_orignal_image;  //칼리브레이션용 원본

Mat calib_gray_image;  //칼리브레이션용 그레이 이미지

//Mat homo_image = imread("C:/Users/Admin/source/repos/capstone_final/calibrate.png", 1);

Mat homo_image = imread("C:/Users/USER$/Desktop/capstone/calibrate.png", 1);

const char* homo_wnd = "Homography Window";



#define CALIBRATIONPOINTS    9

int number_calibration_points_set = 0;

int view_cal_points = 1; // 보정중일때의 flag

int do_map2scene = 0; // 캘리브레이션 완료됬을때 flag



int nMapWrite(LPSTR lpStr, int x, int y)  //mmf를 이용한 데이터 저장
{
	char szData[1024] = "\n";
	if (!hfMemMap)

		CloseHandle(hfMemMap);

	hfMemMap = CreateFileMapping((HANDLE)-1, NULL, PAGE_READWRITE, 0, 1024, L"MemoryMapTest");



	if (hfMemMap == NULL)

		return -1;

	if (GetLastError() == ERROR_ALREADY_EXISTS)

		printf("이미 매핑 오브젝트가 있어요.\n");



	lpStr = (LPSTR)MapViewOfFile(hfMemMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);



	if (lpStr == NULL)

		return -2;



	printf("쓰기 ==%d %d %d초 %d\n", x, y, now->tm_sec, milisec);

	//sprintf(szData, "%d %d %d %d\n", x, y, now->tm_sec, milisec);

	sprintf(szData, "%d", x * 1000 + y);

	//szData[0] = x;

	//szData[1] = y;

	//gets_s(szData);

	strcpy(lpStr, szData);

	strcat(lpStr, "\n");

	UnmapViewOfFile(lpStr);

	printf("쓰기 끝\n");

	return 0;

}



void calib_setGUI() {

	namedWindow(calib_original_wnd, 1);

	namedWindow(homo_wnd, 1);

	Size size = calib_orignal_image.size();





}

void calib_unSetGUI() {

	calib_gray_image.release();

	calib_orignal_image.release();

	//homo_image.release();

	//destroyAllWindows();

}



void unSetGUI() {

	//cvDestroyAllWindows();

	//cvReleaseImage(&orignal_image);

	orignal_image.release();

	//cvReleaseImage(&gray_image);

	gray_image.release();

	cr_image.release();

	pupil_image.release();

	//cvReleaseImage(&overay_image);

	overay_image.release();

}



void refreshViews() {

	// Refresh All Views

	// especially overay_image must erase

	//cvShowImage(original_wnd, orignal_image);

	imshow(original_wnd, orignal_image);

	//cvShowImage(cr_wnd, cr_image);

	imshow(cr_wnd, cr_image);

	//cvShowImage(pupil_wnd, pupil_image);

	imshow(pupil_wnd, pupil_image);

	//cvShowImage(result_wnd, overay_image);

	imshow(result_wnd, overay_image);

	imshow(calib, empty_image);

}



void calib_refreshViews() {

	imshow(calib_original_wnd, calib_orignal_image);

}



void Draw_Cross(Mat image, int centerx, int centery, int x_cross_length, int y_cross_length, Scalar color)

{

	Point pt1, pt2, pt3, pt4;



	pt1.x = centerx - x_cross_length;

	pt1.y = centery;

	pt2.x = centerx + x_cross_length;

	pt2.y = centery;



	pt3.x = centerx;

	pt3.y = centery - y_cross_length;

	pt4.x = centerx;

	pt4.y = centery + y_cross_length;



	line(image, pt1, pt2, color, 1, 8);

	line(image, pt3, pt4, color, 1, 8);

}



void init() {

	// Initialize Image

	orignal_image = imread("C:/tmp/test1.jpg", IMREAD_ANYDEPTH);  //사진일때

	resize(orignal_image, orignal_image, Size(320, 244), 0, 0, CV_INTER_LINEAR);

	gray_image = orignal_image.clone();

	// imporved accuracy

	GaussianBlur(gray_image, gray_image, Size(3, 3), 0);

	cr_image = Mat(gray_image.rows, gray_image.cols, CV_8UC3, Scalar(0));

	pupil_image = Mat(gray_image.rows, gray_image.cols, CV_8UC3, Scalar(0));

	overay_image = Mat(gray_image.rows, gray_image.cols, CV_8UC3, Scalar(0));

	empty_image = Mat(gray_image.rows, gray_image.cols, CV_8UC3);

	memset(overay_image.data, 0, overay_image.size().width * overay_image.size().height * overay_image.depth());

	setGUI();

}



void calib_init() { //칼리브레이션용 초기화

	calib_orignal_image = imread("C:/tmp/test1.jpg", IMREAD_ANYDEPTH);  //사진일때

	resize(calib_orignal_image, calib_orignal_image, Size(320, 244), 0, 0, CV_INTER_LINEAR);

	calib_gray_image = calib_orignal_image.clone();

	GaussianBlur(calib_gray_image, calib_gray_image, Size(3, 3), 0);

}





void on_trackbar(int, void*)

{

	// I lost something? no

}



void timer() {

	ftime(&timebuffer);  // timebuffer를채운다

	ltime = timebuffer.time;  // time_t 정보를가져온다

	milisec = timebuffer.millitm;  // milisec를구한다

	now = localtime(&ltime);  // time 정보를채운다

}



//여기서부터 함수 추가

/*

void draw_all() {



   Draw_Cross(calib_orignal_image, info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y, 3, 3, Scalar(0, 0, 255));



   for (int i = 0; i < starburst->edge_point.size(); i++) {

	  int xx = (int)starburst->edge_point[i].x;

	  int yy = (int)starburst->edge_point[i].y;

	  Draw_Cross(calib_orignal_image, xx, yy, 1, 1, Scalar(0, 255, 0)); // 초록색으로 원 그리기

   }



   Draw_Cross(calib_orignal_image, info->PUPIL.center_of_x, info->PUPIL.center_of_y, 3, 3, Scalar(0, 255, 255));

   ellipse(calib_orignal_image,   // img, center, axis (장축,단축), 회전각, 호의 시작각도, 호의 끝각도, 타원의 색,

	  Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y),

	  Size(info->PUPIL.width, info->PUPIL.height),

	  -info->PUPIL.theta * 180 / PI, 0, 360, Scalar(0, 255, 255), 2);



   line(calib_orignal_image, Point(info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y),

	  Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y), Scalar(0, 255, 0), 1, 8, 0);

}

*/



void affine_matrix_inverse(double a[][3], double r[][3]) {

	// det은 determinant 행렬식이란 것. det22 는 2차원 행렬식

	double det22 = a[0][0] * a[1][1] - a[0][1] * a[1][0]; // ad-bc

	r[0][0] = a[1][1] / det22; // 역행렬 구하는 공식

	r[0][1] = -a[0][1] / det22;

	r[1][0] = -a[1][0] / det22;

	r[1][1] = a[0][0] / det22;



	r[2][0] = r[2][1] = 0;

	r[2][2] = 1 / a[2][2];



	r[0][2] = -r[2][2] * (r[0][0] * a[0][2] + r[0][1] * a[1][2]);

	r[1][2] = -r[2][2] * (r[1][0] * a[0][2] + r[1][1] * a[1][2]);

}



// 행렬 곱하기 a x b = r ==> 즉 r을 반환

// r is result matrix

void matrix_multiply33(double a[][3], double b[][3], double r[][3]) {

	int i, j;

	double result[9];

	double v = 0;



	// 3x3 곱하기

	for (j = 0; j < 3; j++) {

		for (i = 0; i < 3; i++) {

			v = a[j][0] * b[0][i];

			v += a[j][1] * b[1][i];

			v += a[j][2] * b[2][i];

			result[j * 3 + i] = v;

		}

	}



	// result를 다시 r에 집어넣음.

	for (i = 0; i < 3; i++) {

		r[i][0] = result[i * 3];

		r[i][1] = result[i * 3 + 1];

		r[i][2] = result[i * 3 + 2];

	}

}



/*

// 캘리브레이션 9개 점 그리기

void Show_Calibration_Points() {

   int i;

   for (i = 0; i < CALIBRATIONPOINTS; i++)

	  Draw_Cross(scene_image, scenecalipoints[i].x, scenecalipoints[i].y, 25, 25, CV_RGB(255, 255, 255));

}

*/

void Zero_Calibration() {

	int i;

	// 캘리브레이션 9번 하기위해 정보 초기화 

	number_calibration_points_set = 0;

}



void Activate_Calibration() {

	int i;

	if (number_calibration_points_set == CALIBRATIONPOINTS) {

		do_map2scene = !do_map2scene; // 0을 1로

		view_cal_points = !view_cal_points; // 1을 0으로

	}

	// 9개 안모였을 때

	else {

		printf("Attempt to activate calibration without a full set of points.\n");

	}

}



void Set_Calibration_Point(int x, int y) {

	// 9번 캘리브레이션 안했으면 계속 해 

	if (number_calibration_points_set < CALIBRATIONPOINTS) {

		// 횟수

		number_calibration_points_set++;

		printf("calibration points number: %d (total 9)\n", number_calibration_points_set);

	}



	// 9번하면은 초기화하고

	else {

		Zero_Calibration();

	}

}



void pause(void) {  //일시정지

	printf("Press any key to continue . . .");

	getch();  // 아무 키나 1개 입력 받기

	puts(""); // 줄바꿈

}



void on_mouse_homo(int event, int x, int y, int flags, void *param) {

	int i;

	switch (event) {

		// 왼쪽버튼

	case CV_EVENT_LBUTTONDOWN:

		printf("%d %d \n", x, y);

		pupil.x = 10; pupil.y = 10;

		start_point.x = 10; start_point.y = 10;

		end_point.x = calib_orignal_image.size().width - 10;

		end_point.y = calib_orignal_image.size().height - 10;

		starburst_calib = new StarBurst(calib_gray_image);

		starburst_calib->rect = Rect(start_point.x, start_point.y, end_point.x - start_point.x, end_point.y - start_point.y);

		info_calib = starburst_calib->Apply();

		printf("각막반사의 중심은? %d %d \n", info_calib->CORNEAL_REFLEX.center_of_x, info_calib->CORNEAL_REFLEX.center_of_y);

		printf("동공의 중심은? %d %d \n", info_calib->PUPIL.center_of_x, info_calib->PUPIL.center_of_y);

		printf("클릭 %d %d \n", x, y);



		screen.x = x;  //스크린에서의 좌표

		screen.y = y;

		myEye.x = info_calib->PUPIL.center_of_x;  //시선 추적해서 나온 좌표

		myEye.y = info_calib->PUPIL.center_of_y;

		screen_point.push_back(screen);   //좌표들을 벡터에 넣음

		eye_point.push_back(myEye);



		//on_mouse_eye(0);

		Set_Calibration_Point(x, y);

		//pause();

		break;



		// 오른쪽 버튼

	case CV_EVENT_RBUTTONDOWN:

		lambda = findHomography(screen_point, eye_point, RANSAC);

		cout << lambda << endl;

		//pause();

		Activate_Calibration();

		break;

	}

}



// 여기까지



void on_mouse_eye(int flags)

{

	//printf("onmouse!!!!");

	static bool start = 0;

	int x, y;

	int start_x = 10;

	int start_y = 10;

	x = start_x;

	y = start_y;



	int end_x = gray_image.size().width - 10;

	int end_y = gray_image.size().height - 10;

	//   printf("start x,y:  %d %d  end x,y:  %d %d\n", x, y, end_x, end_y);

	   //This is really the left mouse button



	//   printf("left mouse eye window (%d,%d)\n", x, y);

	pupil.x = x;

	pupil.y = y;

	//if (!start) { 

	//printf("start point: %d, %d\n", x, y); 

	start_point.x = x;

	start_point.y = y;

	start = 1;

	flag = 0;

	//}





	//This is really the right mouse button

 // case EVENT_MBUTTONDOWN:

	// break;



	 //This is really the scroll button

 // case EVENT_RBUTTONDOWN:

	memset(overay_image.data, 0, overay_image.size().width * overay_image.size().height * overay_image.depth());

	memset(pupil_image.data, 0, overay_image.size().width * overay_image.size().height * overay_image.depth());  //??? 확실치 않음





	end_point.x = end_x;

	end_point.y = end_y;

	//printf("end point : %d, %d\n", end_point.x, end_point.y);

	//cvRectangle(orignal_image,start_point,end_point,cvScalar(255,0,0),3,8,0);



	// draw in overay_image

	//cvRectangle(overay_image,start_point,end_point,cvScalar(0,255,255),1,8,0);

	rectangle(overay_image, start_point, end_point, Scalar(0, 255, 255), 1, 8, 0);



	flag = 1;

	// StarBurst Algorithme Working on *Gray* Image



	//StarBurst * starburst = new StarBurst(gray_image);

	StarBurst * starburst = new StarBurst(gray_image);

	starburst->rect = Rect(start_point.x, start_point.y, end_point.x - start_point.x, end_point.y - start_point.y);

	//StarBurstInfo * info = starburst->Apply();

	StarBurstInfo * info = starburst->Apply();



	printf("----------------- Result -------------------\n");

	printf("Corneal Pos(%d,%d) - radius(%d)\n",

		info->CORNEAL_REFLEX.center_of_x,

		info->CORNEAL_REFLEX.center_of_y,

		info->CORNEAL_REFLEX.radius);



	// end StarBurst ----------------------------------------------

	//printf("%d %d \n", starburst->getCornealRemovedImage().channels(), cr_image.channels());



	//cvConvertImage(starburst->getCornealRemovedImage(), cr_image,0);

	cvtColor(starburst->getCornealRemovedImage(), cr_image, CV_GRAY2BGR);

	// cvConvertImage(starburst->getCornealRemovedImage(), pupil_image,0);

	cvtColor(starburst->getCornealRemovedImage(), pupil_image, CV_GRAY2BGR);

	// cvtColor(starburst->getCornealRemovedImage(), cr_image, CV_GRAY2RGB);

	// cvtColor(starburst->getCornealRemovedImage(), pupil_image, CV_GRAY2RGB);





	int guess_crx = starburst->rect.width / 2 + starburst->rect.x;         // Best Guess Pupil center x

	int guess_cry = starburst->rect.height / 2 + starburst->rect.y;      // Best Guess Pupil center y



	//draw cr on cr_image

	Draw_Cross(cr_image, info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y, 3, 3, Scalar(0, 0, 255));



	if (info->CORNEAL_REFLEX.center_of_x > 0 || info->CORNEAL_REFLEX.center_of_y > 0 || info->CORNEAL_REFLEX.radius > 0) {



		cv::circle(cr_image, Point(info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y),

			info->CORNEAL_REFLEX.radius, Scalar(0, 0, 255), 1, 8, 0);



		// draw Edge on pupil_image

		for (int i = 0; i < starburst->edge_point.size(); i++) {

			int xx = (int)starburst->edge_point[i].x;

			int yy = (int)starburst->edge_point[i].y;

			Draw_Cross(pupil_image, xx, yy, 2, 2, Scalar(0, 255, 0));

		}



		// draw cr on overay image

		Draw_Cross(overay_image, info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y, 3, 3, Scalar(0, 0, 255));

		Draw_Cross(orignal_image, info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y, 3, 3, Scalar(0, 0, 255));



		circle(overay_image, Point(info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y),

			info->CORNEAL_REFLEX.radius, Scalar(0, 0, 255), 1, 8, 0);

		circle(orignal_image, Point(info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y),

			info->CORNEAL_REFLEX.radius, Scalar(0, 0, 255), 1, 8, 0);



		// draw Pupil on overay image

		//Draw_Cross(pupil_image, info->PUPIL.center_of_x, info->PUPIL.center_of_y , 3,3, cvScalar(0,255,0));

		Draw_Cross(overay_image, info->PUPIL.center_of_x, info->PUPIL.center_of_y, 3, 3, Scalar(0, 255, 255));

		Draw_Cross(orignal_image, info->PUPIL.center_of_x, info->PUPIL.center_of_y, 3, 3, Scalar(0, 255, 255));

		ellipse(overay_image,

			Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y),

			Size(info->PUPIL.width, info->PUPIL.height),

			-info->PUPIL.theta * 180 / PI, 0, 360, Scalar(0, 255, 255), 2);

		ellipse(orignal_image,

			Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y),

			Size(info->PUPIL.width, info->PUPIL.height),

			-info->PUPIL.theta * 180 / PI, 0, 360, Scalar(0, 255, 255), 2);



		// line connect

		line(overay_image, Point(info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y),

			Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y), Scalar(0, 255, 0), 1, 8, 0);

		line(orignal_image, Point(info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y),

			Point(info->PUPIL.center_of_x, info->PUPIL.center_of_y), Scalar(0, 255, 0), 1, 8, 0);

		//CvDPoint get_xy;

		//get_xy.x = info->PUPIL.center_of_x;

		//get_xy.y = info->PUPIL.center_of_y;

		//printf("여기서 오류!!2\n");

		//printf("날 좀 보소2 !!! %lf %lf \n", get_xy.x, get_xy.y);

		//gaze_point2 = homography_map_point(get_xy);

		//printf("날 좀 보소2 !!! %lf %lf \n", gaze_point2.x, gaze_point2.y);

		//int data[] = { info->PUPIL.center_of_x ,info->PUPIL.center_of_y ,1 };

		Mat before = (Mat_<double>(3, 1) << info->PUPIL.center_of_x, info->PUPIL.center_of_y, 1);

		//Mat before(3,1,CV_64F,data);



		cout << before << endl;

		//before[0].x= info->PUPIL.center_of_x;

		//before[0].y= info->PUPIL.center_of_y;

		//Point2f real[1];



		Mat real = lambda.inv()*before;

		//perspectiveTransform(before, real, lambda);

		//warpAffine(before, real,lambda,before.size(), INTER_LINEAR);

		//warpPerspective(before, real, lambda, before.size());

		//lambda2=getPerspectiveTransform(before, real,lambda);

		printf("!!!!555\n");

		cout << real << endl;

		Point calib_point;

		calib_point.x = real.at<double>(0) / real.at<double>(2);

		calib_point.y = real.at<double>(1) / real.at<double>(2);

		Draw_Cross(homo_image, calib_point.x, calib_point.y, 3, 3, Scalar(0, 255, 255));

		//save_XY(info->PUPIL.center_of_x, info->PUPIL.center_of_y);   //x,y 값 txt파일로 저장

		//nMapWrite(lpAddress1, info->PUPIL.center_of_x, info->PUPIL.center_of_y);

		nMapWrite(lpAddress1, calib_point.x, calib_point.y);



		refreshViews();

		free(starburst);

		// break;

		//waitKey(30);

		//}

	}

}



void setGUI() {

	// generate Window

	namedWindow(original_wnd, 1);

	namedWindow(cr_wnd, 1);

	namedWindow(pupil_wnd, 1);

	namedWindow(result_wnd, 1);

	namedWindow(calib, 1);

	// Resizing

	//CvSize size = cvGetSize(orignal_image);

	Size size = orignal_image.size();

	//   cvResizeWindow(cr_wnd,size.width,size.height);

	//   cvResizeWindow(pupil_wnd,size.width,size.height);

	//   cvResizeWindow(result_wnd,size.width,size.height);

	   /*

	   cvMoveWindow(original_wnd,100,100);

	   cvMoveWindow(cr_wnd,100,100);

	   cvMoveWindow(pupil_wnd,100,100);

	   cvMoveWindow(result_wnd,100,100);

	   */

	   // set Callback On Mouse

	//printf("set GUI!!!!");

	//draw_all();

	on_mouse_eye(0);

	// set Callback On Mouse

	//setMouseCallback(original_wnd, on_mouse_eye);

}



int _tmain(int argc, _TCHAR* argv[])

{

	char buf[256];

	char ip_Address[100];

	Mat frameMat;

	int index = 0;

	//printf("웹캠 ip 주소 입력: ");

	//scanf("%s", ip_Address);

	//VideoCapture videoCapture(0);

	//VideoCapture videoCapture("C:/video/eye5.avi");

	VideoCapture videoCapture("http://192.168.0.26:8090/?action=stream");

	if (!videoCapture.isOpened())

	{

		printf("첫번째 카메라를 열수 없습니다. \n");

	}

	int frame_max = videoCapture.get(CAP_PROP_FRAME_COUNT);

	namedWindow("웹캠 영상", WINDOW_AUTOSIZE);



	int i, j;

	double T[3][3], T1[3][3];

	for (j = 0; j < 3; j++) {

		for (i = 0; i < 3; i++) {

			T[j][i] = j * 3 + i + 1;

		}

	}

	T[2][0] = T[2][1] = 0;

	printf("\nT: \n");

	for (j = 0; j < 3; j++) {

		for (i = 0; i < 3; i++) {

			printf("%6.2lf ", T[j][i]);

		}

		printf("\n");

	}

	affine_matrix_inverse(T, T1);

	printf("\nT1: \n");

	for (j = 0; j < 3; j++) {

		for (i = 0; i < 3; i++) {

			printf("%6.2lf ", T1[j][i]);

		}

		printf("\n");

	}

	char c, c2;

	printf("homo_이미지 총 크기 %d %d\n", homo_image.size().width, homo_image.size().height);



	while ((c = cvWaitKey(30)) != 'q') {

		timer();

		//setMouseCallback(original_wnd, on_mouse_origin);

		videoCapture.read(frameMat);

		if (frameMat.empty())

			printf("영상 끝\n");

		//화면에 보여줌

		imshow("웹캠 영상", frameMat);

		calib_setGUI();

		//이미지로 저장  

		if (index % 10 == 0) {

			sprintf(buf, "C:/tmp/test1.jpg");

			imwrite(buf, frameMat);

			calib_init();

			calib_refreshViews();

			//unSetGUI();



		}



		setMouseCallback(homo_wnd, on_mouse_homo);

		if (do_map2scene) { // 캘리브레이션 완료됬을 때 이거 출력!

			calib_unSetGUI();

			break;

			//gaze_point = homography_map_point(diff_vector); 



		}

		else {

			//imshow("웹캠 영상", frameMat);

			imshow(homo_wnd, homo_image);

		}

	}  //칼리브레이션 구간



	while (1)

	{

		timer();

		//웹캡으로부터 한 프레임을 읽어옴  

		videoCapture.read(frameMat);

		if (frameMat.empty())

			printf("영상 끝\n");

		//화면에 보여줌

		imshow("웹캠 영상", frameMat);



		//이미지로 저장  

		if (index % 10 == 0) {

			sprintf(buf, "C:/tmp/test1.jpg");

			//cout << buf << endl;

			imwrite(buf, frameMat);

			init();

			//setGUI();

			//printf("init() 끝\n");

			refreshViews();

			//unSetGUI();

		}

		index++;

		if (waitKey(25) == 27) break; //ESC키 누르면 종료  

	}



	//init();

	// View All thing

	//setGUI();

	//refreshViews();
	// Wating For Key Input
	waitKey(0);
	// unset all Windows & release All Image

	unSetGUI();

	return 0;

}