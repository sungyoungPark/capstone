#include "Ransac.h"
#include <iostream>
#include <opencv2/opencv.hpp>

//#include <vector>

using namespace std;
using namespace cv;

#define CALIBRATIONPOINTS    9
#define MIN_PUPIL_CONTOUR_POINTS     500   
#define MAX_PUPIL_CONTOUR_POINTS     10000    
#define PUPIL_SIZE_TOLERANCE       1000   //range of allowed pupil diameters
#define MAX_CONTOUR_COUNT      20

Point diff_vector = { 0,0 };             //vector between the corneal reflection and pupil
int corneal_reflection_r = 0;       //the radius of corneal reflection
Point gaze_point = { 1,1 };
Point gaze_point2 = { 1,1 };

int view_cal_points = 1; // 보정중일때의 flag
int do_map2scene = 0; // 캘리브레이션 완료됬을때 flag

int number_calibration_points_set = 0; // 현재 캘리브레이션 몇개의 점 을 했는지 진행상황
int ok_calibrate = 0;

Point  calipoints[CALIBRATIONPOINTS];       //conversion from eye to scene calibration points -> 눈에서 Scene으로의 장면 변환점 9개
Point  scenecalipoints[CALIBRATIONPOINTS]; //captured (with mouse) calibration points
Point  pucalipoints[CALIBRATIONPOINTS];   //captured eye points while looking at the calibration points in the scene
Point  crcalipoints[CALIBRATIONPOINTS];    //captured corneal reflection points while looking at the calibration points in the scene
Point  vectors[CALIBRATIONPOINTS];         //differences between the corneal reflection and pupil center -> 동공과 각막반사 차이 벡터 9개


// 연립방정식 변수들
// scene coordinate interpolation variables
float a, b, c, d, e;                            //temporary storage of coefficients -> 4차 연립방정식의 계수들 Cramer's rule을 적용한 행렬법 연립방정식 풀이. 
float aa, bb, cc, dd, ee;                       //pupil X coefficients - 동공 X계수
float ff, gg, hh, ii, jj;                  //pupil Y coefficients - 동공 Y계수 

float centx, centy;                             // translation to center pupil data after biquadratics - 방정식 한 후의 동공 중심좌표
float cmx[4], cmy[4];                           // corner correctioncoefficients - 코너 보정계수
int inx, iny;                                   // translation to center pupil data before biquadratics - 방정식 하기전의 원래 동공 중심좌표

// 색
int White, Red, Green, Blue, Yellow;
int frame_number = 0;

double map_matrix[3][3]; // 방정식들의 해답. 공식들을 통해 구한 답.

// UI 촬영 조작 관련
int save_image = 0; // 이미지 저장 갯수
int image_no = 0; // 이미지 넘버
int save_ellipse = 0;
int ellipse_no = 0;
char eye_file[30];
char scene_file[30];
char ellipse_file[40];

#define FIX_UINT8(x) ( (x)<0 ? 0 : ((x)>255 ? 255:(x)) )

Point zoozak = { 1,1 };


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

int cal_calibration_homography(void) {
	int i, j;
	vector<Point> cal_scene, cal_eye; // stuDPoint - 더블형 x,y좌표 Point같은
	Point scene_center, eye_center;
	vector<Point> eye_nor, scene_nor;
	double dis_scale_scene, dis_scale_eye;

	// 9개 점 값들 다 집어넣음
	for (i = 0; i < 9; i++) {
		cal_scene.push_back(scenecalipoints[i]);
		//cal_scene[i].x = scenecalipoints[i].x;
		//cal_scene[i].y = scenecalipoints[i].y;
		cal_eye.push_back(vectors[i]);
		//cal_eye[i].x = vectors[i].x;scene
		//cal_eye[i].y = vectors[i].y;
	}

	// 9개의 점 scene_nor에서 normalize_point_set으로 scene을 normalize한 좌표값 가져옴
	copy(normalize_point_set(cal_scene, dis_scale_scene, scene_center, CALIBRATIONPOINTS).begin(), normalize_point_set(cal_scene, dis_scale_scene, scene_center, CALIBRATIONPOINTS).end(), scene_nor);
	//scene_nor = normalize_point_set(cal_scene, dis_scale_scene, scene_center, CALIBRATIONPOINTS);
	copy(normalize_point_set(cal_eye, dis_scale_eye, eye_center, CALIBRATIONPOINTS).begin(), normalize_point_set(cal_eye, dis_scale_eye, eye_center, CALIBRATIONPOINTS).end(), scene_nor);
	//eye_nor = normalize_point_set(cal_eye, dis_scale_eye, eye_center, CALIBRATIONPOINTS); // 백터의 xy좌표를 cal_eye에 넣고.
	printf("normalize_point_set end\n");
	printf("scene scale:%lf  center (%lf, %lf)\n", dis_scale_scene, scene_center.x, scene_center.y);
	printf("eye scale:%lf  center (%lf, %lf)\n", dis_scale_eye, eye_center.x, eye_center.y);

	const int homo_row = 18, homo_col = 9;
	double A[homo_row][homo_col];
	int M = homo_row, N = homo_col; //M is row; N is column
	double **ppa = (double**)malloc(sizeof(double*)*M); // 2차원 배열 
	double **ppu = (double**)malloc(sizeof(double*)*M); // 2차원 배열
	double **ppv = (double**)malloc(sizeof(double*)*N); // 2차원 배열
	double pd[homo_col];

	for (i = 0; i < M; i++) {
		ppa[i] = A[i];
		ppu[i] = (double*)malloc(sizeof(double)*N);
	}
	for (i = 0; i < N; i++) {
		ppv[i] = (double*)malloc(sizeof(double)*N);
	}
	for (j = 0; j < M; j++) {
		if (j % 2 == 0) {
			A[j][0] = A[j][1] = A[j][2] = 0;
			A[j][3] = -eye_nor[j / 2].x;
			A[j][4] = -eye_nor[j / 2].y;
			A[j][5] = -1;
			A[j][6] = scene_nor[j / 2].y * eye_nor[j / 2].x;
			A[j][7] = scene_nor[j / 2].y * eye_nor[j / 2].y;
			A[j][8] = scene_nor[j / 2].y;
		}
		else {
			A[j][0] = eye_nor[j / 2].x;
			A[j][1] = eye_nor[j / 2].y;
			A[j][2] = 1;
			A[j][3] = A[j][4] = A[j][5] = 0;
			A[j][6] = -scene_nor[j / 2].x * eye_nor[j / 2].x;
			A[j][7] = -scene_nor[j / 2].x * eye_nor[j / 2].y;
			A[j][8] = -scene_nor[j / 2].x;
		}
	}
	printf("normalize_point_set end\n");
	// --------------------- normalize_point_set 하는 작업 끝

	svd(M, N, ppa, ppu, pd, ppv);
	int min_d_index = 0;
	for (i = 1; i < N; i++) {
		if (pd[i] < pd[min_d_index])
			min_d_index = i;
	}

	// 가장 작은 특이값에 해당하는 열 . 즉 방정식의 해답
	for (i = 0; i < N; i++) {
		map_matrix[i / 3][i % 3] = ppv[i][min_d_index];  // the column of v that corresponds to the smallest singular value, (가장 작은 특이값에 해당하는 열)
											 // which is the solution of the equations -> 이것이 방정식들의 해답이다.
	}

	double T[3][3] = { 0 }, T1[3][3] = { 0 };

	//**** T1행렬 출력
	/*
	printf("\n T1: \n");
	for (j = 0; j < 3; j++) {
	   for (i = 0; i < 3; i++) {
		  printf("%8lf ", T1[j][i]);
	   }
	   printf("\n");
	}
	*/

	T[0][0] = T[1][1] = dis_scale_eye;
	T[0][2] = -dis_scale_eye * eye_center.x;
	T[1][2] = -dis_scale_eye * eye_center.y;
	T[2][2] = 1;

	//**** map_matrix출력(1)
	///*
	printf("\n map_matrix:1 \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", map_matrix[j][i]);
		}
		printf("\n");
	}

	//**** T출력
	printf("\n T:1 \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", T[j][i]);
		}
		printf("\n");
	}
	//*/
	// map_matrix와 T를 곱함.
	matrix_multiply33(map_matrix, T, map_matrix);

	T[0][0] = T[1][1] = dis_scale_scene;
	T[0][2] = -dis_scale_scene * scene_center.x;
	T[1][2] = -dis_scale_scene * scene_center.y;
	T[2][2] = 1;


	//**** map_matrix 출력(2)
	///*
	printf("\n map_matrix:2 \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", map_matrix[j][i]);
		}
		printf("\n");
	}

	//**** T행렬 출력 (2)
	printf("\n T:2 \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", T[j][i]);
		}
		printf("\n");
	}
	//*/
	// T의 역행렬에 affine작업 T1 -> T1과 map_matrix 곱하기
	affine_matrix_inverse(T, T1);
	matrix_multiply33(T1, map_matrix, map_matrix);


	//**** map_matrix 출력 (3)
	///*
	printf("\n map_matrix3: \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", map_matrix[j][i]);
		}
		printf("\n");
	}
	//*/

	// 메모리 반환 작업
	for (i = 0; i < M; i++) { free(ppu[i]); }
	for (i = 0; i < N; i++) { free(ppv[i]); }
	free(ppu); free(ppv); free(ppa);
	//free(&eye_nor); free(&scene_nor);
	printf("\n finish calculate calibration\n");
	return 1;
}

Point homography_map_point(Point p) {
	Point p2;
	double z = map_matrix[2][0] * p.x + map_matrix[2][1] * p.y + map_matrix[2][2];
	p2.x = (int)((map_matrix[0][0] * p.x + map_matrix[0][1] * p.y + map_matrix[0][2]) / z); // -> homography 
	p2.y = (int)((map_matrix[1][0] * p.x + map_matrix[1][1] * p.y + map_matrix[1][2]) / z);
	return p2;
}

// 캘리브레이션 9개 점 그리기
void Show_Calibration_Points() {
	int i;
	for (i = 0; i < CALIBRATIONPOINTS; i++)

		Draw_Cross(scene_image, scenecalipoints[i].x, scenecalipoints[i].y, 25, 25, CV_RGB(255, 255, 255));
}

void Zero_Calibration() {
	int i;
	// 캘리브레이션 9번 하기위해 정보 초기화 
	for (i = 0; i < CALIBRATIONPOINTS; i++) {
		scenecalipoints[i].x = 0; // scene에서의 calipoint점
		scenecalipoints[i].y = 0; // scene에서의 calipoint점

		pucalipoints[i].x = 0; // pupil의 calipoint점
		pucalipoints[i].y = 0; // pupil의 calipoint점

		crcalipoints[i].x = 0; // Corneal reflex의 calipoint점
		crcalipoints[i].y = 0; // Corneal reflex의 calipoint점

		vectors[i].x = 0; // pupil과 Corneal reflex의 차이 vector
		vectors[i].y = 0; // pupil과 Corneal reflex의 차이 vector
	}
	number_calibration_points_set = 0;
}

void Activate_Calibration() {
	int i;
	int calibration_result;
	//printf("Map eye to scene image\n");
	if (number_calibration_points_set == CALIBRATIONPOINTS) {
		calibration_result = cal_calibration_homography();
		printf("Calibration result = %d\n", calibration_result);

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
		//store xy mouse "scene" coordinates into calibration array    
		scenecalipoints[number_calibration_points_set].x = x; // 마우스 x
		scenecalipoints[number_calibration_points_set].y = y; // 마우스 y

		//grab the "pupil" position
		pucalipoints[number_calibration_points_set].x = info->PUPIL.center_of_x;
		pucalipoints[number_calibration_points_set].y = info->PUPIL.center_of_y;

		//grab the "corneal reflection" points  
		crcalipoints[number_calibration_points_set].x = info->CORNEAL_REFLEX.center_of_x;
		crcalipoints[number_calibration_points_set].y = info->CORNEAL_REFLEX.center_of_y;

		//grab the "delta pupil cr" position 
		vectors[number_calibration_points_set].x = diff_vector.x;
		vectors[number_calibration_points_set].y = diff_vector.y;

		// 횟수
		number_calibration_points_set++;
		printf("calibration points number: %d (total 9)\n", number_calibration_points_set);
	}

	// 9번하면은 초기화하고
	else {
		Zero_Calibration();
	}
}

void on_mouse_origin(int event, int x, int y, int flags, void *param) {
	int i;
	switch (event) {
	case CV_EVENT_LBUTTONDOWN:
		printf("%d %d \n", x, y);
	}
}
void on_mouse_homo(int event, int x, int y, int flags, void *param) {
	int i;
	switch (event) {

		// 왼쪽버튼
	case CV_EVENT_LBUTTONDOWN:
		printf("%d %d \n", x, y);
		pupil.x = 10; pupil.y = 10;
		start_point.x = 10; start_point.y = 10;
		end_point.x = original_image->width - 10;
		end_point.y = original_image->height - 10;

		starburst = new StarBurst(gray_image);
		starburst->rect = cvRect(start_point.x, start_point.y, end_point.x - start_point.x, end_point.y - start_point.y);
		info = starburst->Apply();
		printf("각막반사의 중심은? %d %d \n", info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y);
		printf("동공의 중심은? %d %d \n", info->PUPIL.center_of_x, info->PUPIL.center_of_y);
		diff_vector.x = info->PUPIL.center_of_x - info->CORNEAL_REFLEX.center_of_x;
		diff_vector.y = info->PUPIL.center_of_y - info->CORNEAL_REFLEX.center_of_y;
		Set_Calibration_Point(x, y);
		break;

		// 오른쪽 버튼
	case CV_EVENT_RBUTTONDOWN:
		for (int i = 0; i < CALIBRATIONPOINTS; i++) {
			printf("scenecalipoints %d %d \n", scenecalipoints[i].x, scenecalipoints[i].y);
		}
		for (int i = 0; i < CALIBRATIONPOINTS; i++) {
			printf("pucalipoints %d %d \n", pucalipoints[i].x, pucalipoints[i].y);
		}
		for (int i = 0; i < CALIBRATIONPOINTS; i++) {
			printf("crcalipoints %d %d \n", crcalipoints[i].x, crcalipoints[i].y);
		}
		for (int i = 0; i < CALIBRATIONPOINTS; i++) {
			printf("vectors %d %d \n", vectors[i].x, vectors[i].y);
		}

		Activate_Calibration();
		gaze_point = homography_map_point(diff_vector);
		printf("차이벡터x,y %d %d\n", diff_vector.x, diff_vector.y);
		printf("gaze_point: (%d,%d)\n", gaze_point.x, gaze_point.y);
		break;
	}
}

int calib_init(){
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
	printf("homo_이미지 총 크기 %d %d\n", homo_image->width, homo_image->height);
	while ((c = cvWaitKey(30)) != 'q') {
		init(); // 창 생성
		cvSetMouseCallback(homo_window, on_mouse_homo);
		cvSetMouseCallback(original_wnd, on_mouse_origin);
		refreshViews(); // imshow 반복

		if (do_map2scene) { // 캘리브레이션 완료됬을 때 이거 출력!
			cvReleaseImage(&homo_image);
			cvDestroyWindow(homo_window);
			draw_all();
			cvCircle(scene_draw_image, gaze_point, 5, cvScalar(255, 0, 0), 3);
			cvShowImage("scene_draw_image", scene_draw_image);
		}
		else {
			cvNamedWindow(homo_window, 1); // 1이면 WINDOW_AUTOSIZE로 영상의 크기에 자동으로 맞춰줌.
			cvShowImage(homo_window, homo_image);
		}
	}
}
