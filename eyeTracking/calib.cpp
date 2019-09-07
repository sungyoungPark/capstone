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

int view_cal_points = 1; // �������϶��� flag
int do_map2scene = 0; // Ķ���극�̼� �Ϸ������ flag

int number_calibration_points_set = 0; // ���� Ķ���극�̼� ��� �� �� �ߴ��� �����Ȳ
int ok_calibrate = 0;

Point  calipoints[CALIBRATIONPOINTS];       //conversion from eye to scene calibration points -> ������ Scene������ ��� ��ȯ�� 9��
Point  scenecalipoints[CALIBRATIONPOINTS]; //captured (with mouse) calibration points
Point  pucalipoints[CALIBRATIONPOINTS];   //captured eye points while looking at the calibration points in the scene
Point  crcalipoints[CALIBRATIONPOINTS];    //captured corneal reflection points while looking at the calibration points in the scene
Point  vectors[CALIBRATIONPOINTS];         //differences between the corneal reflection and pupil center -> ������ �����ݻ� ���� ���� 9��


// ���������� ������
// scene coordinate interpolation variables
float a, b, c, d, e;                            //temporary storage of coefficients -> 4�� ������������ ����� Cramer's rule�� ������ ��Ĺ� ���������� Ǯ��. 
float aa, bb, cc, dd, ee;                       //pupil X coefficients - ���� X���
float ff, gg, hh, ii, jj;                  //pupil Y coefficients - ���� Y��� 

float centx, centy;                             // translation to center pupil data after biquadratics - ������ �� ���� ���� �߽���ǥ
float cmx[4], cmy[4];                           // corner correctioncoefficients - �ڳ� �������
int inx, iny;                                   // translation to center pupil data before biquadratics - ������ �ϱ����� ���� ���� �߽���ǥ

// ��
int White, Red, Green, Blue, Yellow;
int frame_number = 0;

double map_matrix[3][3]; // �����ĵ��� �ش�. ���ĵ��� ���� ���� ��.

// UI �Կ� ���� ����
int save_image = 0; // �̹��� ���� ����
int image_no = 0; // �̹��� �ѹ�
int save_ellipse = 0;
int ellipse_no = 0;
char eye_file[30];
char scene_file[30];
char ellipse_file[40];

#define FIX_UINT8(x) ( (x)<0 ? 0 : ((x)>255 ? 255:(x)) )

Point zoozak = { 1,1 };


void affine_matrix_inverse(double a[][3], double r[][3]) {
	// det�� determinant ��Ľ��̶� ��. det22 �� 2���� ��Ľ�
	double det22 = a[0][0] * a[1][1] - a[0][1] * a[1][0]; // ad-bc
	r[0][0] = a[1][1] / det22; // ����� ���ϴ� ����
	r[0][1] = -a[0][1] / det22;
	r[1][0] = -a[1][0] / det22;
	r[1][1] = a[0][0] / det22;

	r[2][0] = r[2][1] = 0;
	r[2][2] = 1 / a[2][2];

	r[0][2] = -r[2][2] * (r[0][0] * a[0][2] + r[0][1] * a[1][2]);
	r[1][2] = -r[2][2] * (r[1][0] * a[0][2] + r[1][1] * a[1][2]);
}

// ��� ���ϱ� a x b = r ==> �� r�� ��ȯ
// r is result matrix
void matrix_multiply33(double a[][3], double b[][3], double r[][3]) {
	int i, j;
	double result[9];
	double v = 0;

	// 3x3 ���ϱ�
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			v = a[j][0] * b[0][i];
			v += a[j][1] * b[1][i];
			v += a[j][2] * b[2][i];
			result[j * 3 + i] = v;
		}
	}

	// result�� �ٽ� r�� �������.
	for (i = 0; i < 3; i++) {
		r[i][0] = result[i * 3];
		r[i][1] = result[i * 3 + 1];
		r[i][2] = result[i * 3 + 2];
	}
}

int cal_calibration_homography(void) {
	int i, j;
	vector<Point> cal_scene, cal_eye; // stuDPoint - ������ x,y��ǥ Point����
	Point scene_center, eye_center;
	vector<Point> eye_nor, scene_nor;
	double dis_scale_scene, dis_scale_eye;

	// 9�� �� ���� �� �������
	for (i = 0; i < 9; i++) {
		cal_scene.push_back(scenecalipoints[i]);
		//cal_scene[i].x = scenecalipoints[i].x;
		//cal_scene[i].y = scenecalipoints[i].y;
		cal_eye.push_back(vectors[i]);
		//cal_eye[i].x = vectors[i].x;scene
		//cal_eye[i].y = vectors[i].y;
	}

	// 9���� �� scene_nor���� normalize_point_set���� scene�� normalize�� ��ǥ�� ������
	copy(normalize_point_set(cal_scene, dis_scale_scene, scene_center, CALIBRATIONPOINTS).begin(), normalize_point_set(cal_scene, dis_scale_scene, scene_center, CALIBRATIONPOINTS).end(), scene_nor);
	//scene_nor = normalize_point_set(cal_scene, dis_scale_scene, scene_center, CALIBRATIONPOINTS);
	copy(normalize_point_set(cal_eye, dis_scale_eye, eye_center, CALIBRATIONPOINTS).begin(), normalize_point_set(cal_eye, dis_scale_eye, eye_center, CALIBRATIONPOINTS).end(), scene_nor);
	//eye_nor = normalize_point_set(cal_eye, dis_scale_eye, eye_center, CALIBRATIONPOINTS); // ������ xy��ǥ�� cal_eye�� �ְ�.
	printf("normalize_point_set end\n");
	printf("scene scale:%lf  center (%lf, %lf)\n", dis_scale_scene, scene_center.x, scene_center.y);
	printf("eye scale:%lf  center (%lf, %lf)\n", dis_scale_eye, eye_center.x, eye_center.y);

	const int homo_row = 18, homo_col = 9;
	double A[homo_row][homo_col];
	int M = homo_row, N = homo_col; //M is row; N is column
	double **ppa = (double**)malloc(sizeof(double*)*M); // 2���� �迭 
	double **ppu = (double**)malloc(sizeof(double*)*M); // 2���� �迭
	double **ppv = (double**)malloc(sizeof(double*)*N); // 2���� �迭
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
	// --------------------- normalize_point_set �ϴ� �۾� ��

	svd(M, N, ppa, ppu, pd, ppv);
	int min_d_index = 0;
	for (i = 1; i < N; i++) {
		if (pd[i] < pd[min_d_index])
			min_d_index = i;
	}

	// ���� ���� Ư�̰��� �ش��ϴ� �� . �� �������� �ش�
	for (i = 0; i < N; i++) {
		map_matrix[i / 3][i % 3] = ppv[i][min_d_index];  // the column of v that corresponds to the smallest singular value, (���� ���� Ư�̰��� �ش��ϴ� ��)
											 // which is the solution of the equations -> �̰��� �����ĵ��� �ش��̴�.
	}

	double T[3][3] = { 0 }, T1[3][3] = { 0 };

	//**** T1��� ���
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

	//**** map_matrix���(1)
	///*
	printf("\n map_matrix:1 \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", map_matrix[j][i]);
		}
		printf("\n");
	}

	//**** T���
	printf("\n T:1 \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", T[j][i]);
		}
		printf("\n");
	}
	//*/
	// map_matrix�� T�� ����.
	matrix_multiply33(map_matrix, T, map_matrix);

	T[0][0] = T[1][1] = dis_scale_scene;
	T[0][2] = -dis_scale_scene * scene_center.x;
	T[1][2] = -dis_scale_scene * scene_center.y;
	T[2][2] = 1;


	//**** map_matrix ���(2)
	///*
	printf("\n map_matrix:2 \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", map_matrix[j][i]);
		}
		printf("\n");
	}

	//**** T��� ��� (2)
	printf("\n T:2 \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", T[j][i]);
		}
		printf("\n");
	}
	//*/
	// T�� ����Ŀ� affine�۾� T1 -> T1�� map_matrix ���ϱ�
	affine_matrix_inverse(T, T1);
	matrix_multiply33(T1, map_matrix, map_matrix);


	//**** map_matrix ��� (3)
	///*
	printf("\n map_matrix3: \n");
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 3; i++) {
			printf("%8lf ", map_matrix[j][i]);
		}
		printf("\n");
	}
	//*/

	// �޸� ��ȯ �۾�
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

// Ķ���극�̼� 9�� �� �׸���
void Show_Calibration_Points() {
	int i;
	for (i = 0; i < CALIBRATIONPOINTS; i++)

		Draw_Cross(scene_image, scenecalipoints[i].x, scenecalipoints[i].y, 25, 25, CV_RGB(255, 255, 255));
}

void Zero_Calibration() {
	int i;
	// Ķ���극�̼� 9�� �ϱ����� ���� �ʱ�ȭ 
	for (i = 0; i < CALIBRATIONPOINTS; i++) {
		scenecalipoints[i].x = 0; // scene������ calipoint��
		scenecalipoints[i].y = 0; // scene������ calipoint��

		pucalipoints[i].x = 0; // pupil�� calipoint��
		pucalipoints[i].y = 0; // pupil�� calipoint��

		crcalipoints[i].x = 0; // Corneal reflex�� calipoint��
		crcalipoints[i].y = 0; // Corneal reflex�� calipoint��

		vectors[i].x = 0; // pupil�� Corneal reflex�� ���� vector
		vectors[i].y = 0; // pupil�� Corneal reflex�� ���� vector
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

		do_map2scene = !do_map2scene; // 0�� 1��
		view_cal_points = !view_cal_points; // 1�� 0����
	}

	// 9�� �ȸ��� ��
	else {
		printf("Attempt to activate calibration without a full set of points.\n");
	}
}

void Set_Calibration_Point(int x, int y) {

	// 9�� Ķ���극�̼� �������� ��� �� 
	if (number_calibration_points_set < CALIBRATIONPOINTS) {
		//store xy mouse "scene" coordinates into calibration array    
		scenecalipoints[number_calibration_points_set].x = x; // ���콺 x
		scenecalipoints[number_calibration_points_set].y = y; // ���콺 y

		//grab the "pupil" position
		pucalipoints[number_calibration_points_set].x = info->PUPIL.center_of_x;
		pucalipoints[number_calibration_points_set].y = info->PUPIL.center_of_y;

		//grab the "corneal reflection" points  
		crcalipoints[number_calibration_points_set].x = info->CORNEAL_REFLEX.center_of_x;
		crcalipoints[number_calibration_points_set].y = info->CORNEAL_REFLEX.center_of_y;

		//grab the "delta pupil cr" position 
		vectors[number_calibration_points_set].x = diff_vector.x;
		vectors[number_calibration_points_set].y = diff_vector.y;

		// Ƚ��
		number_calibration_points_set++;
		printf("calibration points number: %d (total 9)\n", number_calibration_points_set);
	}

	// 9���ϸ��� �ʱ�ȭ�ϰ�
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

		// ���ʹ�ư
	case CV_EVENT_LBUTTONDOWN:
		printf("%d %d \n", x, y);
		pupil.x = 10; pupil.y = 10;
		start_point.x = 10; start_point.y = 10;
		end_point.x = original_image->width - 10;
		end_point.y = original_image->height - 10;

		starburst = new StarBurst(gray_image);
		starburst->rect = cvRect(start_point.x, start_point.y, end_point.x - start_point.x, end_point.y - start_point.y);
		info = starburst->Apply();
		printf("�����ݻ��� �߽���? %d %d \n", info->CORNEAL_REFLEX.center_of_x, info->CORNEAL_REFLEX.center_of_y);
		printf("������ �߽���? %d %d \n", info->PUPIL.center_of_x, info->PUPIL.center_of_y);
		diff_vector.x = info->PUPIL.center_of_x - info->CORNEAL_REFLEX.center_of_x;
		diff_vector.y = info->PUPIL.center_of_y - info->CORNEAL_REFLEX.center_of_y;
		Set_Calibration_Point(x, y);
		break;

		// ������ ��ư
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
		printf("���̺���x,y %d %d\n", diff_vector.x, diff_vector.y);
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
	printf("homo_�̹��� �� ũ�� %d %d\n", homo_image->width, homo_image->height);
	while ((c = cvWaitKey(30)) != 'q') {
		init(); // â ����
		cvSetMouseCallback(homo_window, on_mouse_homo);
		cvSetMouseCallback(original_wnd, on_mouse_origin);
		refreshViews(); // imshow �ݺ�

		if (do_map2scene) { // Ķ���극�̼� �Ϸ���� �� �̰� ���!
			cvReleaseImage(&homo_image);
			cvDestroyWindow(homo_window);
			draw_all();
			cvCircle(scene_draw_image, gaze_point, 5, cvScalar(255, 0, 0), 3);
			cvShowImage("scene_draw_image", scene_draw_image);
		}
		else {
			cvNamedWindow(homo_window, 1); // 1�̸� WINDOW_AUTOSIZE�� ������ ũ�⿡ �ڵ����� ������.
			cvShowImage(homo_window, homo_image);
		}
	}
}
