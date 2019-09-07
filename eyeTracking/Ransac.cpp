
#include "stdafx.h"
#include "svd.h"


//------------ Ransac ellipse fitting -----------//
double pupil_param[5] = { 0, 0, 0, 0, 0 };

// Randomly select 5 indeics
void get_5_random_num(int max_num, int* rand_num)
{
	int rand_index = 0;
	int r;
	int i;
	bool is_new = 1;

	if (max_num == 4) {
		for (i = 0; i < 5; i++) {
			rand_num[i] = i;
		}
		return;
	}

	while (rand_index < 5) {
		is_new = 1;
		r = (int)((rand()*1.0 / RAND_MAX) * max_num);
		for (i = 0; i < rand_index; i++) {
			if (r == rand_num[i]) {
				is_new = 0;
				break;
			}
		}
		if (is_new) {
			rand_num[rand_index] = r;
			rand_index++;
		}
	}
}


// solve_ellipse
// conic_param[6] is the parameters of a conic {a, b, c, d, e, f}; conic equation: ax^2 + bxy + cy^2 + dx + ey + f = 0;
// ellipse_param[5] is the parameters of an ellipse {ellipse_a, ellipse_b, cx, cy, theta}; a & b is the major or minor axis; 
// cx & cy is the ellipse center; theta is the ellipse orientation
bool solve_ellipse(double* conic_param, double* ellipse_param)
{
	double a = conic_param[0];
	double b = conic_param[1];
	double c = conic_param[2];
	double d = conic_param[3];
	double e = conic_param[4];
	double f = conic_param[5];
	//get ellipse orientation
	double theta = atan2(b, a - c) / 2;

	//get scaled major/minor axes
	double ct = cos(theta);
	double st = sin(theta);
	double ap = a * ct*ct + b * ct*st + c * st*st;
	double cp = a * st*st - b * ct*st + c * ct*ct;

	//get translations
	double cx = (2 * c*d - b * e) / (b*b - 4 * a*c);
	double cy = (2 * a*e - b * d) / (b*b - 4 * a*c);

	//get scale factor
	double val = a * cx*cx + b * cx*cy + c * cy*cy;
	double scale_inv = val - f;

	if (scale_inv / ap <= 0 || scale_inv / cp <= 0) {
		//printf("Error! ellipse parameters are imaginary a=sqrt(%lf), b=sqrt(%lf)\n", scale_inv/ap, scale_inv/cp);
		memset(ellipse_param, 0, sizeof(double) * 5);
		return false;
	}

	ellipse_param[0] = sqrt(scale_inv / ap);
	ellipse_param[1] = sqrt(scale_inv / cp);
	ellipse_param[2] = cx;
	ellipse_param[3] = cy;
	ellipse_param[4] = theta;
	return true;
	//return true;
}

vector<CvDPoint> normalize_point_set(vector<CvDPoint> &point_set, double &dis_scale, CvDPoint &nor_center, int num)
{
	double sumx = 0, sumy = 0;
	double sumdis = 0;
	//CvPoint *edge = point_set;
	int i;
	for (i = 0; i < num; i++) {
		sumx += point_set[i].x;
		sumy += point_set[i].y;
		sumdis += sqrt((double)(point_set[i].x *point_set[i].x + point_set[i].y * point_set[i].y));
	}

	dis_scale = sqrt((double)2)*num / sumdis;
	nor_center.x = sumx * 1.0 / num;
	nor_center.y = sumy * 1.0 / num;
	//CvPoint *edge_point_nor = (CvPoint*)malloc(sizeof(CvPoint)*num);
	vector<CvDPoint> edge_point_nor;
	//edge = point_set;

	for (i = 0; i < num; i++) {
		CvDPoint edge;
		edge.x = (point_set[i].x - nor_center.x)*dis_scale;
		edge.y = (point_set[i].y - nor_center.y)*dis_scale;

		edge_point_nor.push_back(edge);

	}
	return edge_point_nor;
	//return NULL;
}

vector<CvDPoint> normalize_edge_point(double &dis_scale, CvDPoint &nor_center, int ep_num, vector<CvDPoint> edge_point)
{
	double sumx = 0, sumy = 0;
	double sumdis = 0;

	int i;
	for (i = 0; i < ep_num; i++) {
		sumx += edge_point[i].x;
		sumy += edge_point[i].y;
		sumdis += sqrt((double)(edge_point[i].x *edge_point[i].x + edge_point[i].y * edge_point[i].y));
	}

	dis_scale = sqrt((double)2)*ep_num / sumdis;
	nor_center.x = sumx * 1.0 / ep_num;
	nor_center.y = sumy * 1.0 / ep_num;

	vector<CvDPoint> edge_point_nor;
	//CvPoint *edge_point_nor = (CvPoint*)malloc(sizeof(CvPoint)*ep_num);
	for (i = 0; i < ep_num; i++) {
		CvDPoint edge;
		edge.x = (edge_point[i].x - nor_center.x)*dis_scale;
		edge.y = (edge_point[i].y - nor_center.y)*dis_scale;

		edge_point_nor.push_back(edge);
	}
	return edge_point_nor;
	//return NULL;
}


void denormalize_ellipse_param(double* par, double* normailized_par, double dis_scale, CvDPoint nor_center)
{
	par[0] = normailized_par[0] / dis_scale;	//major or minor axis
	par[1] = normailized_par[1] / dis_scale;
	par[2] = normailized_par[2] / dis_scale + nor_center.x;	//ellipse center
	par[3] = normailized_par[3] / dis_scale + nor_center.y;
}

int* pupil_fitting_inliers(UINT8* pupil_image, int width, int height, int &return_max_inliers_num, vector<CvDPoint> edge_point)
{
	int i;
	int ep_num = edge_point.size();   //ep stands for edge point
	CvDPoint nor_center;
	double dis_scale;
	//double pupil_param[5];
	int ellipse_point_num = 5;	//number of point that needed to fit an ellipse

	if (ep_num < ellipse_point_num) {
		printf("Error! %d points are not enough to fit ellipse\n", ep_num);
		memset(pupil_param, 0, sizeof(pupil_param));
		return_max_inliers_num = 0;
		return NULL;
	}

	//Normalization  // 구조 다시 점검!!
	vector<CvDPoint> edge_point_nor = normalize_edge_point(dis_scale, nor_center, ep_num, edge_point);

	//Ransac
	int *inliers_index = (int*)malloc(sizeof(int)*ep_num);
	int *max_inliers_index = (int*)malloc(sizeof(int)*ep_num);
	int ninliers = 0;
	int max_inliers = 0;
	int sample_num = 1000;	//number of sample
	int ransac_count = 0;
	double dis_threshold = sqrt(3.84)*dis_scale;
	double dis_error;

	memset(inliers_index, int(0), sizeof(int)*ep_num);
	memset(max_inliers_index, int(0), sizeof(int)*ep_num);
	int rand_index[5];
	double A[6][6];
	int M = 6, N = 6; //M is row; N is column
	for (i = 0; i < N; i++) {
		A[i][5] = 1;
		A[5][i] = 0;
	}
	double **ppa = (double**)malloc(sizeof(double*)*M);
	double **ppu = (double**)malloc(sizeof(double*)*M);
	double **ppv = (double**)malloc(sizeof(double*)*N);
	for (i = 0; i < M; i++) {
		ppa[i] = A[i];
		ppu[i] = (double*)malloc(sizeof(double)*N);
	}
	for (i = 0; i < N; i++) {
		ppv[i] = (double*)malloc(sizeof(double)*N);
	}
	double pd[6];
	int min_d_index;
	double conic_par[6] = { 0 };
	double ellipse_par[5] = { 0 };
	double best_ellipse_par[5] = { 0 };
	double ratio;
	while (sample_num > ransac_count) {
		get_5_random_num((ep_num - 1), rand_index);

		//svd decomposition to solve the ellipse parameter
		for (i = 0; i < 5; i++) {
			A[i][0] = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].x;
			A[i][1] = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].y;
			A[i][2] = edge_point_nor[rand_index[i]].y * edge_point_nor[rand_index[i]].y;
			A[i][3] = edge_point_nor[rand_index[i]].x;
			A[i][4] = edge_point_nor[rand_index[i]].y;
		}

		svd(M, N, ppa, ppu, pd, ppv);
		min_d_index = 0;
		for (i = 1; i < N; i++) {
			if (pd[i] < pd[min_d_index])
				min_d_index = i;
		}

		for (i = 0; i < N; i++)
			conic_par[i] = ppv[i][min_d_index];	//the column of v that corresponds to the smallest singular value, 
		//which is the solution of the equations
		ninliers = 0;
		memset(inliers_index, 0, sizeof(int)*ep_num);
		for (i = 0; i < ep_num; i++) {
			dis_error = conic_par[0] * edge_point_nor[i].x*edge_point_nor[i].x +
				conic_par[1] * edge_point_nor[i].x*edge_point_nor[i].y +
				conic_par[2] * edge_point_nor[i].y*edge_point_nor[i].y +
				conic_par[3] * edge_point_nor[i].x + conic_par[4] * edge_point_nor[i].y + conic_par[5];
			if (fabs(dis_error) < dis_threshold) {
				inliers_index[ninliers] = i;
				ninliers++;
			}
		}

		if (ninliers > max_inliers) {
			if (solve_ellipse(conic_par, ellipse_par)) {
				denormalize_ellipse_param(ellipse_par, ellipse_par, dis_scale, nor_center);
				ratio = ellipse_par[0] / ellipse_par[1];
				if (ellipse_par[2] > 0 && ellipse_par[2] <= width - 1 && ellipse_par[3] > 0 && ellipse_par[3] <= height - 1 &&
					ratio > 0.5 && ratio < 2) {
					memcpy(max_inliers_index, inliers_index, sizeof(int)*ep_num);
					for (i = 0; i < 5; i++) {
						best_ellipse_par[i] = ellipse_par[i];
					}
					max_inliers = ninliers;
					sample_num = (int)(log((double)(1 - 0.99)) / log(1.0 - pow(ninliers*1.0 / ep_num, 5)));
				}
			}
		}
		ransac_count++;
		if (ransac_count > 1500) {
			printf("Error! ransac_count exceed! ransac break! sample_num=%d, ransac_count=%d\n", sample_num, ransac_count);
			break;
		}
	}
	//INFO("ransc end\n");
	if (best_ellipse_par[0] > 0 && best_ellipse_par[1] > 0) {
		for (i = 0; i < 5; i++) {
			pupil_param[i] = best_ellipse_par[i];
		}
	}
	else {
		memset(pupil_param, 0, sizeof(pupil_param));
		max_inliers = 0;
		free(max_inliers_index);
		max_inliers_index = NULL;
	}

	for (i = 0; i < M; i++) {
		free(ppu[i]);
		free(ppv[i]);
	}
	free(ppu);
	free(ppv);
	free(ppa);


	free(inliers_index);
	return_max_inliers_num = max_inliers;
	return max_inliers_index;
	//return NULL;
}

