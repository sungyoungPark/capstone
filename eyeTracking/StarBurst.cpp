#include "stdafx.h"
#include "Ransac.h"
#include "StarBurst.h"
#include "StarBurstHelper.h"
#include <time.h>
#include <sys/timeb.h>

extern double pupil_param[5];
StarBurst::StarBurst(Mat image)
{
	// initializing...
	this->m_image = NULL;
	this->m_removed_image = NULL;

	this->m_image = image;

	// Clear Info Data Structure
	memset(&info, 0, sizeof(StarBurstInfo));
	edge_thresh = 3;

}

StarBurst::~StarBurst(void)
{

}

StarBurstInfo * StarBurst::Apply(void)
{

	// create StarBurstHelper Class for PreProcessing...
	// Step 1: Finding Corneal Reflex
	// Step 2: Remove Corneal Reflex by Interporation
	printf("Star Burst --->\n");
	//여기까지 이상 x
	helper = new StarBurstHelper(this->m_image);
	helper->rect = this->rect;
	this->m_removed_image = helper->CornealReflexRemove(info);

	ApplyRANSAC();

	return (&info);
}


void StarBurst::DetectCoutourPoints(void)
{
	int guess_crx = rect.width / 2 + rect.x;			// Best Guess Pupil center x
	int guess_cry = rect.height / 2 + rect.y;		// Best Guess Pupil center y
	// Cleare All Edge Point
	int new_crx = -1;
	int new_cry = -1;
	Point point;
	//while( (guess_crx != new_crx)  &&  (guess_cry != new_cry)){
	while ((abs(guess_crx - new_crx) > 1) && (abs(guess_cry - new_cry) > 1)) {
		//for(int k = 1 ; k<=10 ; k++){
		edge_point.clear();
		printf("------------CLEAR-------------\n");
		if (new_crx != -1 && new_cry != -1) {
			guess_crx = new_crx;
			guess_cry = new_cry;
		}
		printf("center [%3d , %3d]\n", guess_crx, guess_cry);
		// Step 1 : Finding Candicate
		for (int i = 1; i < helper->angle_num; i = i + 25) {
			shotRay(i, guess_crx, guess_cry, edge_thresh);
		}
		printf("shotRay for문 끝\n");
		// Step 2: Each Cadicate Finding Another Candicate
		// Shot to Center Point like Fan (100 degree)
		int degree = 0;

		double theta;
		int last = edge_point.size();
		if (last > 20)
			last = 20;
		for (int i = 0; i < last; i++) {
			int x = edge_point[i].x;
			int y = edge_point[i].y;

			//printf("guess_crx =%d  guess_cry=%d\n", guess_crx, guess_cry);
			//printf("x=%d y=%d\n", x, y);

			// 이부분의 공식이 잘 처리되고 있는지 확인이 필요함
			if (guess_crx < x && guess_cry < y) {
				theta = atan2((double)y - guess_cry, (double)x - guess_crx);
				degree = (int)(2 * PI / theta) + 180;
			}

			else if (guess_crx > x && guess_cry < y) {
				theta = atan2((double)y - guess_cry, (double)guess_crx - x);
				degree = 360 - (int)(2 * PI / theta);
			}
			else if (guess_crx > x && guess_cry > y) {
				theta = atan2((double)y - guess_cry, (double)guess_crx - x);
				degree = (int)(2 * PI / theta);
			}
			else if (guess_crx < x && guess_cry > y) {
				theta = atan2((double)guess_cry - y, (double)x - guess_crx);
				degree = 180 - (int)(2 * PI / theta);
			}

			for (int j = -50; j <= 50; j = j + 15) {
				shotRay(degree + j, x, y, 20);
			}

		}

		// determine new Center of pupil
		point = calculateCoverage();
		new_crx = point.x;
		new_cry = point.y;

	}
}

/**
 * Starburst Entry Point
 */
void StarBurst::ApplyRANSAC(void)
{
	DetectCoutourPoints();
	int max;
	int* inliers_index = pupil_fitting_inliers((UINT8*)m_removed_image.data, m_removed_image.size().width, m_removed_image.size().height, max, this->edge_point);
	info.PUPIL.width = (int)pupil_param[0];
	info.PUPIL.height = (int)pupil_param[1];
	info.PUPIL.center_of_x = (int)pupil_param[2];
	info.PUPIL.center_of_y = (int)pupil_param[3];
	info.PUPIL.theta = pupil_param[4];

	//pupil_param[0], pupil_param[1], pupil_param[2], pupil_param[3], pupil_param[4], inliers_num
	/*printf("ellipse a:%lf; b:%lf, cx:%lf, cy:%lf, theta:%lf; inliers_num:%d\n\n",
	pupil_param[0], pupil_param[1], pupil_param[2], pupil_param[3], pupil_param[4], inliers_num);*/
}

Mat StarBurst::getCornealRemovedImage(void)
{
	return this->m_removed_image;
}

void StarBurst::destroy_edge_point(void)
{
	/*
	vector <cvDPoint*>::iterator iter;

	if (edge_point.size() != 0) {
		for (iter = edge_point.begin(); iter != edge_point.end( ) ; iter++ ) {
			free(*iter);
		}
		edge_point.clear();
	}
	*/
}

// Feature Based Approach
void StarBurst::shotRay(int angleDegree, int x, int y, int thresh)
{
	if (x > 300 && y > 300) {
		return;
	}
	int angle = fitDegree(angleDegree);
	double r = 0.0;
	double xx = -1.0;			// x'
	double yy = -1.0;			// y'
	// center pixel value
	int pixelvalue1 = *(m_removed_image.data + (y*(m_removed_image.size().width)) + x);
	// if (xx,yy) at border then can't find candicate
	while ((xx == -1.0 && yy == -1.0) || (!isBorder(xx, yy))) {
		xx = (r * helper->cos_array[angle]) + x;
		yy = (r * helper->sin_array[angle]) + y;
		int pixelvalue2 = *(m_removed_image.data + ((int)yy*(m_removed_image.size().width)) + (int)xx);
		// calculate intensity...
		//if( (abs(pixelvalue2-pixelvalue1)) > thresh){
		if (pixelvalue2 - pixelvalue1 > thresh) {
			//printf("%d - %d = %d / %d\n",pixelvalue2 , pixelvalue1,pixelvalue2 -pixelvalue1 , thresh);
			CvDPoint point;
			// 항상 안쪽에 값으로 위치할 수 있도록 함!!
			xx = (r - 3.0) * helper->cos_array[angle] + (double)x;
			yy = (r - 3.0) * helper->sin_array[angle] + (double)y;
			point.x = xx;
			point.y = yy;
			edge_point.push_back(point);
			printf("DETECT CONTOUR POINT = [%3d,%3d]\n",xx,yy);
			return;
		}
		r = r + 1.0;
	}
}




bool StarBurst::isBorder(int xx, int yy)
{
	/*
	if((xx <= rect.x || xx >= rect.width +rect.x||
		yy <= rect.y || yy >= rect.height +rect.y))
	{
	*/
	if ((xx < 0 || xx >= m_removed_image.size().width ||
		yy < 0 || yy >= m_removed_image.size().height))
	{
		return true;
	}
	return false;
}

Point StarBurst::calculateCoverage(void)
{

	if (edge_point.size() == 0) {
		return Point(rect.width / 2 + rect.x, rect.height / 2 + rect.y);
	}
	int avg_x = 0;
	int avg_y = 0;
	int s = edge_point.size();
	for (int i = 0; i < s; i++) {
		avg_x = avg_x + edge_point[i].x;
		avg_y = avg_y + edge_point[i].y;
	}
	return Point(avg_x / s, avg_y / s);
}

int StarBurst::fitDegree(int degree)
{
	if (degree >= 0 && degree < 360) {
		return degree;
	}
	else if (degree >= 360) {
		return degree % 360; //degree - 360;
	}
	else if (degree < 0) {
		return 360 + degree;
	}
}
