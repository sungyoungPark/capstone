#include "StdAfx.h"
#include "StarBurstHelper.h"


StarBurstHelper::StarBurstHelper(Mat image)
{
	// initializing...
	this->m_image = NULL;
	this->m_removed_image = NULL;

	this->m_image = image;
	this->m_removed_image = this->m_image.clone();	// clone by original


	angle_delta = 1 * PI / 180;														// it is a radian Value of one degree : 0.0174
	angle_num = (int)(2 * PI / angle_delta);									// number of angle : 360
	biggest_crar = (int)((m_image.size().height / 10) / 2.5);			// bggest radius of r in Image : it would be lower than (height/10)
	angle_array = (double*)malloc(angle_num * sizeof(double));
	sin_array = (double*)malloc(angle_num * sizeof(double));
	cos_array = (double*)malloc(angle_num * sizeof(double));

	// calculate sin, cos for each angle
	for (int i = 0; i < angle_num; i++) {
		angle_array[i] = i * angle_delta;
		cos_array[i] = cos(angle_array[i]);
		sin_array[i] = sin(angle_array[i]);
	}	// End Prepare Data

}

StarBurstHelper::~StarBurstHelper(void)
{
}

Mat StarBurstHelper::CornealReflexRemove(StarBurstInfo &info)   //여기 찾기
{

	printf("pupil %d %d\n", m_pupil.x, m_pupil.y);
	printf("m_statyPoint  %d %d\n", m_startPoint.x, m_startPoint.y);

	if ((m_image.empty()) ||
		(m_pupil.x == 0 && m_pupil.y == 0) ||
		(m_startPoint.x = 0 && m_startPoint.y == 0)) {
		printf("Error in StarBurst Helper Class::CornealReflexRemove Method\n");
		//return m_image;  //???? 불확실
	}

	// Function Call to Find CR location
	CrDetect();

	int fit_radius = this->getFitRadius();

	fit_radius = (int)(2.5*fit_radius);

	printf("Fit Radius = %d\n", fit_radius);

	info.CORNEAL_REFLEX.center_of_x = crx;
	info.CORNEAL_REFLEX.center_of_y = cry;
	info.CORNEAL_REFLEX.radius = crar;
	info.CORNEAL_REFLEX.fit_radius = fit_radius;

	CrRemove(info);
	//imshow("코니얼", m_removed_image);
	return this->m_removed_image;
}

void StarBurstHelper::CrDetect(void)
{
	//Rect rect(80, 80, 110, 110);
	//m_image= m_image(rect);

	Mat origin;
	int r = (m_image.size().width - 1) / 2; // window size?
	int startx = MAX(m_startPoint.x - r, 0);
	int endx = MIN(m_startPoint.x + r, m_image.size().width - 1);
	int starty = MAX(m_startPoint.y - r, 0);
	int endy = MIN(m_startPoint.y + r, m_image.size().height - 1);
	int biggest_crar = (int)((m_image.size().height / 10) / 2.5);

	printf(" r %d startx %d endx %d starty %d endy %d biggest_crar %d \n", r, startx, endx, starty, endy, biggest_crar);

	///* For Debugging...
	//printf("(%d,%d) - (%d,%d)\n",startx,starty,endx,endy);
	//*/

	int threshold;								// Threshold Value
	double min_value, max_value;	// pixel Value
	Point min_loc, max_loc;			// location

	// Finding Minium Pixel Value, Maxium Pixel Value,and Where they Located?
	minMaxLoc(m_image, &min_value, &max_value, &min_loc, &max_loc);

	printf("m_image data %d\n", m_image.data);

	IplImage *t_image = &IplImage(m_image);
	IplImage * threshold_image = cvCloneImage(t_image);
	// Clone Image type from original Image to Threshold_image
	//Mat threshold_image;
	//m_image.copyTo(threshold_image);

	// Setting Region Of Interest
	cvSetImageROI(t_image, rect);
	cvSetImageROI(threshold_image, rect);

	//m_image.copyTo(origin);

	// 관심영역 자르기 (Crop ROI).
	//m_image = m_image(rect);
	//threshold_image = threshold_image(rect);

	CvSeq* contour = NULL;
	CvMemStorage* storage = cvCreateMemStorage(0);

	// array for score : estimated contour
	double *scores = (double*)malloc(sizeof(double)*((int)max_value + 1));
	memset(scores, 0, sizeof(double)*((int)max_value + 1));

	// maxium area 
	int area, max_area, sum_area;


	CvSeq *max_contour;

	// apply adaptive thresholding technique iteration

	for (threshold = (int)max_value; threshold >= 1; threshold--) {

		cvThreshold(t_image, threshold_image, threshold, 1, CV_THRESH_BINARY);
		cvFindContours(threshold_image, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		max_area = 0;
		sum_area = 0;
		max_contour = contour;
		for (; contour != 0; contour = contour->h_next) {

			area = contour->total + (int)(fabs(cvContourArea(contour, CV_WHOLE_SEQ)));
			sum_area += area;
			if (area > max_area) {
				max_area = area;
				max_contour = contour;
			}
		}
		if (sum_area - max_area > 0) {
			scores[threshold - 1] = max_area / (sum_area - max_area);
			/*printf("max_area: %d, max_contour: %d, sum_area: %d; scores[%d]: %lf\n",
					max_area, max_contour->total, sum_area, threshold-1, scores[threshold-1]);      */
		}
		else
			continue;

		if (scores[threshold - 1] - scores[threshold] < 0) {
			//found the corneal reflection
			//printf("max_area %d\n", max_area);   //max_area 문제있음
			crar = (int)sqrt(max_area / PI);
			int sum_x = 0;
			int sum_y = 0;
			CvPoint *point;
			for (int i = 0; i < max_contour->total; i++) {
				point = CV_GET_SEQ_ELEM(CvPoint, max_contour, i);
				sum_x += point->x;
				sum_y += point->y;
			}
			//printf("count %d\n", max_contour->total);
			//printf("sum_x %d sum_y %d \n", sum_x, sum_y);
			crx = sum_x / max_contour->total;
			cry = sum_y / max_contour->total;
			break;
		}
	}
	printf("(corneal reflection) max_value = %lf; threshold = %d\n", max_value, threshold);
	//printf("(corneal reflection) Scores:\n");
	//for (int i = (int)max_value; i >= threshold-1; i--) {
	//	printf("%6.2lf", scores[i]);
	//}
	//printf("\n");



	// Reset Image Region Of Interest & Release variable
	free(scores);
	cvReleaseMemStorage(&storage);
	cvResetImageROI(t_image);
	cvResetImageROI(threshold_image);
	//storage.clear();

	//m_image = m_image(rect);
	//origin.copyTo(m_image);
	//m_image.copyTo(m_image(rect));

	//threshold_image.release();


	// final step to calculate result;
	if (crar > biggest_crar) {
		cry = crx = -1;
		crar = -1;
	}

	if (crx != -1 && cry != -1) {
		crx += rect.x;
		cry += rect.y;
	}
}




void StarBurstHelper::CrRemove(StarBurstInfo info)
{
	// Image width must 
	int q = 0;
	//printf("\nstart ----------------------- CR REMOVE ----------------------------------\n");
	int crr = info.CORNEAL_REFLEX.fit_radius;
	if (crx == -1 || cry == -1 || crr == -1) {
		printf("\nsize error\n");
		//return;
	}

	if (crx - crr < 0 || crx + crr >= m_removed_image.size().width || cry - crr < 0 || cry + crr >= m_removed_image.size().height) {
		printf("\nError! Corneal reflection is too near the image border\n");
		//return;
	}

	int i, r, r2, x, y;
	UINT8 *perimeter_pixel = (UINT8*)malloc(angle_num * sizeof(int));
	int sum = 0;
	double avg;
	for (i = 0; i < angle_num; i++) {
		x = (int)(crx + crr * cos_array[i]);
		y = (int)(cry + crr * sin_array[i]);
		// get Adjust Pixel around center position 
		perimeter_pixel[i] = (UINT8)(*(m_removed_image.data + (y*(m_removed_image.size().width + q)) + x));
		//printf("[%d,%d] = %d\n",x,y,perimeter_pixel[i]);
		sum += perimeter_pixel[i];
	}
	// average pixel value
	avg = sum * 1.0 / angle_num;

	for (r = 1; r < crr; r++) {
		r2 = crr - r;
		for (i = 0; i < angle_num; i++) {
			x = (int)(crx + r * cos_array[i]);
			y = (int)(cry + r * sin_array[i]);

			*(m_removed_image.data + (y*(m_removed_image.size().width + q)) + x) =
				(UINT8)((r2*1.0 / crr)*avg + (r*1.0 / crr)*perimeter_pixel[i]);
			//printf("\n(%d,%d) = %d\n // %d",x,y,value , *(m_removed_image->imageData+(y*(m_removed_image->width))+q+x)); 
		}
		/*printf("\nr=%d: %d (avg:%lf, end:%d)\n", r, (UINT8)((r2*1.0/crr)*avg + (r*1.0/crr)*perimeter_pixel[i-1]),
			   avg, perimeter_pixel[i-1]);*/
	}
	free(perimeter_pixel);
	//printf("\nend ----------------------- CR REMOVE ----------------------------------\n");
}

// CR영역에서 가장 알맞은 radius값을 찾습니다.
int StarBurstHelper::getFitRadius(void)
{
	// if Error Occur or Apply Not Preprocessing..
	if (crx == -1 || cry == -1 || crar == -1)
		return -1;

	double *ratio = (double*)malloc((biggest_crar - crar + 1) * sizeof(double));
	int i, r, r_delta = 1;
	int x, y, x2, y2;
	double sum, sum2;
	//printf("\ncrar1 %d\n", crar);
	for (r = crar; r <= biggest_crar; r++) {
		sum = 0;
		sum2 = 0;
		//printf("angle_num %d\n", angle_num);
		//printf("m_image %d %d %d\n", m_image.size().height, m_image.size().width, m_image.data);
		//printf("crx cry %d %d \n", crx, cry);
		for (i = 0; i < angle_num; i++) {
			x = (int)(crx + (r + r_delta)*cos_array[i]);
			y = (int)(cry + (r + r_delta)*sin_array[i]);
			x2 = (int)(crx + (r - r_delta)*cos_array[i]);
			y2 = (int)(cry + (r + r_delta)*sin_array[i]);

			if ((x >= 0 && y >= 0 && x < m_image.size().width && y < m_image.size().height) &&
				(x2 >= 0 && y2 >= 0 && x2 < m_image.size().width && y2 < m_image.size().height)) {
				sum += *(m_image.data + y * m_image.size().width + x);
				sum2 += *(m_image.data + y2 * m_image.size().width + x2);
				//printf("sum sum2 %d %d \n", sum, sum2);
			}
		}

		ratio[r - crar] = sum / sum2;
		if (r - crar >= 2) {
			if (ratio[r - crar - 2] < ratio[r - crar - 1] && ratio[r - crar] < ratio[r - crar - 1]) {
				free(ratio);
				printf("r-1 %d\n", r - 1);
				return r - 1;
			}
		}
	}

	free(ratio);
	printf("ATTN! fit_circle_radius_to_corneal_reflection() do not change the radius\n");
	printf("crar2 %d\n", crar);
	return crar;
}
