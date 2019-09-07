#pragma once
#include "stdafx.h"
#define PI  3.141592653589		// PI value

class StarBurstHelper
{
public:
	StarBurstHelper(Mat image);	// Constructor for StarBurst Helper Class
public:
	~StarBurstHelper(void);
public:
	Mat CornealReflexRemove(StarBurstInfo &info);											// Mothod for Corneal Reflex Remove : Unique *public* modifier
	Rect rect;																										// Region Of Interest
private:
	void CrDetect(void);																							// Step 1 : Finding Corneal Reflex Point
	void CrRemove(StarBurstInfo info);																						// Step 2 : Removing Corneal Reflex by Interporation
	Mat m_image;																						// Gray Image for finding CR
	Mat m_removed_image;																		// Gray Image with CR removed
	Point m_pupil, m_startPoint;
	int crx;																												// Corneal Center x
	int cry;																												// Corneal Center y
	int crar;																											// Corneal Radius

public:
	float angle_delta;						// it is a radian Value of one degree : 0.0174
	int angle_num;		// number of angle : 360
	int biggest_crar;	// bggest radius of r in Image : it would be lower than (height/10)

	double *angle_array;
	double *sin_array;
	double *cos_array;

	int getFitRadius(void);
};
