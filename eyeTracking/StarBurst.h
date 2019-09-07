#pragma once
#include "stdafx.h"
#include "StarBurstHelper.h"
#include "Ransac.h"
//#define N 18															// Number of ray for detecting feature point



class StarBurst
{
public:
	StarBurst(Mat image);								// Constructor for StarBurst Algorithme
public:
	~StarBurst(void);
public:
	StarBurstInfo * Apply(void);									// Unique Method for Apply StarBurst Algorithme
public:
	void DetectCoutourPoints(void);							// Step 1: Finding CoutourPoints
	void ApplyRANSAC(void);										// Step 2: Finding 
	StarBurstInfo  info;												// return Structure
	int edge_thresh;
public:
	vector <CvDPoint> edge_point;
	vector <int> edge_intensity_diff;
	StarBurstHelper * helper;
public:	// for Debug ... it is must private
	Mat m_image;											// member variable : source Image
	Mat m_removed_image;											// member variable : source Image
	Rect rect;															// Region Of Interest

public:
	Mat getCornealRemovedImage(void);
public:
	void destroy_edge_point(void);
public:
	void shotRay(int angle, int x, int y, int thresh);
public:
	bool isBorder(int xx, int yy);
	Point calculateCoverage(void);
public:
	int fitDegree(int degree);
};