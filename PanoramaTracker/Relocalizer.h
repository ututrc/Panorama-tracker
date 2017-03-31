# pragma once
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include "PtSettings.h"

using namespace cv;
//Class to handle the relocalization that happens when tracking is dropped
class Relocalizer
{
public:
	//Add a new relocalizer image
	void AddRelocImage(Mat image, float x_rot, float y_rot, float z_rot);

	//Run the relocalizer against image, and return the best candidate
	void Relocalize(Mat image, float &x_rot, float &y_rot, float &z_rot, float &quality) const;
	
	//Return info of all relocalization images and corresponding rotations
	void GetRelocalizationInfo(std::vector<Mat> &images, std::vector<float> &x_rotations, std::vector<float> &y_rotations, std::vector<float> &z_rotations) const;
	
	//Add a relocalization point without modifying (resizing) the given image. Used with (deprecated) map save/load
	void AddRelocalizationPointUnmodified(Mat image, float x_rot, float y_rot, float z_rot);
private:

	static const int reloc_image_width_ = 80;
	static const int reloc_image_height_ = 60;

	static const int blur_size_ = 5;

	//Struct that represents a single relocalizer image and its data
	struct RelocalizerPoint
	{
		RelocalizerPoint(){};
		RelocalizerPoint(Mat img, float x, float y, float z) :
		image(img),
		x_angle(x),
		y_angle(y),
		z_angle(z)
		{};

		Mat image;
		float x_angle;
		float y_angle;
		float z_angle;
	};

	//Add reloc point from the given image
	void addRelocalizationPoint(Mat image, float x_rot, float y_rot, float z_rot);
	
	//All stored relocalizer points
	std::vector<RelocalizerPoint> relocalization_points;
	
	//Find the best match for image and return that point
	RelocalizerPoint relocalize(Mat image, float &quality) const;
	
	//Blur the input image 
	void blurImage(Mat image, Mat &result) const;
	
	//Match 2 images of same size and return quality
	float matchTemplates(Mat image1, Mat image2) const;
};