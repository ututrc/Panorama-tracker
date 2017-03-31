#include "Relocalizer.h"

void Relocalizer::addRelocalizationPoint(Mat image, float x_rot, float y_rot, float z_rot)
{
	//Resize and blur the input images. Small blurred images are "easier" to match
	//using template matching, and consume less memory
	Mat resized_img, blurred_img;
	resize(image, resized_img, Size(reloc_image_width_, reloc_image_height_));
	blurImage(resized_img, blurred_img);
	relocalization_points.push_back(RelocalizerPoint(blurred_img, x_rot, y_rot, z_rot));
}

void Relocalizer::AddRelocImage(Mat image, float x_rot, float y_rot, float z_rot)
{
	addRelocalizationPoint(image, x_rot, y_rot, z_rot);
}

void Relocalizer::Relocalize(Mat image, float& x_rot, float& y_rot, float& z_rot, float &quality) const
{
	//Get the best match and return it
	RelocalizerPoint pt = relocalize(image, quality);
	x_rot = pt.x_angle;
	y_rot = pt.y_angle;
	z_rot = pt.z_angle;
}

void Relocalizer::GetRelocalizationInfo(std::vector<Mat>& images, std::vector<float>& x_rotations, std::vector<float>& y_rotations, std::vector<float>& z_rotations) const
{
	for (size_t i = 0; i < relocalization_points.size(); i++)
	{
		images.push_back(relocalization_points.at(i).image);
		x_rotations.push_back(relocalization_points.at(i).x_angle);
		y_rotations.push_back(relocalization_points.at(i).y_angle);
		z_rotations.push_back(relocalization_points.at(i).z_angle);
	}
}

void Relocalizer::AddRelocalizationPointUnmodified(Mat image, float x_rot, float y_rot, float z_rot)
{
	relocalization_points.push_back(RelocalizerPoint(image, x_rot, y_rot, z_rot));
}

Relocalizer::RelocalizerPoint Relocalizer::relocalize(Mat image, float &quality) const
{
	//Blur and resize similar to when adding images
	Mat resized_img, blurred_img;
	resize(image, resized_img, Size(reloc_image_width_, reloc_image_height_));
	blurImage(resized_img, blurred_img);

	//Match with all points and pick the best match
	float best_quality = 100;
	RelocalizerPoint best_point;
	for (size_t i = 0; i < relocalization_points.size(); i++)
	{
		float q = matchTemplates(blurred_img, relocalization_points.at(i).image);
		if (q < best_quality)
		{
			best_quality = q;
			best_point = relocalization_points.at(i);
		}
	}

	quality = best_quality;
	return best_point;
}

void Relocalizer::blurImage(Mat image, Mat &result) const
{
	Mat blurred_image;
	blur(image, result, Size(blur_size_, blur_size_));
}

float Relocalizer::matchTemplates(Mat image1, Mat image2) const
{
	float quality;
	Mat result;

	matchTemplate(image1, image2, result, CV_TM_SQDIFF_NORMED);
	double min_val; double max_val; Point min_loc; Point max_loc;
	Point match_loc;
	minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc, Mat());
	match_loc = min_loc;
	quality = min_val;
	return quality;
}