#include "ballpipeline.h"

namespace grip {

ballpipeline::ballpipeline() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void ballpipeline::Process(cv::Mat& source0){
	//Step CV_resize0:
	//input
	cv::Mat cvResizeSrc = source0;
	cv::Size cvResizeDsize(0, 0);
	double cvResizeFx = 0.5;  // default Double
	double cvResizeFy = 0.5;  // default Double
    int cvResizeInterpolation = cv::INTER_NEAREST;
	cvResize(cvResizeSrc, cvResizeDsize, cvResizeFx, cvResizeFy, cvResizeInterpolation, this->cvResizeOutput);
	//Step Blur0:
	//input
	cv::Mat blurInput = cvResizeOutput;
	BlurType blurType = BlurType::MEDIAN;
	double blurRadius = 5.405405405405405;  // default Double
	blur(blurInput, blurType, blurRadius, this->blurOutput);
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThreshold0Input = blurOutput;
	double hsvThreshold0Hue[] = {100.35971223021585, 130.8532423208191};
	double hsvThreshold0Saturation[] = {128.41726618705036, 255.0};
	double hsvThreshold0Value[] = {59.62230215827338, 255.0};
	hsvThreshold(hsvThreshold0Input, hsvThreshold0Hue, hsvThreshold0Saturation, hsvThreshold0Value, this->hsvThreshold0Output);
	//Step HSV_Threshold1:
	//input
	cv::Mat hsvThreshold1Input = blurOutput;
	double hsvThreshold1Hue[] = {159.87914701904893, 178.99202054275557};
	double hsvThreshold1Saturation[] = {128.41726618705036, 255.0};
	double hsvThreshold1Value[] = {59.62230215827338, 255.0};
	hsvThreshold(hsvThreshold1Input, hsvThreshold1Hue, hsvThreshold1Saturation, hsvThreshold1Value, this->hsvThreshold1Output);
	//Step CV_erode0:
	//input
	cv::Mat cvErode0Src = hsvThreshold0Output;
	cv::Mat cvErode0Kernel;
	cv::Point cvErode0Anchor(-1, -1);
	double cvErode0Iterations = 4.0;  // default Double
    int cvErode0Bordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvErode0Bordervalue(-1);
	cvErode(cvErode0Src, cvErode0Kernel, cvErode0Anchor, cvErode0Iterations, cvErode0Bordertype, cvErode0Bordervalue, this->cvErode0Output);
	//Step CV_erode1:
	//input
	cv::Mat cvErode1Src = hsvThreshold1Output;
	cv::Mat cvErode1Kernel;
	cv::Point cvErode1Anchor(-1, -1);
	double cvErode1Iterations = 4.0;  // default Double
    int cvErode1Bordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvErode1Bordervalue(-1);
	cvErode(cvErode1Src, cvErode1Kernel, cvErode1Anchor, cvErode1Iterations, cvErode1Bordertype, cvErode1Bordervalue, this->cvErode1Output);
	//Step CV_dilate0:
	//input
	cv::Mat cvDilate0Src = cvErode0Output;
	cv::Mat cvDilate0Kernel;
	cv::Point cvDilate0Anchor(-1, -1);
	double cvDilate0Iterations = 4.0;  // default Double
    int cvDilate0Bordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvDilate0Bordervalue(-1);
	cvDilate(cvDilate0Src, cvDilate0Kernel, cvDilate0Anchor, cvDilate0Iterations, cvDilate0Bordertype, cvDilate0Bordervalue, this->cvDilate0Output);
	//Step CV_dilate1:
	//input
	cv::Mat cvDilate1Src = cvErode1Output;
	cv::Mat cvDilate1Kernel;
	cv::Point cvDilate1Anchor(-1, -1);
	double cvDilate1Iterations = 4.0;  // default Double
    int cvDilate1Bordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvDilate1Bordervalue(-1);
	cvDilate(cvDilate1Src, cvDilate1Kernel, cvDilate1Anchor, cvDilate1Iterations, cvDilate1Bordertype, cvDilate1Bordervalue, this->cvDilate1Output);
	//Step Find_Blobs0:
	//input
	cv::Mat findBlobs0Input = cvDilate0Output;
	double findBlobs0MinArea = 400.0;  // default Double
	double findBlobs0Circularity[] = {0.737410071942446, 1.0};
	bool findBlobs0DarkBlobs = false;  // default Boolean
	findBlobs(findBlobs0Input, findBlobs0MinArea, findBlobs0Circularity, findBlobs0DarkBlobs, this->findBlobs0Output);
	//Step Find_Blobs1:
	//input
	cv::Mat findBlobs1Input = cvDilate1Output;
	double findBlobs1MinArea = 400.0;  // default Double
	double findBlobs1Circularity[] = {0.764388489208633, 1.0};
	bool findBlobs1DarkBlobs = false;  // default Boolean
	findBlobs(findBlobs1Input, findBlobs1MinArea, findBlobs1Circularity, findBlobs1DarkBlobs, this->findBlobs1Output);
}

/**
 * This method is a generated getter for the output of a CV_resize.
 * @return Mat output from CV_resize.
 */
cv::Mat* ballpipeline::GetCvResizeOutput(){
	return &(this->cvResizeOutput);
}
/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* ballpipeline::GetBlurOutput(){
	return &(this->blurOutput);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* ballpipeline::GetHsvThreshold0Output(){
	return &(this->hsvThreshold0Output);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* ballpipeline::GetHsvThreshold1Output(){
	return &(this->hsvThreshold1Output);
}
/**
 * This method is a generated getter for the output of a CV_erode.
 * @return Mat output from CV_erode.
 */
cv::Mat* ballpipeline::GetCvErode0Output(){
	return &(this->cvErode0Output);
}
/**
 * This method is a generated getter for the output of a CV_erode.
 * @return Mat output from CV_erode.
 */
cv::Mat* ballpipeline::GetCvErode1Output(){
	return &(this->cvErode1Output);
}
/**
 * This method is a generated getter for the output of a CV_dilate.
 * @return Mat output from CV_dilate.
 */
cv::Mat* ballpipeline::GetCvDilate0Output(){
	return &(this->cvDilate0Output);
}
/**
 * This method is a generated getter for the output of a CV_dilate.
 * @return Mat output from CV_dilate.
 */
cv::Mat* ballpipeline::GetCvDilate1Output(){
	return &(this->cvDilate1Output);
}
/**
 * This method is a generated getter for the output of a Find_Blobs.
 * @return BlobsReport output from Find_Blobs.
 */
std::vector<cv::KeyPoint>* ballpipeline::GetFindBlobs0Output(){
	return &(this->findBlobs0Output);
}
/**
 * This method is a generated getter for the output of a Find_Blobs.
 * @return BlobsReport output from Find_Blobs.
 */
std::vector<cv::KeyPoint>* ballpipeline::GetFindBlobs1Output(){
	return &(this->findBlobs1Output);
}
	/**
	 * Resizes an Image.
	 * @param src The image to resize.
	 * @param dSize size to set the image.
	 * @param fx scale factor along X axis.
	 * @param fy scale factor along Y axis.
	 * @param interpolation type of interpolation to use.
	 * @param dst output image.
	 */
	void ballpipeline::cvResize(cv::Mat &src, cv::Size &dSize, double fx, double fy, int interpolation, cv::Mat &dst) {
		cv::resize(src, dst, dSize, fx, fy, interpolation);
	}

	/**
	 * Softens an image using one of several filters.
	 *
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	void ballpipeline::blur(cv::Mat &input, BlurType &type, double doubleRadius, cv::Mat &output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type) {
			case BOX:
				kernelSize = 2 * radius + 1;
				cv::blur(input,output,cv::Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				cv::medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				cv::bilateralFilter(input, output, -1, radius, radius);
				break;
        }
	}
	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param val The min and max value.
	 * @param output The image in which to store the output.
	 */
	void ballpipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Expands area of lower value in an image.
	 * @param src the Image to erode.
	 * @param kernel the kernel for erosion.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the erosion.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	void ballpipeline::cvErode(cv::Mat &src, cv::Mat &kernel, cv::Point &anchor, double iterations, int borderType, cv::Scalar &borderValue, cv::Mat &dst) {
		cv::erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Expands area of higher value in an image.
	 * @param src the Image to dilate.
	 * @param kernel the kernel for dilation.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the dilation.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	void ballpipeline::cvDilate(cv::Mat &src, cv::Mat &kernel, cv::Point &anchor, double iterations, int borderType, cv::Scalar &borderValue, cv::Mat &dst) {
		cv::dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Detects groups of pixels in an image.
	 *
	 * @param input The image on which to perform the find blobs.
	 * @param minArea The minimum size of a blob that will be found.
	 * @param circularity The minimum and maximum circularity of blobs that will be found.
	 * @param darkBlobs The boolean that determines if light or dark blobs are found.
	 * @param blobList The output where the MatOfKeyPoint is stored.
	 */
	//void findBlobs(Mat *input, double *minArea, double circularity[2],
		//bool *darkBlobs, vector<KeyPoint> *blobList) {
	void ballpipeline::findBlobs(cv::Mat &input, double minArea, double circularity[], bool darkBlobs, std::vector<cv::KeyPoint> &blobList) {
		blobList.clear();
		cv::SimpleBlobDetector::Params params;
		params.filterByColor = 1;
		params.blobColor = (darkBlobs ? 0 : 255);
		params.minThreshold = 10;
		params.maxThreshold = 220;
		params.filterByArea = true;
		params.minArea = minArea;
		params.filterByCircularity = true;
		params.minCircularity = circularity[0];
		params.maxCircularity = circularity[1];
		params.filterByConvexity = false;
		params.filterByInertia = false;
		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		detector->detect(input, blobList);
	}



} // end grip namespace

