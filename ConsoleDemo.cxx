////////////////////////////////////////////////////////////////////////////////
// SoftKinetic DepthSense SDK
//
// COPYRIGHT AND CONFIDENTIALITY NOTICE - SOFTKINETIC CONFIDENTIAL
// INFORMATION
//
// All rights reserved to SOFTKINETIC SENSORS NV (a
// company incorporated and existing under the laws of Belgium, with
// its principal place of business at Boulevard de la Plainelaan 15,
// 1050 Brussels (Belgium), registered with the Crossroads bank for
// enterprises under company number 0811 341 454 - "Softkinetic
// Sensors").
//
// The source code of the SoftKinetic DepthSense Camera Drivers is
// proprietary and confidential information of Softkinetic Sensors NV.
//
// For any question about terms and conditions, please contact:
// info@softkinetic.com Copyright (c) 2002-2015 Softkinetic Sensors NV
////////////////////////////////////////////////////////////////////////////////


#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <vector>
#include <exception>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <DepthSense.hxx>

using namespace DepthSense;
using namespace std;
using namespace cv;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;

IplImage
*g_depthImage = NULL,
*g_videoImage = NULL; // initialized in main, used in CBs



CvSize
g_szDepth = cvSize(320, 240), // QVGA
g_szVideo = cvSize(640, 480); //VGA

Mat depthImage(g_szDepth, CV_32FC1);
Mat videoImage;

int left_offset = 9;
int right_offset = 56;
int top_offset = 32;
int bottom_offset = 26;
int depth_threshold = 75;

bool running = 1;

bool g_saveImageFlag = false, g_saveDepthFlag = false;

void yuy2rgb(unsigned char *dst, const unsigned char *src, const int width, const int height) {
	int x, y;
	const int width2 = width * 2;
	const int width4 = width * 3;
	const unsigned char *src1 = src;
	unsigned char *dst1 = dst;

	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x += 2) {
			int x2 = x * 2;
			int y1 = src1[x2];
			int y2 = src1[x2 + 2];
			int u = src1[x2 + 1] - 128;
			int v = src1[x2 + 3] - 128;
			int uvr = (15748 * v) / 10000;
			int uvg = (-1873 * u - 4681 * v) / 10000;
			int uvb = (18556 * u) / 10000;

			int x4 = x * 3;
			int r1 = y1 + uvr;
			int r2 = y2 + uvr;
			int g1 = y1 + uvg;
			int g2 = y2 + uvg;
			int b1 = y1 + uvb;
			int b2 = y2 + uvb;

			dst1[x4 + 0] = (b1 > 255) ? 255 : ((b1 < 0) ? 0 : b1);
			dst1[x4 + 1] = (g1 > 255) ? 255 : ((g1 < 0) ? 0 : g1);
			dst1[x4 + 2] = (r1 > 255) ? 255 : ((r1 < 0) ? 0 : r1);
			//dst1[x4+3] = 255;

			dst1[x4 + 3] = (b2 > 255) ? 255 : ((b2 < 0) ? 0 : b2);
			dst1[x4 + 4] = (g2 > 255) ? 255 : ((g2 < 0) ? 0 : g2);
			dst1[x4 + 5] = (r2 > 255) ? 255 : ((r2 < 0) ? 0 : r2);
		}
		src1 += width2;
		dst1 += width4;
	}
}

void mjpegrgb(unsigned char *dst, const unsigned char *src, const int width, const int height) {
	int z;
	const unsigned char *src1 = src;
	unsigned char *dst1 = dst;
	for (z = 0; z < width*height * 3; z++) {
		dst1[z] = src1[z];
	}
}

/*----------------------------------------------------------------------------*/
// New audio sample event handler
void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
{
	//printf("A#%u: %d\n",g_aFrames,data.audioData.size());
	g_aFrames++;
}

/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
	int32_t w, h;
	FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);
	if (running)
		mjpegrgb((unsigned char *)g_videoImage->imageData, data.colorMap, w, h);
}

/*----------------------------------------------------------------------------*/
// New depth sample event handler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
	//printf("Z#%u: %d\n",g_dFrames,data.vertices.size());
	int32_t w, h;

	if (running) {
		FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);
		int count = 0; // DS data index
		if (data.depthMapFloatingPoint != 0) {// just in case !
			for (int i = 0; i < h; i++) {
				for (int j = 0; j < w; j++) {
					// some arbitrary scaling to make this visible
					float val = data.depthMapFloatingPoint[count++];
					if (!g_saveImageFlag && !g_saveDepthFlag)
						val *= 200;
					if (val < 0)
						val = 255; // catch the saturated points
					cvSet2D(g_depthImage, i, j, cvScalar(val));
				}
			}
		}
		else {
			printf("no fp\n");
		}
	}

	double rate = 1.0;
	videoImage = cvarrToMat(g_videoImage, false);
	Mat temp;
	depthImage = cvarrToMat(g_depthImage, false);
	depthImage = depthImage.colRange(left_offset, depthImage.cols - right_offset);
	depthImage = depthImage.rowRange(top_offset, depthImage.rows - bottom_offset);
	resize(depthImage, depthImage, g_szDepth);
	//cvarrToMat(g_depthImage, false).convertTo(temp,CV_32FC1);
	//depthImage = depthImage*(1-rate);
	//depthImage = depthImage + temp*(rate);

	Mat videoDown = Mat(g_szDepth, CV_8UC3);
	Mat depthMask = Mat(depthImage.size(), CV_32FC1);
	threshold(depthImage, depthMask, depth_threshold, 255, THRESH_BINARY_INV);

	Mat kernel = Mat::ones(4, 4, CV_8UC1);

	erode(depthMask, depthMask, kernel);
	resize(videoImage, videoDown, videoDown.size());

	Mat rgbMask;
	cvtColor(depthMask, rgbMask, CV_GRAY2RGB);
	rgbMask = rgbMask *(1.0 / 255);
	//depthImage = cvarrToMat(g_depthImage, false);
	imshow("Video", videoDown);
	imshow("Depth", depthImage);
	imshow("DepthMask", depthMask);
	imshow("Masked", videoDown.mul(rgbMask));

	// Quit the main loop after 200 depth frames received
	char key = cvWaitKey(10);
	if (key == 27) {
		printf("Quitting main loop from OpenCV\n");
		g_context.quit();
	}
	else if (key == 'w') {
		if (left_offset >= 1)
			left_offset -= 1;
	}
	else if (key == 'q') {
		left_offset += 1;
	}
	else if (key == 'a') {
		if (right_offset >= 1)
			right_offset -= 1;
	}
	else if (key == 's') {
		right_offset += 1;
	}
	else if (key == 'd') {
		if (top_offset >= 1)
			top_offset -= 1;
	}
	else if (key == 'e') {
		top_offset += 1;
	}
	else if (key == 'r') {
		if (bottom_offset >= 1)
			bottom_offset -= 1;
	}
	else if (key == 'f') {
		bottom_offset += 1;
	}
	else if (key == 't') {
		depth_threshold += 1;
	}
	else if (key == 'g') {
		depth_threshold -= 1;
	}
	else if (key == 'm') {
		printf("Left: %d, Right: %d, Top %d, Bottom %d\n", left_offset, right_offset, top_offset, bottom_offset);
	}
	else if (key == ' ') {
		printf("Toggled Pause\n");
		running = !running;
	}
	else if (key == 'p') {
		char filename[100];
		int g_fTime = clock();
		sprintf(filename, "savedImages/df%d.%d.bmp", (int)(g_fTime / CLOCKS_PER_SEC), (int)(g_fTime%CLOCKS_PER_SEC));
		imwrite(filename, depthImage);
		sprintf(filename, "savedImages/vf%d.%d.jpg", (int)(g_fTime / CLOCKS_PER_SEC), (int)(g_fTime%CLOCKS_PER_SEC));
		imwrite(filename, videoDown);
		sprintf(filename, "savedImages/mf%d.%d.bmp", (int)(g_fTime / CLOCKS_PER_SEC), (int)(g_fTime%CLOCKS_PER_SEC));
		imwrite(filename, depthMask);
	}
}

/*----------------------------------------------------------------------------*/
void configureAudioNode()
{
	g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

	AudioNode::Configuration config = g_anode.getConfiguration();
	config.sampleRate = 44100;

	try
	{
		g_context.requestControl(g_anode, 0);

		g_anode.setConfiguration(config);

		g_anode.setInputMixerLevel(0.5f);
	}
	catch (ArgumentException& e)
	{
		printf("Argument Exception: %s\n", e.what());
	}
	catch (UnauthorizedAccessException& e)
	{
		printf("Unauthorized Access Exception: %s\n", e.what());
	}
	catch (ConfigurationException& e)
	{
		printf("Configuration Exception: %s\n", e.what());
	}
	catch (StreamingException& e)
	{
		printf("Streaming Exception: %s\n", e.what());
	}
	catch (TimeoutException&)
	{
		printf("TimeoutException\n");
	}
}

/*----------------------------------------------------------------------------*/
void configureDepthNode()
{
	g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);

	DepthNode::Configuration config = g_dnode.getConfiguration();
	config.frameFormat = FRAME_FORMAT_QQVGA;
	config.framerate = 60;
	config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
	config.saturation = true;
	//g_dnode.setEnableVertices(true);
	g_dnode.setEnableDepthMapFloatingPoint(true);


	try
	{
		g_context.requestControl(g_dnode, 0);

		g_dnode.setConfiguration(config);
	}
	catch (ArgumentException& e)
	{
		printf("Argument Exception: %s\n", e.what());
	}
	catch (UnauthorizedAccessException& e)
	{
		printf("Unauthorized Access Exception: %s\n", e.what());
	}
	catch (IOException& e)
	{
		printf("IO Exception: %s\n", e.what());
	}
	catch (InvalidOperationException& e)
	{
		printf("Invalid Operation Exception: %s\n", e.what());
	}
	catch (ConfigurationException& e)
	{
		printf("Configuration Exception: %s\n", e.what());
	}
	catch (StreamingException& e)
	{
		printf("Streaming Exception: %s\n", e.what());
	}
	catch (TimeoutException&)
	{
		printf("TimeoutException\n");
	}

}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
	// connect new color sample handler
	g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

	ColorNode::Configuration config = g_cnode.getConfiguration();
	config.frameFormat = FRAME_FORMAT_VGA;
	config.compression = COMPRESSION_TYPE_MJPEG;

	//config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
	//config.framerate = 25;

	g_cnode.setEnableColorMap(true);

	try
	{
		g_context.requestControl(g_cnode, 0);

		g_cnode.setConfiguration(config);
	}
	catch (ArgumentException& e)
	{
		printf("Argument Exception: %s\n", e.what());
	}
	catch (UnauthorizedAccessException& e)
	{
		printf("Unauthorized Access Exception: %s\n", e.what());
	}
	catch (IOException& e)
	{
		printf("IO Exception: %s\n", e.what());
	}
	catch (InvalidOperationException& e)
	{
		printf("Invalid Operation Exception: %s\n", e.what());
	}
	catch (ConfigurationException& e)
	{
		printf("Configuration Exception: %s\n", e.what());
	}
	catch (StreamingException& e)
	{
		printf("Streaming Exception: %s\n", e.what());
	}
	catch (TimeoutException&)
	{
		printf("TimeoutException\n");
	}
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
	if ((node.is<DepthNode>()) && (!g_dnode.isSet()))
	{
		g_dnode = node.as<DepthNode>();
		configureDepthNode();
		g_context.registerNode(node);
	}

	if ((node.is<ColorNode>()) && (!g_cnode.isSet()))
	{
		g_cnode = node.as<ColorNode>();
		configureColorNode();
		g_context.registerNode(node);
	}

	if ((node.is<AudioNode>()) && (!g_anode.isSet()))
	{
		g_anode = node.as<AudioNode>();
		configureAudioNode();
		g_context.registerNode(node);
	}
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
	configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
	if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode))
		g_anode.unset();
	if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
		g_cnode.unset();
	if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
		g_dnode.unset();
	printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
	if (!g_bDeviceFound)
	{
		data.device.nodeAddedEvent().connect(&onNodeConnected);
		data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
		g_bDeviceFound = true;
	}
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
	g_bDeviceFound = false;
	printf("Device disconnected\n");
}

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
	g_context = Context::create("localhost");

	g_context.deviceAddedEvent().connect(&onDeviceConnected);
	g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

	// Get the list of currently connected devices
	vector<Device> da = g_context.getDevices();

	// We are only interested in the first device
	if (da.size() >= 1)
	{
		g_bDeviceFound = true;

		da[0].nodeAddedEvent().connect(&onNodeConnected);
		da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

		vector<Node> na = da[0].getNodes();

		printf("Found %u nodes\n", na.size());

		for (int n = 0; n < (int)na.size(); n++)
			configureNode(na[n]);
	}

	// VGA format color image
	g_videoImage = cvCreateImage(g_szVideo, IPL_DEPTH_8U, 3);
	if (g_videoImage == NULL)
	{
		printf("Unable to create video image buffer\n"); exit(0);
	}

	// QVGA format depth image
	g_depthImage = cvCreateImage(g_szDepth, IPL_DEPTH_8U, 1);
	if (g_depthImage == NULL)
	{
		printf("Unable to create depth image buffer\n"); exit(0);
	}

	printf("dml@Fordham version of DS ConsoleDemo. June 2013.\n");
	printf("Click onto in image for commands. ESC to exit.\n");
	printf("Use \'W\' to toggle dumping of depth and visual images.\n");
	printf("Use \'w\' to toggle dumping of depth images only.\n\n");

	g_context.startNodes();


	g_context.run();



	if (g_pProjHelper)
		delete g_pProjHelper;

	/*Mat image;

	image = imread("./targetGestures.png", CV_LOAD_IMAGE_COLOR);   // Read the file

	if (!image.data)                              // Check for invalid input
	{
	cout << "Could not open or find the image" << std::endl;
	return -1;
	}
	cout << image;
	cout << image.size();

	namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
	imshow("Display window", image);                   // Show our image inside it.

	waitKey(0);*/

	g_context.stopNodes();

	if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
	if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
	if (g_anode.isSet()) g_context.unregisterNode(g_anode);

	return 0;
}
