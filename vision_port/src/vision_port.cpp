/*------------------------------------------------------------------------------
 *  Title:        vision_port.cpp
 *  Description:  ROS node for subscribing to images.
 *----------------------------------------------------------------------------*/

/*
 *
 *      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the Stingray, iBotics nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "vision_port.h"

// Boost algorithms
#include "boost.h"
#include "gate.cpp"
#include "buoy.cpp"
#include "pipe.cpp"
#include "hedge.cpp"

typedef actionlib::SimpleActionServer<vision_port::visionAction> ActionServer;

/*------------------------------------------------------------------------------
 * VisionPort()
 * Constructor.
 *----------------------------------------------------------------------------*/

VisionPort::VisionPort()
{
} // end VisionPort()


/*------------------------------------------------------------------------------
 * ~VisionPort()
 * Destructor.
 *----------------------------------------------------------------------------*/

VisionPort::~VisionPort()
{
} // end ~VisionPort()


/*---------------------------------------------------------------------
* imageCallback()
* Callback for when an image arrives.
* -------------------------------------------------------------------*/

void VisionPort::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  try
  {
    cv_input = bridge.imgMsgToCv(msg, "bgr8");
  }        
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  if (VisionPort::image_to_view == 0)
  {
  }
  else if (VisionPort::image_to_view == 1) // vision_find_buoy()
  {

    // Create pointer to CvMat input image
    img_in_ptr = cv_input;

    // Create pointer to CvMat bin image
    bin_img_ptr = &bin_img;

    int dotx = 10000;
    int doty = 10000;
    
    int* dotx_ptr = &dotx;
    int* doty_ptr = &doty;

    // Find the buoy in the image
    vision_find_buoy(dotx_ptr, doty_ptr, 1);// look for red

    cvCircle(img_in_ptr, cvPoint(dotx, doty), 10, cvScalar(0, 255, 0), 5, 8);

    cvShowImage ("bin", bin_img_ptr);
    cvShowImage ("view", img_in_ptr);
    cvShowImage ("hsv", img_hsv_ptr);        

    cvReleaseImage(&img_hsv_ptr);
    cvReleaseImage(&bin_img_ptr);
  }
  else if (VisionPort::image_to_view == 2) // vision_find_pipe()
  {
    // Create pointer to CvMat input image
    img_in_ptr = cv_input;

    // Create pointer to CvMat bin image
    bin_img_ptr = &bin_img;

    CvPoint center[2];
    double bearing[4];

    vision_find_pipe(center, bearing);

    // Draw a circle at the centroid location.
    cvCircle(img_in_ptr, center[0], 10, cvScalar(255, 0, 0), 5, 8);

    // Draw the bearing at the centeroid.
    float d = sqrt(bearing[0] * bearing[0] + bearing[1] * bearing[1]);

    cvLine(img_in_ptr, center[0], cvPoint(cvRound(center[0].x - (bearing[0] / d)*25.0), cvRound(center[0].y - (bearing[1] / d)*25.0)),
           cvScalar(255, 0, 0), 3, 8);

    if (center[1].x > 0 && center[1].y > 0 && bearing[2] != -10000.0 && bearing[3] != -10000.0)
    {
      // Draw a circle at the centroid location.
      cvCircle(img_in_ptr, center[1], 10, cvScalar(0, 0, 255), 5, 8);

      // Draw the bearing at the centeroid.
      d = sqrt(bearing[2] * bearing[2] + bearing[3] * bearing[3]);
      cvLine(img_in_ptr, center[1], cvPoint(cvRound(center[1].x - (bearing[2] / d)*25.0), cvRound(center[1].y - (bearing[3] / d)*25.0)),
             cvScalar(0, 0, 255), 3, 8);
    }

    cvShowImage("bin", bin_img_ptr);
    cvShowImage("view", img_in_ptr);
    cvShowImage("hsv", img_hsv_ptr);        

    cvReleaseImage(&img_hsv_ptr);
    cvReleaseImage(&bin_img_ptr);
  }
  else if (VisionPort::image_to_view == 3) // vision_find_hedge()
  {
    // Create pointer to CvMat input image
    img_in_ptr = cv_input;

    // Create pointer to CvMat bin image
    bin_img_ptr = &bin_img;

    int dotx = 10000;
    int doty = 10000;
    
    int* dotx_ptr = &dotx;
    int* doty_ptr = &doty;

    // Find the buoy in the image
    vision_find_hedge(dotx_ptr, doty_ptr);

    cvCircle(img_in_ptr, cvPoint(dotx, doty), 10, cvScalar(0, 255, 0), 5, 8);

    cvShowImage("bin", bin_img_ptr);
    cvShowImage("view", img_in_ptr);
    cvShowImage("hsv", img_hsv_ptr);        

    cvReleaseImage(&img_hsv_ptr);
    cvReleaseImage(&bin_img_ptr);
  }
} // end imageCallback()


/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/

void VisionPort::configCallback(vision_port::visionPortParamsConfig &config, uint32_t level)
{
    // Set class variables to new values.
    boost_wanted  = config.boost_wanted;
    image_to_view = config.image_to_view;
    h_low         = config.h_low;
    h_high        = config.h_high;
    s_low         = config.s_low;
    s_high        = config.s_high;
    v_low         = config.v_low;
    v_high        = config.v_high;
    ROS_WARN("reconfiguring image_to_view = %d", image_to_view);
} // end configCallback()


/*------------------------------------------------------------------------------
 * execute()
 * Callback function for actionlib server.
 *----------------------------------------------------------------------------*/

void execute(const vision_port::visionGoalConstPtr &goal, ActionServer *as)
{
    as->setSucceeded();
} // end execute()


/*---------------------------------------------------------------------
* main()
* Main function for ROS node.
* -------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "vision_port");
    ros::NodeHandle n;

    // Set up a VisionPort object.
    VisionPort *vision_port = new VisionPort();

    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<vision_port::visionPortParamsConfig> gain_srv;
    dynamic_reconfigure::Server<vision_port::visionPortParamsConfig>::CallbackType f;
    f = boost::bind(&VisionPort::configCallback, vision_port, _1, _2);
    gain_srv.setCallback(f);

    // Initialize node parameters.
    int rate;
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("image_to_view", vision_port->image_to_view, int(1));
    private_node_handle_.param("rate",          rate,                       int(30));
    
    // Create window for binary image.
    int image_previous = -1;
    cvNamedWindow("bin");
    cvStartWindowThread();
    // Create a window and give it a name.
    cvNamedWindow("view");
    cvStartWindowThread();
    // Start showing the window.
    cvNamedWindow("hsv");
    cvStartWindowThread();

    // Create an actionlib server.
    ActionServer vision_as(n, "vision", boost::bind(&execute, _1, &vision_as), false);
    vision_as.start();

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);
    
    // Main loop.
    while (n.ok())
    {
        // Prepare to receive images.
        image_transport::ImageTransport it(n);

        // Set up an image subscriber based on "planner task".
        if (vision_port->image_to_view == 0)
        {
            if (image_previous != 0)
            {
                ROS_WARN("Process subscribing to a image file.");
                vision_port->sub = it.subscribe("", 1, &VisionPort::imageCallback, vision_port);            
            }
        }
        else if (vision_port->image_to_view == 1) // vision_find_buoy()
        {
            if (image_previous != 1)
            {
                ROS_WARN("Subscribing to left image, for buoy find algorithm.");
                vision_port->sub = it.subscribe("camera_left/image_rect_color", 1, &VisionPort::imageCallback, vision_port);
            }    
        }
        else if (vision_port->image_to_view == 2) // vision_find_pipe()
        {
            if (image_previous != 2)
            {
                ROS_WARN("Subscribing to right image, for pipe find algorithm.");
                vision_port->sub = it.subscribe("camera_right/image_rect_color", 1, &VisionPort::imageCallback, vision_port);
            }
        }
        else if (vision_port->image_to_view == 3) // vision_find_hedge()
        {
            if (image_previous != 3)
            {
                ROS_WARN("Subscribing to right image, for hedge find algorithm.");
                vision_port->sub = it.subscribe("camera_right/image_rect_color", 1, &VisionPort::imageCallback, vision_port);
            }
        }
        else // Case not used by our system
        {
            if (image_previous == 0 || image_previous == 1 || image_previous == 2 || image_previous == 3)
            {
                ROS_WARN("Value imageToView set to a parameter not used by the system.");
                vision_port->sub = it.subscribe("", 1, &VisionPort::imageCallback, vision_port);            
            }
        }

        image_previous = vision_port->image_to_view;

        ros::spinOnce();
        r.sleep();
    }

    // Close window.
    cvDestroyWindow("bin");
    // Close window.
    cvDestroyWindow("view");
    // Close window.
    cvDestroyWindow("hsv");

    return 0;
} // end main()


/*---------------------------------------------------------------------
* int vision_find_buoy()
*   int *dotx        -
*   int *doty        - 
*   IplImage *src_img - input image
*   IplImage *bin_img - output image
* Process a buoy in an image
* -------------------------------------------------------------------*/
int VisionPort::vision_find_buoy(int *dotx, int *doty, int color)
{
    CvPoint center;
    IplConvKernel *wS = cvCreateStructuringElementEx(3, 3,
                        (int)floor((3.0) / 2), (int)floor((3.0) / 2), CV_SHAPE_RECT);
    
    int num_pix = 0;
    int touch_thresh = 0;
    int detect_thresh = 0;

    // Initialize to impossible values.
    center.x = -10000;
    center.y = -10000;

    // Create intermediate images for scratch space.
    img_hsv_ptr = cvCreateImage(cvGetSize(img_in_ptr), IPL_DEPTH_8U, 3);
    bin_img_ptr = cvCreateImage(cvGetSize(img_in_ptr), IPL_DEPTH_8U, 1);
    IplImage *out_img_ptr = cvCreateImage(cvGetSize(img_in_ptr), IPL_DEPTH_8U, 1);

    // Convert the image from RGB to HSV color space.
    cvCvtColor(img_in_ptr, img_hsv_ptr, CV_BGR2HSV);

    if (BUOY_TECHNIQUE == boost_wanted)
    {
        // Setup thresholds
        touch_thresh = 15000;
        detect_thresh = 0;

        // Buoy Boost Technique
        vision_boost_buoy(img_hsv_ptr, bin_img_ptr, &center, color);
    }
    else
    {
        // Setup thresholds
        touch_thresh = 150000;
        detect_thresh = 40;

        // Threshold all three channels using our own values.
        cvInRangeS(img_hsv_ptr, cvScalar(h_low, s_low, v_low),
                   cvScalar(h_high, s_high, v_high), bin_img_ptr);

        // Use a median filter image to remove outliers.
        cvSmooth(bin_img_ptr, out_img_ptr, CV_MEDIAN, 7, 7, 0. , 0.);
        cvMorphologyEx(bin_img_ptr, bin_img_ptr, wS, NULL, CV_MOP_CLOSE, 1);

        // Find the centroid.
        center = vision_find_centroid(bin_img_ptr, 5);
    }

    // Set the found center of the dot
    *dotx = center.x;
    *doty = center.y;

    cvReleaseImage(&out_img_ptr);

    // Check to see how many pixels of are detected in the image.
    num_pix = cvCountNonZero(bin_img_ptr);

    if (num_pix > touch_thresh)
    {
        return 2;
    }

    if (num_pix <= detect_thresh)
    {
        return -1;
    }

    // Check that the values of dotx & doty are not negative.
    if (dotx < 0 || doty < 0)
    {
        return 0;
    }

    // Process complete
    return 1;
} // vision_find_buoy()


/*------------------------------------------------------------------------------
 * int vision_boost_buoy()
 * Creates a binary image using the boosting predictor.
 * Finds the center of the buoy in the image.
 *----------------------------------------------------------------------------*/

int VisionPort::vision_boost_buoy(IplImage *src_img, IplImage *bin_img, CvPoint *center, int color)
{
    // Declare variables.
    unsigned char *src_data = (unsigned char*)src_img->imageData;
    unsigned char *bin_data = (unsigned char*)bin_img->imageData;
    double dst[2] = {0, 0};
    double thresh = 1.0;
    int i = 0, j = 0, res = 0;
    double h = 0.0, s = 0.0, v = 0.0;
    double* hsv[3] = {&h, &s, &v};
    IplConvKernel* B = NULL;
    static CvMemStorage* mem_storage = NULL;
    static CvSeq* contours = NULL;
    CvContourScanner scanner;
    CvSeq* c = NULL;
    CvSeq* c_new = NULL;
    double c_area = 0.0, c_max = 0.0, c_max2 = 0.0;
    CvMoments moments;
    double M00 = 0.0, M01 = 0.0, M10 = 0.0;
    CvPoint c_center, c_center2;
    c_center.x = -1, c_center.y = -1;
    c_center2.x = -1, c_center2.y = -1;

    // Create Morphological Kernel
    B = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT);

    // Loop through the first image to fill the left part of the new image.
    for (i = 0; i < src_img->height; i++)
    {
        for (j = 0; j < src_img->width; j++)
        {
            *hsv[0] = src_data[i * src_img->widthStep + j * src_img->nChannels + 0];
            *hsv[1] = src_data[i * src_img->widthStep + j * src_img->nChannels + 1];
            *hsv[2] = src_data[i * src_img->widthStep + j * src_img->nChannels + 2];

			if (color == VISION_BUOY_RED)
			{
				// Predict on this point
				if (predict_buoy_transdec((void**)hsv, dst))
				{
					if (dst[1] > thresh)
					{
						res = 0xff;
					}
					else
					{
						// Secondary Prediction on this point
						if (predict_buoy_pool((void**)hsv, dst))
						{
							if (dst[1] > thresh)
							{
								res = 0xff;
							}
							else
							{
								res = 0;
							}
						}
						else
						{
							fprintf(stderr, "%s(): Secondary Prediction Fail.\n", __FUNCTION__);
						}
					}
				}
				else
				{
					fprintf(stderr, "%s(): Prediction Fail.\n", __FUNCTION__);
				}
			}
			else if (color == VISION_BUOY_YELLOW)
			{
				// Predict on this point
				if (predict_buoy_y((void**)hsv, dst))
				{
					if (dst[1] > thresh)
					{
						res = 0xff;
					}
					else
					{
						res = 0;
					}
				}
				else
				{
					fprintf(stderr, "%s(): Prediction Fail.\n", __FUNCTION__);
				}
			}
			else if (color == VISION_BUOY_GREEN)
			{
				// Predict on this point
				if (predict_buoy_g((void**)hsv, dst))
				{
					if (dst[1] > thresh)
					{
						res = 0xff;
					}
					else
					{
						res = 0;
					}
				}
				else
				{
					fprintf(stderr, "%s(): Prediction Fail.\n", __FUNCTION__);
				}
			}
			else
			{
				fprintf(stderr, "%s(): No Color Specified.\n", __FUNCTION__);
			}

            bin_data[i * bin_img->widthStep + j * bin_img->nChannels + 0] = res;
            bin_data[i * bin_img->widthStep + j * bin_img->nChannels + 1] = res;
            bin_data[i * bin_img->widthStep + j * bin_img->nChannels + 2] = res;
        }
    }

    // Use Opening to remove noise
    cvMorphologyEx(bin_img, bin_img, NULL, B, CV_MOP_OPEN, 1);

    // Use Closing to fill in blobs
    cvMorphologyEx(bin_img, bin_img, NULL, B, CV_MOP_CLOSE, 2);

    // Use smooth to smooth the shapes
    cvSmooth(bin_img, bin_img, CV_MEDIAN, 7, 7, 0., 0.);

    // Find Countours around remaining regions
    if (mem_storage == NULL)
    {
        mem_storage = cvCreateMemStorage(0);
    }
    else
    {
        cvClearMemStorage(mem_storage);
    }

    scanner = cvStartFindContours(bin_img, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    while ((c = cvFindNextContour(scanner)) != NULL)
    {
        // Convex Hull Contour Approximation
        c_new = cvConvexHull2(c, mem_storage, CV_CLOCKWISE, 1);
        cvSubstituteContour(scanner, c_new);
    }
    contours = cvEndFindContours(&scanner);

    // Draw the resulting contours
    for (c = contours; c != NULL; c = c->h_next)
    {
        cvDrawContours(bin_img, c, CV_RGB(0xff, 0xff, 0xff), CV_RGB(0x00, 0x00, 0x00), -1, CV_FILLED, 8);
    }

    // Dilate because convex hull shrinks the contour
    cvDilate(bin_img, bin_img, B, 1);

    // Use smooth to smooth the shapes again
    cvSmooth(bin_img, bin_img, CV_MEDIAN, 7, 7, 0., 0.);

    // Find Countours around remaining regions
    if (mem_storage == NULL)
    {
        mem_storage = cvCreateMemStorage(0);
    }
    else
    {
        cvClearMemStorage(mem_storage);
    }
    scanner = cvStartFindContours(bin_img, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    while ((c = cvFindNextContour(scanner)) != NULL)
    {
        cvDrawContours(bin_img, c, CV_RGB(0xff, 0xff, 0xff), CV_RGB(0x00, 0x00, 0x00), -1, CV_FILLED, 8);

        // Get this contours area
        c_area = abs(cvContourArea(c, CV_WHOLE_SEQ));

        if (c_area > c_max)
        {
            // Set old max to be second oldest
            c_max2 = c_max;
            c_center2.x = c_center.x;
            c_center2.y = c_center.y;

            // Set max area
            c_max = c_area;

            // Get this contours center
            cvContourMoments(c, &moments);
            M00 = cvGetSpatialMoment(&moments, 0, 0);
            M10 = cvGetSpatialMoment(&moments, 1, 0);
            M01 = cvGetSpatialMoment(&moments, 0, 1);
            c_center.x = (int)(M10 / M00);
            c_center.y = (int)(M01 / M00);
        }
        else if (c_area > c_max2)
        {
            // Set second max area
            c_max2 = c_area;

            // Get this contours center
            cvContourMoments(c, &moments);
            M00 = cvGetSpatialMoment(&moments, 0, 0);
            M10 = cvGetSpatialMoment(&moments, 1, 0);
            M01 = cvGetSpatialMoment(&moments, 0, 1);
            c_center2.x = (int)(M10 / M00);
            c_center2.y = (int)(M01 / M00);
        }
    }
    contours = cvEndFindContours(&scanner);

    // If area is within range and second largest is above then pick the second
    if (c_max2 > 0 && c_max - c_max2 < 400 && c_center2.y < c_center.y)
    {
        center->x = c_center2.x;
        center->y = c_center2.y;
    }
    else if (c_center.x != -1 && c_center.y != -1)
    {
        center->x = c_center.x;
        center->y = c_center.y;
    }

    cvReleaseStructuringElement(&B);

    return 1;
} // end vision_boost_buoy()


/*------------------------------------------------------------------------------
 * int vision_find_pipe()
 * Finds a orange pipe in an image.
 *----------------------------------------------------------------------------*/
int VisionPort::vision_find_pipe(CvPoint center[2], double bearing[4])
{
    int detect_thresh = 0;
    int num_pix = 0;

    // Initialize to impossible values.
    center[0].x = -10000, center[0].y = -10000;
    center[1].x = -10000, center[1].y = -10000;

    // Create intermediate images for scratch space.
    img_hsv_ptr = cvCreateImage(cvGetSize(img_in_ptr), IPL_DEPTH_8U, 3);
    bin_img_ptr = cvCreateImage(cvGetSize(img_in_ptr), IPL_DEPTH_8U, 1);
    IplImage *out_img_ptr = cvCreateImage(cvGetSize(img_in_ptr), IPL_DEPTH_8U, 1);

    // Segment the flipped image into a binary image.
    cvCvtColor(img_in_ptr, img_hsv_ptr, CV_BGR2HSV);

    if (PIPE_TECHNIQUE == boost_wanted)
    {
        // Setup thresholds
        detect_thresh = 0;

        // Pipe Boost Technique
        vision_boost_pipe(img_hsv_ptr, bin_img_ptr, center, bearing);
    }
    else
    {
        // Setup thresholds
        detect_thresh = 1500;

        // Threshold all three channels using our own values.
        cvInRangeS(img_hsv_ptr, cvScalar(h_low, s_low, v_low),
                   cvScalar(h_high, s_high, v_high), bin_img_ptr);

        // Use a median filter to remove outliers.
        cvSmooth(bin_img_ptr, out_img_ptr, CV_MEDIAN, 5, 5, 0. , 0.);

        // Process the image to get the bearing and centroid.
        bearing[0] = vision_get_bearing(out_img_ptr);
        bearing[1] = 1.0;

        center[0] = vision_find_centroid(bin_img_ptr, 0);
    }

    // Clear variables to free memory.
    cvReleaseImage(&out_img_ptr);

    // Check to see how many pixels are detected in the image.
    num_pix = cvCountNonZero(bin_img_ptr);

    if (num_pix <= detect_thresh)
    {
        return -1;
    }

    // Check that the center point is not negative and bearing is valid.
    if (center[0].x < 0 || center[0].y < 0 || bearing[0] == -10000.0 || bearing[1] == -10000.0)
    {
        return 0;
    }

    return 1;
} // end vision_find_pipe()


/*------------------------------------------------------------------------------
 * int vision_boost_pipe()
 * Creates a binary image using the boosting predictor.
 * Finds the center and the bearing of the pipe(s) in the image.
 *----------------------------------------------------------------------------*/

int VisionPort::vision_boost_pipe(IplImage *src_img, IplImage *bin_img, CvPoint center[2], double bearing[4])
{
    // Declare variables.
    unsigned char *src_data = (unsigned char*)src_img->imageData;
    unsigned char *bin_data = (unsigned char*)bin_img->imageData;
    double dst[2] = {0, 0};
    double thresh = 1.0;
    int i = 0, j = 0, k = 0, res = 0;

    // Binary image variables.
    double h = 0.0, s = 0.0, v = 0.0;
    double* hsv[3] = {&h, &s, &v};
    IplConvKernel* B = NULL;
    static CvMemStorage* mem_storage = NULL;
    static CvSeq* contours = NULL;
    CvContourScanner scanner;
    CvSeq* c = NULL;
    CvSeq* c_new = NULL;

    // Center variable.s
    double c_area = 0.0, c_max = 0.0, c_max2 = 0.0;
    CvMoments moments;
    double M00 = 0.0, M01 = 0.0, M10 = 0.0;

    // Bearing variables.
    CvPoint *ppt = NULL;
    CvPoint *points = NULL;
    CvMat point_mat;
    CvMat *L_error = NULL;
    float fit_line[4];
    float *vlines = NULL;
    double pt_error = 0.0;
    double tot_error = 0.0;
    int *merge = NULL;
    int merge_count = 0;
    CvPoint *merge_points = NULL;
    CvMat merge_mat;
    int *parallel = NULL;
    int parallel_count = 0;

    // Create Morphological Kernel
    B = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT);

    // Loop through the first image to fill the left part of the new image.
    for (i = 0; i < src_img->height; i++)
    {
        for (j = 0; j < src_img->width; j++)
        {
            *hsv[0] = src_data[i * src_img->widthStep + j * src_img->nChannels + 0];
            *hsv[1] = src_data[i * src_img->widthStep + j * src_img->nChannels + 1];
            *hsv[2] = src_data[i * src_img->widthStep + j * src_img->nChannels + 2];

            // Predict on this point
            if (predict_pipe((void**)hsv, dst))
            {
                if (dst[1] > thresh)
                {
                    res = 0xff;
                }
                else
                {
                    res = 0;
                }
            }
            else
            {
                fprintf(stderr, "%s(): Prediction Fail.\n", __FUNCTION__);
            }

            bin_data[i * bin_img->widthStep + j * bin_img->nChannels + 0] = res;
            bin_data[i * bin_img->widthStep + j * bin_img->nChannels + 1] = res;
            bin_data[i * bin_img->widthStep + j * bin_img->nChannels + 2] = res;
        }
    }

    // Use Opening to remove noise
    cvMorphologyEx(bin_img, bin_img, NULL, B, CV_MOP_OPEN, 1);

    // Use Closing to fill in blobs
    cvMorphologyEx(bin_img, bin_img, NULL, B, CV_MOP_CLOSE, 2);

    // Use smooth to smooth the shapes
    cvSmooth(bin_img, bin_img, CV_MEDIAN, 7, 7, 0., 0.);

    // Find Countours around remaining regions
    if (mem_storage == NULL)
    {
        mem_storage = cvCreateMemStorage(0);
    }
    else
    {
        cvClearMemStorage(mem_storage);
    }

    scanner = cvStartFindContours(bin_img, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    while ((c = cvFindNextContour(scanner)) != NULL)
    {
        // Convex Hull Contour Approximation
        c_new = cvConvexHull2(c, mem_storage, CV_CLOCKWISE, 1);
        cvSubstituteContour(scanner, c_new);
    }
    contours = cvEndFindContours(&scanner);

    // Draw the resulting contours
    for (c = contours; c != NULL; c = c->h_next)
    {
        cvDrawContours(bin_img, c, CV_RGB(0xff, 0xff, 0xff), CV_RGB(0x00, 0x00, 0x00), -1, CV_FILLED, 8);
    }

    // Dilate because convex hull shrinks the contour
    cvDilate(bin_img, bin_img, B, 1);

    // Use smooth to smooth the shapes again
    cvSmooth(bin_img, bin_img, CV_MEDIAN, 7, 7, 0., 0.);

    // Find Countours around remaining regions
    if (mem_storage == NULL)
    {
        mem_storage = cvCreateMemStorage(0);
    }
    else
    {
        cvClearMemStorage(mem_storage);
    }

    scanner = cvStartFindContours(bin_img, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    while ((c = cvFindNextContour(scanner)) != NULL)
    {
        cvDrawContours(bin_img, c, CV_RGB(0xff, 0xff, 0xff), CV_RGB(0x00, 0x00, 0x00), -1, CV_FILLED, 8);

        // Get this contours area
        c_area = abs(cvContourArea(c, CV_WHOLE_SEQ));

        if (c_area > c_max)
        {
            // Set old max to be second oldest
            c_max2 = c_max;
            center[1].x = center[0].x;
            center[1].y = center[0].y;

            // Set max area
            c_max = c_area;

            // Get this contours center
            cvContourMoments(c, &moments);
            M00 = cvGetSpatialMoment(&moments, 0, 0);
            M10 = cvGetSpatialMoment(&moments, 1, 0);
            M01 = cvGetSpatialMoment(&moments, 0, 1);
            center[0].x = (int)(M10 / M00);
            center[0].y = (int)(M01 / M00);
        }
        else if (c_area > c_max2)
        {
            // Set second max area
            c_max2 = c_area;

            // Get this contours center
            cvContourMoments(c, &moments);
            M00 = cvGetSpatialMoment(&moments, 0, 0);
            M10 = cvGetSpatialMoment(&moments, 1, 0);
            M01 = cvGetSpatialMoment(&moments, 0, 1);
            center[1].x = (int)(M10 / M00);
            center[1].y = (int)(M01 / M00);
        }
    }
    contours = cvEndFindContours(&scanner);

    // Find the edges of the blobs
    cvCanny(bin_img, bin_img, 50, 150);

    // No need to look for bearing if no center.
    if (center[0].x < 0 && center[0].y < 0)
    {
        return 1;
    }

    // Process Hough Transform on Contours
    if (mem_storage == NULL)
    {
        mem_storage = cvCreateMemStorage(0);
    }
    else
    {
        cvClearMemStorage(mem_storage);
    }

    // Initialize hough variables.
    points = (CvPoint*)malloc(4 * sizeof(points[0]));
    point_mat = cvMat(1, 4, CV_32SC2, points);
    L_error = cvCreateMat(1, 4, CV_32SC1);

    // Probabilistic Hough Transform
    c = cvHoughLines2(bin_img, mem_storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 120, 20, 20, 20);

    // Create merge array
    merge = new int[c->total];
    for (i = 0; i < c->total; i++)
    {
        merge[i] = -1;
    }

    // Create parallel array
    parallel = new int[c->total];
    for (i = 0; i < c->total; i++)
    {
        parallel[i] = -1;
    }

    // Create vlines array
    vlines = new float[c->total*4];
    for (i = 0; i < c->total; i++)
    {
        vlines[i] = -1;
    }

    // Loop through combinations of lines to merge collinear
    for (i = 0; i < c->total; i++)
    {
        // Only consider if this line hasn't been merged
        if (merge[i] == -1)
        {
            // Get the points for this line
            ppt = (CvPoint*)cvGetSeqElem(c, i);
            points[0] = ppt[0];
            points[1] = ppt[1];

            // Init this line for merge
            merge[i] = i;
            merge_count = 1;

            for (j = i + 1; j < c->total; j++)
            {
                // Only compare if this line hasn't been merged
                if (merge[j] == -1)
                {
                    // Get the points for this line
                    ppt = (CvPoint*)cvGetSeqElem(c, j);
                    points[2] = ppt[0];
                    points[3] = ppt[1];

                    // Find the best fit line to the four points
                    cvFitLine(&point_mat, CV_DIST_L1, 1, 0.001, 0.001, fit_line);

                    // Calculate the error between the line and each point
                    tot_error = 0.0;
                    for (k = 0; k < 4; k++)
                    {
                        pt_error = abs(fit_line[1] * (fit_line[2] - points[k].x) + (-1 * fit_line[0] * (fit_line[3] - points[k].y)));
                        pt_error = pt_error / sqrt((fit_line[1] * fit_line[1]) + (fit_line[0] * fit_line[0]));
                        tot_error += pt_error;
                    }

                    // Check if error is less than threshold
                    if (tot_error / 4 < 3.0)
                    {
                        merge[j] = i;
                        merge_count++;
                    }
                }
            }

            // Merge the similar lines with this one
            if (merge_count > 0)
            {
                merge_points = (CvPoint*)malloc(merge_count * 2 * sizeof(merge_points[0]));
                merge_mat = cvMat(1, merge_count * 2, CV_32SC2, merge_points);

                // Get all the points of the lines to merge
                int mergeIx = 0;
                for (k = 0; k < c->total; k++)
                {
                    if (merge[k] == i)
                    {
                        ppt = (CvPoint*)cvGetSeqElem(c, k);
                        merge_points[mergeIx] = ppt[0];
                        merge_points[mergeIx+1] = ppt[1];
                        mergeIx += 2;
                    }
                }

                // Find the best fit line to the merge points
                cvFitLine(&merge_mat, CV_DIST_L1, 1, 0.001, 0.001, fit_line);

                vlines[4*i+0] = fit_line[0];
                vlines[4*i+1] = fit_line[1];
                vlines[4*i+2] = fit_line[2];
                vlines[4*i+3] = fit_line[3];

                free(merge_points);
            }
        }
    }

    // Remove lines that don't have a parallel
    float best_err_C1 = -1.0;
    int best_L1_C1 = -1;
    int best_L2_C1 = -1;
    float best_err_C2 = -1.0;
    int best_L1_C2 = -1;
    int best_L2_C2 = -1;
    parallel_count = 0;
    for (i = 0; i < c->total; i++)
    {
        if (merge[i] == i && parallel[i] == -1)
        {
            for (j = 0; j < c->total; j++)
            {
                if (j != i && merge[j] == j)
                {
                    // Calculate degrees of difference.
                    float degs1 = (atan2(vlines[4*i+1], vlines[4*i+0]) * 360) / (2 * M_PI);
                    float degs2 = (atan2(vlines[4*j+1], vlines[4*j+0]) * 360) / (2 * M_PI);

                    // Make sure if degs are 90 that the vector is rounded.
                    if (degs1 == 90.0)
                    {
                        vlines[4*i+0] = 0.0;
                        vlines[4*i+1] = 1.0;
                    }
                    if (degs2 == 90.0)
                    {
                        vlines[4*j+0] = 0.0;
                        vlines[4*j+1] = 1.0;
                    }

                    // Make sure if degs are 0 that the vector is rounded.
                    if (degs1 == 0.0)
                    {
                        vlines[4*i+0] = 1.0;
                        vlines[4*i+1] = 0.0;
                    }
                    if (degs2 == 0.0)
                    {
                        vlines[4*j+0] = 1.0;
                        vlines[4*j+1] = 0.0;
                    }

                    // Move the lines 180 so all are moving up.
                    if (degs1 < 0)
                    {
                        degs1 += 180;
                    }
                    if (degs2 < 0)
                    {
                        degs2 += 180;
                    }

                    float diff = fabs(degs1 - degs2);

                    // If diff is over 90 degs, then 180 it.
                    if (diff > 90.0)
                    {
                        diff = 180.0 - diff;
                    }

                    if (diff < 10.0)
                    {
                        float err_L1_C1 = vlines[4*i+1] * (vlines[4*i+2] - center[0].x) + (-1 * vlines[4*i+0] * (vlines[4*i+3] - center[0].y));
                        err_L1_C1 = err_L1_C1 / sqrt((vlines[4*i+1] * vlines[4*i+1]) + (vlines[4*i+0] * vlines[4*i+0]));
                        float err_L2_C1 = vlines[4*j+1] * (vlines[4*j+2] - center[0].x) + (-1 * vlines[4*j+0] * (vlines[4*j+3] - center[0].y));
                        err_L2_C1 = err_L2_C1 / sqrt((vlines[4*j+1] * vlines[4*j+1]) + (vlines[4*j+0] * vlines[4*j+0]));
                        float err_L1_C2 = vlines[4*i+1] * (vlines[4*i+2] - center[1].x) + (-1 * vlines[4*i+0] * (vlines[4*i+3] - center[1].y));
                        err_L1_C2 = err_L1_C2 / sqrt((vlines[4*i+1] * vlines[4*i+1]) + (vlines[4*i+0] * vlines[4*i+0]));
                        float err_L2_C2 = vlines[4*j+1] * (vlines[4*j+2] - center[1].x) + (-1 * vlines[4*j+0] * (vlines[4*j+3] - center[1].y));
                        err_L2_C2 = err_L2_C2 / sqrt((vlines[4*j+1] * vlines[4*j+1]) + (vlines[4*j+0] * vlines[4*j+0]));

                        // If the vectors are opposite, then we have to convert one err value.
                        if ((vlines[4*i+0] * vlines[4*i+1]) * (vlines[4*j+0] * vlines[4*j+1]) < 0)
                        {
                            err_L2_C1 *= -1.0;
                            err_L2_C2 *= -1.0;
                        }

                        // Proximity check if there is a second center.
                        // No second center or there is one and the lines are closer to the first one.
                        if ((center[1].x < 0 && center[1].y < 0) ||
                            (center[1].x >= 0 && center[1].y >= 0 &&
                            fabs(err_L1_C1) < fabs(err_L1_C2) && fabs(err_L2_C1) < fabs(err_L2_C2)))
                        {
                            // Surrounding the centroid check.
                            if ((err_L1_C1 * err_L2_C1) < 0)
                            {
                                // Check if this is better than a previous pair.
                                if (best_err_C1 == -1.0 ||
                                    ((fabs(err_L1_C1) + fabs(err_L2_C1)) / 2) < best_err_C1)
                                {
                                    // Save the pair of parallel lines.
                                    if (parallel[i] == -1)
                                    {
                                        parallel[i] = parallel_count;
                                    }
                                    if (parallel[j] == -1)
                                    {
                                        parallel[j] = parallel_count;
                                    }
                                    parallel_count++;

                                    // Save best proximity to center 1.
                                    best_err_C1 = (fabs(err_L1_C1) + fabs(err_L2_C1)) / 2;

                                    // Remove previous best lines for center 1.
                                    if (best_L1_C1 != -1)
                                    {
                                        parallel[best_L1_C1] = -2;
                                        merge[best_L1_C1] = -1;
                                    }
                                    if (best_L2_C1 != -1)
                                    {
                                        parallel[best_L2_C1] = -2;
                                        merge[best_L2_C1] = -1;
                                    }

                                    // Save the new best lines for center 1.
                                    best_L1_C1 = i;
                                    best_L2_C1 = j;

                                    break;
                                }
                            }
                        }
                        // There is a second center and the lines are closer to it.
                        else if (center[1].x >= 0 && center[1].y >= 0 &&
                                 fabs(err_L1_C1) >= fabs(err_L1_C2) && fabs(err_L2_C1) >= fabs(err_L2_C2))
                        {
                            // Surrounding the centroid check.
                            if ((err_L1_C2 * err_L2_C2) < 0)
                            {
                                // Check if this is better than a previous pair.
                                if (best_err_C2 == -1.0 ||
                                    ((fabs(err_L1_C2) + fabs(err_L2_C2)) / 2) < best_err_C2)
                                {
                                    // Save the pair of parallel lines.
                                    if (parallel[i] == -1)
                                    {
                                        parallel[i] = parallel_count;
                                    }
                                    if (parallel[j] == -1)
                                    {
                                        parallel[j] = parallel_count;
                                    }
                                    parallel_count++;

                                    // Save best proximity to center 2.
                                    best_err_C2 = (fabs(err_L1_C2) + fabs(err_L2_C2)) / 2;

                                    // Remove previous best lines for center 2.
                                    if (best_L1_C2 != -1)
                                    {
                                        parallel[best_L1_C2] = -2;
                                        merge[best_L1_C2] = -1;
                                    }
                                    if (best_L2_C2 != -1)
                                    {
                                        parallel[best_L2_C2] = -2;
                                        merge[best_L2_C2] = -1;
                                    }

                                    // Save the new best lines for center 2.
                                    best_L1_C2 = i;
                                    best_L2_C2 = j;

                                    break;
                                }
                            }
                        }
                    }
                }
            }

            // If no parallel found then remove this line
            if (j == c->total)
            {
                merge[i] = -1;
            }
        }
    }

    // Draw the remaining lines
    bearing[0] = 0.0;
    bearing[1] = 0.0;
    bearing[2] = 0.0;
    bearing[3] = 0.0;
    merge_count = 0;
    parallel_count = 0;
    for (i = 0; i < c->total; i++)
    {
        if (merge[i] == i)
        {
            float d = sqrt((double)vlines[4*i+0] * vlines[4*i+0] + (double)vlines[4*i+1] * vlines[4*i+1]);
            float t = (float)(bin_img->width + bin_img->height);
            ppt[0].x = cvRound(vlines[4*i+2] - (vlines[4*i+0] / d) * t);
            ppt[0].y = cvRound(vlines[4*i+3] - (vlines[4*i+1] / d) * t);
            ppt[1].x = cvRound(vlines[4*i+2] + (vlines[4*i+0] / d) * t);
            ppt[1].y = cvRound(vlines[4*i+3] + (vlines[4*i+1] / d) * t);

            cvLine(bin_img, ppt[0], ppt[1], CV_RGB(0xff, 0xff, 0xff), 3, 8);

            if (center[0].x > 0 && center[0].y > 0 && center[1].x > 0 && center[1].y > 0)
            {
                if (((fabs(vlines[4*i+2] - center[0].x) + fabs(vlines[4*i+3] - center[0].y)) / 2) <
                    ((fabs(vlines[4*i+2] - center[1].x) + fabs(vlines[4*i+3] - center[1].y)) / 2))
                {
                    //bearing[0] += vlines[4*i+0];
                    //bearing[1] += vlines[4*i+1];
                    bearing[0] += vlines[4*i+0] / vlines[4*i+1];
                    merge_count++;
                }
                else
                {
                    //bearing[2] += vlines[4*i+0];
                    //bearing[3] += vlines[4*i+1];
                    bearing[2] += vlines[4*i+0] / vlines[4*i+1];
                    parallel_count++;
                }
            }
            else if (center[0].x > 0 && center[0].y > 0)
            {
                //bearing[0] += vlines[4*i+0];
                //bearing[1] += vlines[4*i+1];
                bearing[0] += vlines[4*i+0] / vlines[4*i+1];
                merge_count++;
            }
        }
    }

    // Calculate Bearing
    if (merge_count > 0)
    {
        bearing[0] = bearing[0] / merge_count;
        bearing[1] = 1.0;
    }
    else
    {
        bearing[0] = -10000.0;
        bearing[1] = -10000.0;
    }
    if (parallel_count > 0)
    {
        // If second pipe is only pipe, make it the first.
        if (merge_count == 0)
        {
            bearing[0] = bearing[2] / parallel_count;
            bearing[1] = 1.0;
            bearing[2] = -10000.0;
            bearing[3] = -10000.0;
            center[0] = center[1];
            center[1].x = -10000.0;
            center[1].y = -10000.0;
        }
        else
        {
            bearing[2] = bearing[2] / parallel_count;
            bearing[3] = 1.0;
        }
    }
    else
    {
        bearing[2] = -10000.0;
        bearing[3] = -10000.0;
    }

    delete merge;
    delete vlines;
    delete parallel;
    free(points);
    cvReleaseStructuringElement(&B);

    return 1;
} // end vision_boost_pipe()



/*------------------------------------------------------------------------------
 * int vision_find_hedge()
 * Finds the entry gate object from a camera.
 *----------------------------------------------------------------------------*/

int VisionPort::vision_find_hedge(int *dotx, int *doty)
{
    CvPoint center;
    IplConvKernel *wS = cvCreateStructuringElementEx(3, 3,
                        (int)floor((3.0) / 2), (int)floor((3.0) / 2), CV_SHAPE_RECT);
    int num_pix = 0;
    int touch_thresh = 50000;
    int detect_thresh = 1500;
    
    // Initialize to impossible values.
    center.x = -10000;
    center.y = -10000;
	
	// Create intermediate images for scratch space.
    img_hsv_ptr = cvCreateImage(cvGetSize(img_in_ptr), IPL_DEPTH_8U, 3);
    bin_img_ptr = cvCreateImage(cvGetSize(img_in_ptr), IPL_DEPTH_8U, 1);
    IplImage *out_img_ptr = cvCreateImage(cvGetSize(img_in_ptr), IPL_DEPTH_8U, 1);
	  
    // Convert the image from RGB to HSV color space.
    cvCvtColor(img_in_ptr, img_hsv_ptr, CV_BGR2HSV);

    if (GATE_TECHNIQUE == boost_wanted)
    {
        // Setup thresholds
        touch_thresh = 150000;
        detect_thresh = 0;

        // Hedge Boost Technique
        vision_boost_hedge(img_hsv_ptr, bin_img_ptr, &center);
    }
    else
    {
        // Setup thresholds
        touch_thresh = 50000;
        detect_thresh = 1500;
               
        // Threshold all three channels using our own values.
        cvInRangeS(img_hsv_ptr, cvScalar(h_low, s_low, v_low),
                   cvScalar(h_high, s_high, v_high), bin_img_ptr);
           
        // Use a median filter image to remove outliers.
        cvSmooth(bin_img_ptr, out_img_ptr, CV_MEDIAN, 11, 11, 0. , 0.);
        cvMorphologyEx(bin_img_ptr, bin_img_ptr, wS, NULL, CV_MOP_CLOSE, 1);

        // Find the centroid.
        center = vision_find_centroid(bin_img_ptr, 5);
    }

    // Set the found center of the dot
    *dotx = center.x;
    *doty = center.y;
    // Clear variables to free memory.
    cvReleaseImage(&out_img_ptr);
    
    // Check to see how many pixels of are detected in the image.
    num_pix = cvCountNonZero(bin_img_ptr);

    if (num_pix > touch_thresh)
    {
        return 2;
    }
    if (num_pix <= detect_thresh)
    {
        return 0;
    }
    // Check that the values of dotx & doty are not negative.
    if (dotx < 0 || doty < 0)
    {
        return 0;
    }

    return 1; 
} // end vision_find_hedge()


/*------------------------------------------------------------------------------
 * int vision_boost_hedge()
 * Creates a binary image using the boosting predictor.
 * Finds the center of the hedge in the image.
 *----------------------------------------------------------------------------*/

int VisionPort::vision_boost_hedge(IplImage *src_img, IplImage *bin_img, CvPoint *center)
{
    // Declare variables.
    unsigned char *src_data = (unsigned char*)src_img->imageData;
    unsigned char *bin_data = (unsigned char*)bin_img->imageData;
    double dst[2] = {0, 0};
    double thresh = 1.0;
    int i = 0, j = 0, res = 0;
    double h = 0.0, s = 0.0, v = 0.0;
    double* hsv[3] = {&h, &s, &v};
    IplConvKernel* B = NULL;
    static CvMemStorage* mem_storage = NULL;
    static CvSeq* contours = NULL;
    CvContourScanner scanner;
    CvSeq* c = NULL;
    double c_area = 0.0, c_max = 0.0, c_max2 = 0.0;
    CvMoments moments;
    double M00 = 0.0, M01 = 0.0, M10 = 0.0;
    CvPoint c_center, c_center2;
    c_center.x = -1, c_center.y = -1;
    c_center2.x = -1, c_center2.y = -1;

    // Create Morphological Kernel
    B = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT);

    // Loop through the first image to fill the left part of the new image.
    for (i = 0; i < src_img->height; i++)
    {
        for (j = 0; j < src_img->width; j++)
        {
            *hsv[0] = src_data[i * src_img->widthStep + j * src_img->nChannels + 0];
            *hsv[1] = src_data[i * src_img->widthStep + j * src_img->nChannels + 1];
            *hsv[2] = src_data[i * src_img->widthStep + j * src_img->nChannels + 2];

            // Predict on this point
            if (predict_hedge((void**)hsv, dst))
            {
                if (dst[1] > thresh)
                {
                    res = 0xff;
                }
                else
                {
                    res = 0;
                }
            }
            else
            {
                fprintf(stderr, "%s(): Prediction Fail.\n", __FUNCTION__);
            }

            bin_data[i * bin_img->widthStep + j * bin_img->nChannels + 0] = res;
            bin_data[i * bin_img->widthStep + j * bin_img->nChannels + 1] = res;
            bin_data[i * bin_img->widthStep + j * bin_img->nChannels + 2] = res;
        }
    }

    // Use Opening to remove noise
    cvMorphologyEx(bin_img, bin_img, NULL, B, CV_MOP_OPEN, 1);

    // Use Closing to fill in blobs
    cvMorphologyEx(bin_img, bin_img, NULL, B, CV_MOP_CLOSE, 2);

    // Use smooth to smooth the shapes
    cvSmooth(bin_img, bin_img, CV_MEDIAN, 7, 7, 0., 0.);
	
    // Find Countours around remaining regions
    if (mem_storage == NULL)
    {
        mem_storage = cvCreateMemStorage(0);
    }
    else
    {
        cvClearMemStorage(mem_storage);
    }
    scanner = cvStartFindContours(bin_img, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    while ((c = cvFindNextContour(scanner)) != NULL)
    {
        cvDrawContours(bin_img, c, CV_RGB(0xff, 0xff, 0xff), CV_RGB(0x00, 0x00, 0x00), -1, CV_FILLED, 8);

        // Get this contours area
        c_area = abs(cvContourArea(c, CV_WHOLE_SEQ));

        if (c_area > c_max)
        {
            // Set old max to be second oldest
            c_max2 = c_max;
            c_center2.x = c_center.x;
            c_center2.y = c_center.y;

            // Set max area
            c_max = c_area;

            // Get this contours center
            cvContourMoments(c, &moments);
            M00 = cvGetSpatialMoment(&moments, 0, 0);
            M10 = cvGetSpatialMoment(&moments, 1, 0);
            M01 = cvGetSpatialMoment(&moments, 0, 1);
            c_center.x = (int)(M10 / M00);
            c_center.y = (int)(M01 / M00);
        }
        else if (c_area > c_max2)
        {
            // Set second max area
            c_max2 = c_area;

            // Get this contours center
            cvContourMoments(c, &moments);
            M00 = cvGetSpatialMoment(&moments, 0, 0);
            M10 = cvGetSpatialMoment(&moments, 1, 0);
            M01 = cvGetSpatialMoment(&moments, 0, 1);
            c_center2.x = (int)(M10 / M00);
            c_center2.y = (int)(M01 / M00);
        }
    }
    contours = cvEndFindContours(&scanner);
    
    // Use the largest area no matter what
    center->x = c_center.x;
    center->y = c_center.y;

    cvReleaseStructuringElement(&B);

    return 1;
} // end vision_boost_hedge()


/*------------------------------------------------------------------------------
 * CvPoint vision_find_centroid()
 * Finds the centroid of the pixels in the image.
 *----------------------------------------------------------------------------*/
CvPoint VisionPort::vision_find_centroid(IplImage *bin_image, int thresh)
{
    // Return value.
    CvPoint centroid;

    // Get image width and height.
    int width = bin_image->width;
    int height = bin_image->height;

    // Totals.
    unsigned int rowTotal = 0;
    unsigned int colTotal = 0;
    int count = 0;

    // Counters.
    unsigned int ii = 0;
    unsigned int jj = 0;
    bool detected = false;

    // Find centroid.
    for (ii = (bin_image->roi == NULL ? 0 : (unsigned int)bin_image->roi->xOffset);
         ii < (bin_image->roi == NULL ? (unsigned int)width : (unsigned int)(bin_image->roi->xOffset + bin_image->roi->width));
         ii++)
    {
        for (jj = (bin_image->roi == NULL ? 0 : (unsigned int)bin_image->roi->yOffset);
             jj < (bin_image->roi == NULL ? (unsigned int)height : (unsigned int)(bin_image->roi->yOffset + bin_image->roi->height));
             jj++)
        {
            if (bin_image->imageData[ii + jj * width] != 0)
            {
                if (bin_image->roi == NULL)
                {
                    colTotal += ii;
                    rowTotal += jj;
                }
                else
                {
                    colTotal += ii - (unsigned int)bin_image->roi->xOffset;
                    rowTotal += jj - (unsigned int)bin_image->roi->yOffset;
                }
                count++;
            }
        }
    }

    // Check if an object is detected.
    if (count > thresh)
        detected = true;

    // If the centroid was detected convert it.
    if (detected)
    {
        centroid.x = (int)colTotal / count;
        centroid.y = (int)rowTotal / count;
    }
    else   //* Send back negatives if we do not have a positive detection.
    {
        centroid.x = -1;
        centroid.y = -1;
    }

    return centroid;
} // end vision_find_centroid()



/*------------------------------------------------------------------------------
 * float vision_get_bearing()
 * Fits edges of pipe to a line and calculates its angle.
 *----------------------------------------------------------------------------*/

double VisionPort::vision_get_bearing(IplImage *input_bin_img)
{
    CvScalar Right_STD;
    CvScalar Left_STD;
    Right_STD.val[0] = HUGE_VAL;
    Left_STD.val[0] = HUGE_VAL;
    int left_edge_count = 0;
    int right_edge_count = 0;
    int im_height = input_bin_img->height;
    int im_width = input_bin_img->width;
    int edge_threshold = 3;
    double maxSTD = 5000;
    int k;
    CvSeq *point_seq;
    CvMemStorage *storage;
    CvMat *L_error;
    CvMat *R_error;
    CvPoint2D32f point;

    // Slope (m) and b for left, right and combined estimate.
    double mL = 0.0;
    double mR = 0.0;
    double m = 0.0;

    // Edge vectors, first element = x, second = y.
    int left_edge[im_height][2];
    int right_edge[im_height][2];
    int i = 0;
    int j = 0;

    // Initialize edge arrays, mset may be better.
    for (i = 0; i < im_height; i++)
    {   
        left_edge[i][0] = 0;
        left_edge[i][1] = 0;
        right_edge[i][0] = 0;
        right_edge[i][1] = 0;
    }
    for (i = 0; i < im_height - 10; i++)
    {
        // Scan through each line of image and look for first non zero pixel
        // then get the (i,j) pixel value.
        while ((cvGet2D(input_bin_img, i, j).val[0] < 1) && (j < im_width - 1))
        {
            j++;
        }
        // If we exit before getting to end of row, edge exists.
        if ((j < im_width - 1) && (j > 0))
        {
            left_edge[left_edge_count][0] = i; //FLIP i, j, here worksish
            left_edge[left_edge_count][1] = j;
            left_edge_count++;
            // Continue scanning to find right edge.
            while ((cvGet2D(input_bin_img, i, j).val[0] > 0) && (j < im_width - 1))
            {
                j++;
            }
            if (j < im_width - 2)  // Scan didn't get to end of image so right edge exists.
            {
                right_edge[right_edge_count][0] = i;
                right_edge[right_edge_count][1] = j;
                right_edge_count++;
            }
        }
        j = 0;
    }

    if ((left_edge_count < edge_threshold) && (right_edge_count < edge_threshold))
    {
        return 0.0;
    }

    // Begin fitline.
    float *left_line = new float[4];
    float *right_line = new float[4];

    storage = cvCreateMemStorage(0);
    point_seq = cvCreateSeq(CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage);

    if (left_edge_count > edge_threshold)
    {
        for (i = 0; i < left_edge_count; i++)
        {
            point = cvPoint2D32f(left_edge[i][0], left_edge[i][1]);
            cvSeqPush(point_seq, &point);
        }
        cvFitLine(point_seq, CV_DIST_L12, 0, 0.001, 0.001, left_line);
        mL = atan2(left_line[1], left_line[0]);
        L_error = cvCreateMat(1, left_edge_count - 1, CV_32SC1);
        for (k = 0; k < left_edge_count - 1; k++)
        {
            // Save errors in vector L_error.
            cvSetReal2D(L_error, 0, k, (double)(left_edge[k][2]) -
                        (double)((left_line[1] / left_line[0]) * (left_edge[k][0] - left_line[1]) + left_line[3]));
        }
        // Calculate standard deviation of error.
        cvAvgSdv(L_error, NULL, &Left_STD, NULL);
        cvClearSeq(point_seq);
    }

    if (right_edge_count > edge_threshold)
    {
        for (i = 0; i < right_edge_count; i++)
        {
            point = cvPoint2D32f(left_edge[i][0], left_edge[i][1]);
            cvSeqPush(point_seq, &point);
        }
        cvFitLine(point_seq, CV_DIST_L12, 0, 0.001, 0.001, right_line);
        mR = atan2(right_line[1], right_line[0]);
        R_error = cvCreateMat(1, right_edge_count - 1, CV_32SC1);
        for (k = 0; k < right_edge_count - 1; k++)
        {
            cvSetReal2D(R_error, 0, k, (double)(right_edge[k][2]) -
                        (double)((left_line[1] / left_line[0]) * (right_edge[k][0] - right_line[1]) + right_line[3]));
        }
        cvAvgSdv(R_error, NULL, &Right_STD, NULL);
        cvClearSeq(point_seq);
    }

    // If estimate is really poor, do not update bearing.
    if ((Right_STD.val[0] > maxSTD) && (Left_STD.val[0] > maxSTD))
    {
        delete left_line;
        delete right_line;
        cvReleaseMemStorage(&storage);
        return 0.0;
    }

    // Only a left edge, ignore right.
    if (right_edge_count <= edge_threshold)
    {
        m = mL;
    }
    // Only a right edge, ignore left.
    else if (left_edge_count <= edge_threshold)
    {
        m = mR;
    }
    // Both edges exist, scale each estimate by variances.
    else
    {
        m = (Right_STD.val[0] * mL + Left_STD.val[0] * mR) /
            (Right_STD.val[0] + Left_STD.val[0]);
    }

    delete left_line;
    delete right_line;
    cvReleaseMemStorage(&storage);

    return m;
} // end vision_get_bearing()
