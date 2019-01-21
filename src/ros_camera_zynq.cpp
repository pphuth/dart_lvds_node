/** ----------------------------------------------------------------------------
 *
 * Simple Basler Dart LVDS Frame Grabber ROS Node
 * http://www.greenteam-stuttgart.de/driverless
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (c) 2018, GreenTeam Driverless
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * -----------------------------------------------------------------------------
 *
 * @file    ros_camera_znq.cpp
 *
 * @brief   ROS Node for Grabbing Images From Basler Dart LVDS
 *
 * @author  Philipp Huth
 *
 * @date    10.02.2018
 *
 * @copyright (c) 2017, GreenTeam Driverless
 *
 * @license BSD 3-Clause License
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

// Include local header for instant camera hardware trigger configuration.
#include "HardwareTriggerConfiguration.h"

// Include local header for BCON hardware trigger configuration.
#include "BconTriggerGenerator.h"

// Include header to use the BCON control.
#include <basler/bconctl.h>

#include <unistd.h>    // for getopt()

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;


// Use the trigger generator in the FPGA to trigger the camera with 10 fps default.
BconTriggerGenerator trggen;
int period_ms;
int duration_ms;

bool waitForEnterOnExit;
int exitCode;

CInstantCamera camera;


/******************************************************************************/
// Set the 3 board user LEDs to the binary presentation of num
static void set_leds(int num)
{
    int result;
    result = num & (1 << 0) ? bconctl_board_led_on(bconctl_led_user0) : bconctl_board_led_off(bconctl_led_user0);
    result = num & (1 << 1) ? bconctl_board_led_on(bconctl_led_user1) : bconctl_board_led_off(bconctl_led_user1);
    result = num & (1 << 2) ? bconctl_board_led_on(bconctl_led_user2) : bconctl_board_led_off(bconctl_led_user2);
    PYLON_UNUSED(result);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_left");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);


    cv::Mat frame, cv_img_rgb;
    sensor_msgs::ImagePtr image_msg;


    // Testing
    cv::Mat img; // << image MUST be contained here
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time


    frame = cv::Mat(2592,1944, CV_8UC3, cv::Scalar(0,0,255));
    img_msg.height = 1944;
    img_msg.width = 2592;
    img_msg.step = 19;

    // For 14 fps Camera
    period_ms = 100;
    duration_ms = 50;

    ROS_INFO( "Grab images ");
    ROS_INFO( "using trigger generator: period= %d ms, duration= %d ms",period_ms ,duration_ms );


    if (!trggen.Start(period_ms, duration_ms))
        //return EXIT_FAILURE;
	ROS_INFO("Error");

    // Before using any pylon methods, the pylon runtime must be initialized.
    PylonInitialize();


    try
    {
        // Create an instant camera object with the camera device found first.
        CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());

        // Print the model name of the camera.
        ROS_INFO( "Using device %s", camera.GetDeviceInfo().GetModelName().c_str() );

        set_leds(0);

        // Register the standard configuration event handler for enabling hardware triggering.
        // The hardware trigger configuration handler replaces the default configuration
        // as all currently registered configuration handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
        camera.RegisterConfiguration( new CHardwareTriggerConfiguration("RisingEdge"), RegistrationMode_ReplaceAll, Cleanup_Delete);

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        camera.MaxNumBuffer = 5;

        // Start the grabbing of countOfImagesToGrab images.
        camera.StartGrabbing();

        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;

        // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
        // when countOfImagesToGrab images have been retrieved.

	    ros::Rate loop_rate(5);

	while ( ros::ok() && camera.IsGrabbing() ){
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                set_leds((uint8_t)ptrGrabResult->GetID());

                // Access the image data.
		        ROS_INFO("Frame ID: %d; Size X: %d Y: %d", (uint8_t)ptrGrabResult->GetID(), (uint16_t)ptrGrabResult->GetWidth(), (uint16_t)ptrGrabResult->GetHeight());
                const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
                if ((uint16_t)ptrGrabResult->GetWidth() > 0 && (uint16_t)ptrGrabResult->GetHeight() > 0)
                {
                    cv_img_rgb = cv::Mat((uint16_t)ptrGrabResult->GetHeight(), (uint16_t)ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) ptrGrabResult->GetBuffer());
                    image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_img_rgb ).toImageMsg();

                    // Publish the grabbed Image with ROS
                    ROS_INFO("Publish Image");
                    pub.publish(image_msg);
                }
            }
            else
            {
                cerr << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }

	    // 	ROS loop sleep
	    ros::spinOnce();
	    loop_rate.sleep();
        }
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
        exitCode = EXIT_FAILURE;
    }

return 0;

}
