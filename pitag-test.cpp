/*
 * Copyright (c) 2017 Matthew Petroff
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "FiducialDefines.h"
#include "FiducialModelPi.h"

using namespace ipa_Fiducials;

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        puts("Usage: pitag-test modelfile imagefile");
        return 1;
    }
    
    std::shared_ptr<AbstractFiducialModel> tag_detector;
    cv::Mat camera_matrix;
    std::string model_filename = argv[1];
    std::string image_filename = argv[2];
    std::vector<t_pose> tags_vec;
    
    tag_detector = std::shared_ptr<FiducialModelPi>(new FiducialModelPi());
    
    camera_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
    
    if (tag_detector->Init(camera_matrix, model_filename, false) & RET_FAILED)
    {
        puts("Initializing fiducial detector with camera matrix [FAILED]");
        return -1;
    }
    
    cv::Mat image;
    image = cv::imread(image_filename, CV_LOAD_IMAGE_COLOR);
    
    unsigned long ret_val = RET_OK;
    
    std::vector<t_points> vec_points;
    
    if (tag_detector->GetPoints(image, vec_points) & RET_FAILED)
    {
        puts("No tags detected");
        return -1;
    }

    for (unsigned int i=0; i<vec_points.size(); i++)
    {
	    std::cout << "Detected Tag " << vec_points[i].id << ": " << vec_points[i].image_points << std::endl;
    }
    
    return 0;
}
