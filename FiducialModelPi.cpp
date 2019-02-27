#include "FiducialModelPi.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ellipserefine.cpp"

using namespace ipa_Fiducials;


FiducialModelPi::FiducialModelPi()
{
                
}

FiducialModelPi::~FiducialModelPi()
{
        
}


unsigned long FiducialModelPi::GetPoints(cv::Mat& image, std::vector<t_points>& vec_points)
{
    m_image_size_factor = image.cols*1./640.0;  // express relative to a 640x480 pixels camera image

    cv::Mat src_mat_8U1;
    bool debug = false;
    if (debug)
        m_debug_img = image.clone();

// ------------ Convert image to gray scale if necessary -------------------
    if (image.channels() == 3)
    {
        src_mat_8U1.create(image.rows, image.cols, CV_8UC1);
        cv::cvtColor(image, src_mat_8U1, CV_RGB2GRAY );
    }
    else
    {
        src_mat_8U1 = image;
    }

    if (debug)
    {
        cv::imshow("00 Grayscale", src_mat_8U1);
        cv::waitKey(0);
    }
    
    cv::Mat grayscale_image = src_mat_8U1.clone();

// ------------ Filtering --------------------------------------------------
    if (false)
    {
        // Divide the image by its morphologically closed counterpart
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(19,19));
        cv::Mat closed;
        cv::morphologyEx(src_mat_8U1, closed, cv::MORPH_CLOSE, kernel);

        if (debug)
        {
            cv::imshow("10 filtering closed", closed);
            cv::waitKey(0);
        }

        src_mat_8U1.convertTo(src_mat_8U1, CV_32F); // divide requires floating-point
        cv::divide(src_mat_8U1, closed, src_mat_8U1, 1, CV_32F);
        cv::normalize(src_mat_8U1, src_mat_8U1, 0, 255, cv::NORM_MINMAX);
        src_mat_8U1.convertTo(src_mat_8U1, CV_8UC1); // convert back to unsigned int

        if (debug)
        {
            cv::imshow("11 filtering divide", src_mat_8U1);
            cv::waitKey(0);
        }
    }

// ------------ Adaptive thresholding --------------------------------------

    int minus_c = 21;
    int half_kernel_size = 20 * m_image_size_factor;    // express relative to a 640x480 pixels camera image

    if(m_use_fast_pi_tag)
    {
        minus_c = 11;
        half_kernel_size = 5;
    }        
    cv::adaptiveThreshold(src_mat_8U1, src_mat_8U1, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 2*half_kernel_size+1, minus_c);

    if (debug)
    {
        cv::imshow("20 Adaptive thresholding", src_mat_8U1);
        cv::waitKey(0);
    }

// ------------ Contour extraction --------------------------------------
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(src_mat_8U1, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    if (debug)
    {
        cv::Mat contour_image = m_debug_img;
 
        for(size_t i = 0; i < contours.size(); i++)
            cv::drawContours(contour_image, contours, (int)i, cv::Scalar(0, 0, 255), 1, 8);
        cv::imshow("30 Contours", contour_image);
        cv::waitKey(0);
    }

// ------------ Ellipse extraction --------------------------------------
    int min_ellipse_size = 3; // Min ellipse size at 70cm distance is 20x20 pixels
    //int min_contour_points = int(1.5 * min_ellipse_size);
    int max_ellipse_aspect_ratio = 7;
        
    if(m_use_fast_pi_tag)
    {
        min_ellipse_size = 5;
        max_ellipse_aspect_ratio = 15;
    }
        
    std::vector<cv::RotatedRect> ellipses;
    for(size_t i = 0; i < contours.size(); i++)
    {
        size_t count = contours[i].size();
        if( count < 6 )
            continue;

        cv::Mat pointsf;
        cv::Mat(contours[i]).convertTo(pointsf, CV_32F);
        cv::RotatedRect box = cv::fitEllipse(pointsf);

        // Plausibility checks
        double box_max = std::max(box.size.width, box.size.height);
        double box_min = std::min(box.size.width, box.size.height);
        if( box_max > box_min*max_ellipse_aspect_ratio )
            continue;
        if (box_max > std::min(src_mat_8U1.rows, src_mat_8U1.cols)*0.2)
            continue;
        if (box_min < 0.5*min_ellipse_size)
            continue;
        if (box_max < min_ellipse_size)
            continue;
        if(box.center.x < 0 || box.center.x >= src_mat_8U1.cols || box.center.y < 0 || box.center.y >= src_mat_8U1.rows)
            continue;

        bool add_ellipse = true;

        if(m_use_fast_pi_tag)
        {
            double ellipse_aspect_ratio = box.size.height/box.size.width;
            if(box.size.height > box.size.width) ellipse_aspect_ratio = 1/ellipse_aspect_ratio;
            if(box.size.area() > 200 && ellipse_aspect_ratio < 0.1)
                continue;

            //order ellipses in ascending order with respect to size
            for(size_t j = 0; j < ellipses.size() && add_ellipse ;j++)
            {
                if(box.size.area() > ellipses[j].size.area())
                {
                    ellipses.insert(ellipses.begin()+j,box);
                    add_ellipse = false;
                }
            }

        }
        else
        {
            // Check for double borders on circles and keep only larger ones
            for(unsigned int i = 0; i < ellipses.size(); i++)
            {
                double dist_thresh = box_min*0.1;
                double dist = std::abs(box.center.x - ellipses[i].center.x) + std::abs(box.center.y - ellipses[i].center.y);
                if (dist < dist_thresh)
                {
                    add_ellipse = false;
                    ellipses[i] = box;
                    break;
                }
            }
        }

        if (add_ellipse)
            ellipses.push_back(box);
    }

    //Fast Pi Tag
    std::vector<cv::Point2i> points;
    std::vector<cv::Rect> rois;
    cv::Mat ellipsevoting(src_mat_8U1.rows,src_mat_8U1.cols,CV_32FC1);
    cv::Mat ellipsedensity(src_mat_8U1.rows,src_mat_8U1.cols,CV_8UC1);

    if(m_use_fast_pi_tag)
    {
        std::vector<size_t> badellipses;

        //Fil cv::Mat with -1
        for(int i = 0; i < ellipsevoting.rows; i++)
        {
            for(int j = 0; j < ellipsevoting.cols; j++)
            {
                ellipsevoting.at<float> (i,j) = -1;
                ellipsedensity.at<unsigned char> (i,j) = 0;
            }
        }


        //ellipse density voting
        for(size_t i = 0; i < ellipses.size(); i++)
        {
            unsigned int votingsize = (int)std::min(src_mat_8U1.rows,src_mat_8U1.cols)*0.005;
            unsigned int vr_x = votingsize;
            unsigned int vr_y = votingsize;

            for(int k = -(int)vr_x/2; k < (int)vr_x/2; k++){
                for(int l = -(int)vr_y/2; l < (int)vr_y/2; l++){

                    int x = ellipses[i].center.x+l;
                    int y = ellipses[i].center.y+k;

                    //Border Overshoot
                    if(x >= src_mat_8U1.cols || x < 0)
                        continue;
                    if(y >= src_mat_8U1.rows || y < 0)
                        continue;

                    ellipsedensity.at<unsigned char> (y,x) = ellipsedensity.at<unsigned char> (y,x) + 1;
                }
            }
        }

        //check if a ellipse is already in the same place - > Take bigger one(ellipses are order in ascending order)
        //if(false) -> vote
        for(size_t i = 0; i < ellipses.size(); i++)
        {
            //unsigned int min = std::min(ellipses[i].size.height,ellipses[i].size.width);
            //voting area
            unsigned int votingsize = (int)std::min(src_mat_8U1.rows,src_mat_8U1.cols)*0.003;
            unsigned int vr_x = votingsize;
            unsigned int vr_y = votingsize;

            bool insert = true;

            for(int k = -(int)vr_x/2; k < (int)vr_x/2; k++){
                for(int l = -(int)vr_y/2; l < (int)vr_y/2; l++){

                    int x = ellipses[i].center.x+l;
                    int y = ellipses[i].center.y+k;

                    //Border Overshoot
                    if(x >= src_mat_8U1.cols || x < 0)
                        continue;
                    if(y >= src_mat_8U1.rows || y < 0)
                        continue;

                    if(ellipsevoting.at<float> (y,x) != -1){
                        insert = false;
                    }
                }
            }

            if(insert)
            {
                for(int k = -(int)vr_x/2; k < (int)vr_x/2; k++)
                {
                    for(int l = -(int)vr_y/2; l < (int)vr_y/2; l++)
                    {

                        int x = ellipses[i].center.x+l;
                        int y = ellipses[i].center.y+k;

                        //Border Overshoot
                        if(x >= src_mat_8U1.cols || x < 0)
                            continue;
                        if(y >= src_mat_8U1.rows || y < 0)
                            continue;

                        ellipsevoting.at<float> (y,x) = (float)i;
                    }
                }
            } else {
                badellipses.push_back(i);
            }
        }

        //store points with high ellipse density
        for(int c = 4; (points.empty() || c >= 3) && c >= 2;c--){
            for(int i = 0; i < ellipsedensity.rows; i++){
                for(int j = 0; j < ellipsedensity.cols; j++){
                    if(ellipsedensity.at<unsigned char> (i,j) >= c){
                        cv::Point2i point;
                        point.x = j;
                        point.y = i;
                        points.push_back(point);
                    }
                }
            }
        }

        //kick points which are too close together
        int max_distance = (int)std::max(src_mat_8U1.rows,src_mat_8U1.cols)*0.15;
        for(size_t i = 0; i < points.size(); i++){
            for(size_t j = i+1; j < points.size(); j++){

                int dist =  (int)cv::sqrt(double((points[i].x-points[j].x)*(points[i].x-points[j].x)+(points[i].y-points[j].y)*(points[i].y-points[j].y)));

                if(dist < max_distance){
                    points.erase(points.begin()+j);
                    j--;
                }
            }
        }

        //choose size of ROI
        for(size_t i = 0; i < points.size(); i++){

            //Find ellipse with smallest distance to point[i]
            unsigned int id = 0;
            double min_dist = cv::sqrt((points[i].x-ellipses[id].center.x)*(points[i].x-ellipses[id].center.x)+(points[i].y-ellipses[id].center.y)*(points[i].y-ellipses[id].center.y));
            for(size_t j = 1; j < ellipses.size(); j++){

                double dist = cv::sqrt((points[i].x-ellipses[j].center.x)*(points[i].x-ellipses[j].center.x)+(points[i].y-ellipses[j].center.y)*(points[i].y-ellipses[j].center.y));
                if(dist < min_dist){
                    min_dist = dist;
                    id = j;
                }
            }

            //compute roi and save in rois
            int side = (int)std::max(src_mat_8U1.cols,src_mat_8U1.rows)*0.2;

            cv::Point2i topleft;
            topleft.x = points[i].x - side;
            topleft.y = points[i].y - side;

            //keep image borders
            if(topleft.x + 2*side >= src_mat_8U1.cols) topleft.x = src_mat_8U1.cols - 2*side;
            if(topleft.x < 0) topleft.x = 0;
            if(topleft.y + 2*side >= src_mat_8U1.rows) topleft.y = src_mat_8U1.rows - 2*side;
            if(topleft.y < 0) topleft.y = 0;


            //Shrink rois to max number of  ellipses
            std::vector<cv::RotatedRect> ellipses_roi(ellipses);
            std::vector<cv::RotatedRect> ellipses_roi_tmp;

            //TODO This values are only  valid when used with the Kinect in FullHd mode
            //TODO Update is necessary to make it work with all cameras .. based on ellipse density in image
            //TODO See Diploma Thesis: Robust Object Detection Using Fiducial Markers from Matthias Nösner
            unsigned int max_ellipses_roi = 0.05*ellipses.size();
            if(max_ellipses_roi > 450) max_ellipses_roi = 450;
            if(max_ellipses_roi < 150) max_ellipses_roi = 150;
            //unsigned int max_ellipses_roi = 150;
            int stepsize = 2;

            while(ellipses_roi.size() > max_ellipses_roi){

                ellipses_roi_tmp.clear();
                for(size_t m = 0; m < ellipses_roi.size(); m++){
                    if(topleft.x < ellipses_roi[m].center.x &&
                            ellipses_roi[m].center.x < topleft.x + 2*side &&
                            topleft.y < ellipses_roi[m].center.y &&
                            ellipses_roi[m].center.y < topleft.y + 2*side){

                        ellipses_roi_tmp.push_back(ellipses_roi[m]);
                    }
                }

                ellipses_roi.clear();
                for(size_t a = 0; a < ellipses_roi_tmp.size(); a++)
                    ellipses_roi.push_back(ellipses_roi_tmp[a]);

                topleft.x += stepsize;
                topleft.y += stepsize;
                side -= stepsize;

            }

            //shrink as long a no ellipses are lost
            bool shrink = true;
            while(shrink){

                topleft.x += stepsize;
                topleft.y += stepsize;
                side -= stepsize;

                ellipses_roi_tmp.clear();

                for(size_t m = 0; m < ellipses_roi.size(); m++){
                    if(topleft.x < ellipses_roi[m].center.x &&
                            ellipses_roi[m].center.x < topleft.x + 2*side &&
                            topleft.y < ellipses_roi[m].center.y &&
                            ellipses_roi[m].center.y < topleft.y + 2*side){

                        ellipses_roi_tmp.push_back(ellipses_roi[m]);
                    }
                }

                //detect loosing ellipses
                if(ellipses_roi_tmp.size() < ellipses_roi.size()){
                    shrink = false;
                    topleft.x -= stepsize;
                    topleft.y -= stepsize;
                    side += stepsize;
                }

                ellipses_roi.clear();
                for(size_t a = 0; a < ellipses_roi_tmp.size(); a++)
                    ellipses_roi.push_back(ellipses_roi_tmp[a]);


            }

            //rois have the same id like the points in vector "points"
            rois.push_back(cv::Rect(topleft.x,topleft.y,(int)2*side,(int)2*side));
            if(debug) rectangle(m_debug_img, cv::Rect(topleft.x,topleft.y,(int)2*side,(int)2*side), cv::Scalar(255,255,255));
        }

        //Kick ellipses in inverted order
        while (!badellipses.empty())
        {
            size_t index = badellipses.back();
            badellipses.pop_back();
            ellipses.erase(ellipses.begin()+index);
        }
    }
    //Fast PiTag -END-

    if (debug)
    {
        cv::Mat ellipse_image = m_debug_img;
        for(unsigned int i = 0; i < ellipses.size(); i++)
        {
            cv::ellipse(ellipse_image, ellipses[i], cv::Scalar(0,255,0), 1, CV_AA);
            cv::ellipse(ellipse_image, ellipses[i].center, ellipses[i].size*0.5f, ellipses[i].angle, 0, 360, cv::Scalar(0,255,255), 1, CV_AA);
        }

        //Fast Pi Tag
        //Make them white for visualization
        if(m_use_fast_pi_tag)
        {
            cv::Mat ellipsedensity_img(src_mat_8U1.rows,src_mat_8U1.cols,CV_8UC1);
            for(int i = 0; i < ellipsedensity_img.rows; i++){
                for(int j = 0; j < ellipsedensity_img.cols; j++){
                    ellipsedensity_img.at<unsigned char> (i,j) = 0;
                }
            }

            //draw points
            for(size_t i = 0; i < points.size();i++)
            {
                ellipsedensity_img.at<unsigned char> (points[i].y,points[i].x) = 255;
            }

            //draw rois
            for(size_t i = 0; i < rois.size();i++){
                cv::imshow("Roi:", m_debug_img(rois[i]));
            }

            for(int i =0; i < ellipsevoting.rows;i++)
            {
                for(int j =0; j < ellipsevoting.cols;j++)
                {
                    if(ellipsevoting.at<float> (i,j) > -1)
                    {
                        ellipsevoting.at<float> (i,j) = 255;
                    }
                }
            }
            cv::imshow("80 Ellipsevoting", ellipsevoting);
            cv::imshow("90 Ellipsedensity", ellipsedensity_img);
        }
        //Fast Pi Tag -ENd-

        cv::imshow("40 Ellipses", ellipse_image);
        cv::waitKey(0);
    }
        
        
    //PITAG: For PITAG the loop is executed only once with all ellipses in the cloud
    //FASTPITAG: For FASTPITAG the number of loop excecutions is based on the number of rois
    std::vector<cv::RotatedRect> ellipses_copy(ellipses);
    bool once = true;
        
    for(size_t n = 0; n < points.size() || (!m_use_fast_pi_tag && once); n++){ //Each point is the center of a roi
        once = false;

        if(m_use_fast_pi_tag){
            ellipses.clear();
            //prepare ellipse cloud for marker detection
            for(size_t m = 0; m < ellipses_copy.size(); m++){
                if(rois[n].x < ellipses_copy[m].center.x &&
                        ellipses_copy[m].center.x < rois[n].x + rois[n].width &&
                        rois[n].y < ellipses_copy[m].center.y &&
                        ellipses_copy[m].center.y < rois[n].y + rois[n].height ){
                    //ellipses size is 10*smaller than roi
                    double factor = 0.05;
                    if((int)ellipses_copy[m].size.width*ellipses_copy[m].size.height < (int)rois[n].width*rois[n].height*factor){
                        ellipses.push_back(ellipses_copy[m]);
                    }
                }
            }
        }

        // ------------ Fiducial corner extraction --------------------------------------
        std::vector<std::vector<cv::RotatedRect> > marker_lines;
        double max_pixel_dist_to_line; // Will be set automatically
        double max_ellipse_difference; // Will be set automatically
        double deviation_of_aspectratio = 0.3; //m_use_fast_pi_tag
        // Compute area
        std::vector<double> ref_A;
        std::vector<double> ref_Ratio;
        for(unsigned int i = 0; i < ellipses.size(); i++)
            ref_A.push_back(std::max(ellipses[i].size.width, ellipses[i].size.height));
            
        //FPITAG
        if(m_use_fast_pi_tag)
        {
            for(unsigned int i = 0; i < ellipses.size(); i++)
            {
                double ellipse_aspect_ratio = ellipses[i].size.height/ellipses[i].size.width;
                if(ellipses[i].size.height > ellipses[i].size.width) ellipse_aspect_ratio = 1/ellipse_aspect_ratio;
                ref_Ratio.push_back(ellipse_aspect_ratio);
            }
        }
        //FPITAG

        for(unsigned int i = 0; i < ellipses.size(); i++)
        {
            for(unsigned int j = i+1; j < ellipses.size(); j++)
            {
                //Fast Pi Tag
                if(m_use_fast_pi_tag)
                {
                    if(std::abs(ref_Ratio[i]-ref_Ratio[j]) > deviation_of_aspectratio)
                        continue;
                }
                //Fast Pi Tag

                // Check area
                max_ellipse_difference = 0.5 * std::min(ref_A[i], ref_A[j]);
                if (std::abs(ref_A[i] - ref_A[j]) > max_ellipse_difference)
                    continue;

                // Compute line equation
                cv::Point2f vec_IJ = ellipses[j].center - ellipses[i].center;
                double dot_IJ_IJ = vec_IJ.ddot(vec_IJ);

                // Check all other ellipses if they fit to the line equation
                // Condition: Between two points are at most two other points
                // Not more and not less
                std::vector<cv::RotatedRect> line_candidate;
                int nLine_Candidates = 0;

                for(unsigned int k = 0; k < ellipses.size() && nLine_Candidates < 2; k++)
                {
                    //Fast Pi Tag
                    if(m_use_fast_pi_tag)
                    {
                        if(std::abs(ref_Ratio[j]-ref_Ratio[k]) > deviation_of_aspectratio)
                            continue;
                    }
                    //Fast Pi Tag

                    // Check area
                    max_ellipse_difference = 0.5 * std::min(ref_A[j], ref_A[k]);
                    if (std::abs(ref_A[j] - ref_A[k]) > max_ellipse_difference)
                        continue;

                    if (k == i || k == j)
                        continue;

                    // Check if k lies on the line between i and j
                    cv::Point2f vec_IK = ellipses[k].center - ellipses[i].center;
                    double t_k = vec_IK.ddot(vec_IJ) / dot_IJ_IJ;
                    if (t_k < 0 || t_k > 1)
                        continue;

                    // Check distance to line
                    cv::Point2f proj_k = ellipses[i].center + vec_IJ * t_k;
                    cv::Point2f vec_KprojK = proj_k - ellipses[k].center;
                    double d_k_sqr = (vec_KprojK.x*vec_KprojK.x) + (vec_KprojK.y*vec_KprojK.y);

                    max_pixel_dist_to_line = std::sqrt(std::min(ellipses[k].size.height, ellipses[k].size.width));
                    max_pixel_dist_to_line = std::max(2.0, max_pixel_dist_to_line);
                    if (d_k_sqr > max_pixel_dist_to_line*max_pixel_dist_to_line)
                        continue;

                    for(unsigned int l = k+1; l < ellipses.size() && nLine_Candidates < 2; l++)
                    {
                        //Fast Pi Tag
                        if(m_use_fast_pi_tag)
                        {
                            if(std::abs(ref_Ratio[k]-ref_Ratio[l]) > deviation_of_aspectratio)
                                continue;
                        }
                        //Fast Pi Tag
                            
                        // Check area
                        max_ellipse_difference = 0.5 * std::min(ref_A[k], ref_A[l]);
                        if (std::abs(ref_A[k] - ref_A[l]) > max_ellipse_difference)
                            continue;

                        if (l == i || l == j)
                            continue;

                        // Check if l lies on the line between i and j
                        cv::Point2f vec_IL = ellipses[l].center - ellipses[i].center;
                        double t_l = vec_IL.ddot(vec_IJ) / dot_IJ_IJ;
                        if (t_l < 0 || t_l > 1)
                            continue;

                        // Check distance to line
                        cv::Point2f proj_l = ellipses[i].center + vec_IJ * t_l;
                        cv::Point2f vec_LprojL = proj_l - ellipses[l].center;
                        double d_l_sqr = (vec_LprojL.x*vec_LprojL.x) + (vec_LprojL.y*vec_LprojL.y);

                        max_pixel_dist_to_line = std::sqrt(std::min(ellipses[l].size.height, ellipses[l].size.width));
                        max_pixel_dist_to_line = std::max(2.0, max_pixel_dist_to_line);
                        if (d_l_sqr > max_pixel_dist_to_line*max_pixel_dist_to_line)
                            continue;

                        // Yeah, we found 4 fitting points
                        line_candidate.push_back(ellipses[i]);
                        if (t_k < t_l)
                        {
                            line_candidate.push_back(ellipses[k]);
                            line_candidate.push_back(ellipses[l]);
                        }
                        else
                        {
                            line_candidate.push_back(ellipses[l]);
                            line_candidate.push_back(ellipses[k]);
                        }
                        line_candidate.push_back(ellipses[j]);
                        nLine_Candidates++;
                    }
                }

                // See condition above
                if(nLine_Candidates == 1)
                    marker_lines.push_back(line_candidate);
            }
        }

        if (debug)
        {
            //cv::Mat line_image = cv::Mat::zeros(src_mat_8U1.size(), CV_8UC3);
            cv::Mat line_image = m_debug_img.clone();
            for(unsigned int i = 0; i < marker_lines.size(); i++)
            {
                cv::line(line_image, marker_lines[i][0].center, marker_lines[i][3].center, cv::Scalar(0, 255, 255), 1, 8);
            }
            cv::imshow("50 Lines", line_image);
            cv::waitKey(0);
        }

        // ------------ Fiducial line association --------------------------------------
        double cross_ratio_max_dist = 0.03; //0.03
        std::vector<t_pi> final_tag_vec;

        for (unsigned int i = 0; i < m_ref_tag_vec.size(); i++)
        {
            m_ref_tag_vec[i].fitting_image_lines_0.clear();
            m_ref_tag_vec[i].fitting_image_lines_1.clear();
        }

        for(unsigned int i = 0; i < marker_lines.size(); i++)
        {
            // Cross ratio i
            cv::Point2f i_AB = marker_lines[i][1].center - marker_lines[i][0].center;
            cv::Point2f i_BD = marker_lines[i][3].center - marker_lines[i][1].center;
            cv::Point2f i_AC = marker_lines[i][2].center - marker_lines[i][0].center;
            cv::Point2f i_CD = marker_lines[i][3].center - marker_lines[i][2].center;
            double l_AB = std::sqrt(i_AB.x*i_AB.x + i_AB.y*i_AB.y);
            double l_BD = std::sqrt(i_BD.x*i_BD.x + i_BD.y*i_BD.y);
            double l_AC = std::sqrt(i_AC.x*i_AC.x + i_AC.y*i_AC.y);
            double l_CD = std::sqrt(i_CD.x*i_CD.x + i_CD.y*i_CD.y);
            double cross_ratio_i = (l_AB/l_BD)/(l_AC/l_CD);

            // Associate lines to markers based on their cross ratio
            for (unsigned int j = 0; j < m_ref_tag_vec.size(); j++)
            {
                if (std::abs(cross_ratio_i - m_ref_tag_vec[j].cross_ration_0) < cross_ratio_max_dist)
                    m_ref_tag_vec[j].fitting_image_lines_0.push_back(marker_lines[i]);
                else if (std::abs(cross_ratio_i - m_ref_tag_vec[j].cross_ration_1) < cross_ratio_max_dist)
                    m_ref_tag_vec[j].fitting_image_lines_1.push_back(marker_lines[i]);
            }
        }

        if (debug)
        {
            //cv::Mat line_image = cv::Mat::zeros(src_mat_8U1.size(), CV_8UC3);
            cv::Mat line_image = m_debug_img.clone();
            for (unsigned int j = 0; j < m_ref_tag_vec.size(); j++)
            {
                for(unsigned int i = 0; i < m_ref_tag_vec[j].fitting_image_lines_0.size(); i++)
                {
                    cv::line(line_image, m_ref_tag_vec[j].fitting_image_lines_0[i][0].center, m_ref_tag_vec[j].fitting_image_lines_0[i][3].center, cv::Scalar(255, 255, 0), 1, 8);
                }
                for(unsigned int i = 0; i < m_ref_tag_vec[j].fitting_image_lines_1.size(); i++)
                {
                    cv::line(line_image, m_ref_tag_vec[j].fitting_image_lines_1[i][0].center, m_ref_tag_vec[j].fitting_image_lines_1[i][3].center, cv::Scalar(255, 0, 255), 1, 8);
                }
            }
            cv::imshow("51 Valid Lines", line_image);
            cv::waitKey(0);
        }

        // Search for all tag types independently
        for(unsigned int i = 0; i < m_ref_tag_vec.size(); i++)
        {
            std::vector<t_pi> ul_tag_vec;
            std::vector<t_pi> lr_tag_vec;

            // Take into account that multi associations from one line to many others may occure
            std::vector<std::vector<int> >ul_idx_lines_0(
                    m_ref_tag_vec[i].fitting_image_lines_0.size(), std::vector<int>());
            std::vector<std::vector<int> >lr_idx_lines_1(
                    m_ref_tag_vec[i].fitting_image_lines_1.size(), std::vector<int>());

            // -----------------------UPPER LEFT ------------------------------------------------------
            // Check for a common upper left corner
            // cross_ratio = largest
            for(unsigned int j = 0; j < m_ref_tag_vec[i].fitting_image_lines_0.size(); j++)
            {
                for(unsigned int k = j+1; k < m_ref_tag_vec[i].fitting_image_lines_0.size(); k++)
                {
                    bool corners_are_matching = false;
                    bool reorder_j = false;
                    bool reorder_k = false;
                    if (m_ref_tag_vec[i].fitting_image_lines_0[j][0].center == m_ref_tag_vec[i].fitting_image_lines_0[k][0].center)
                    {
                        corners_are_matching = true;
                    }
                    else if (m_ref_tag_vec[i].fitting_image_lines_0[j][3].center == m_ref_tag_vec[i].fitting_image_lines_0[k][0].center)
                    {
                        corners_are_matching = true;
                        reorder_j = true;

                    }
                    else if (m_ref_tag_vec[i].fitting_image_lines_0[j][0].center == m_ref_tag_vec[i].fitting_image_lines_0[k][3].center)
                    {
                        corners_are_matching = true;
                        reorder_k = true;
                    }
                    else if (m_ref_tag_vec[i].fitting_image_lines_0[j][3].center == m_ref_tag_vec[i].fitting_image_lines_0[k][3].center)
                    {
                        corners_are_matching = true;
                        reorder_j = true;
                        reorder_k = true;
                    }

                    if (!corners_are_matching)
                        continue;

                    // Index 0 should corresponds to the common corner
                    if (reorder_j)
                    {
                        cv::RotatedRect tmp = m_ref_tag_vec[i].fitting_image_lines_0[j][3];
                        m_ref_tag_vec[i].fitting_image_lines_0[j][3] = m_ref_tag_vec[i].fitting_image_lines_0[j][0];
                        m_ref_tag_vec[i].fitting_image_lines_0[j][0] = tmp;
                        tmp = m_ref_tag_vec[i].fitting_image_lines_0[j][2];
                        m_ref_tag_vec[i].fitting_image_lines_0[j][2] = m_ref_tag_vec[i].fitting_image_lines_0[j][1];
                        m_ref_tag_vec[i].fitting_image_lines_0[j][1] = tmp;
                    }
                    if (reorder_k)
                    {
                        cv::RotatedRect tmp = m_ref_tag_vec[i].fitting_image_lines_0[k][3];
                        m_ref_tag_vec[i].fitting_image_lines_0[k][3] = m_ref_tag_vec[i].fitting_image_lines_0[k][0];
                        m_ref_tag_vec[i].fitting_image_lines_0[k][0] = tmp;
                        tmp = m_ref_tag_vec[i].fitting_image_lines_0[k][2];
                        m_ref_tag_vec[i].fitting_image_lines_0[k][2] = m_ref_tag_vec[i].fitting_image_lines_0[k][1];
                        m_ref_tag_vec[i].fitting_image_lines_0[k][1] = tmp;
                    }

                    // Compute angular ordering (clockwise)
                    cv::Point2f tag_corner0 = m_ref_tag_vec[i].fitting_image_lines_0[j][3].center;
                    cv::Point2f tag_corner1 = m_ref_tag_vec[i].fitting_image_lines_0[k][3].center;
                    cv::Point2f tag_cornerUL = m_ref_tag_vec[i].fitting_image_lines_0[j][0].center;
                    cv::Point2f tag_center = tag_corner1 + 0.5 * (tag_corner0 - tag_corner1);

                    cv::Point2f vec_center_cUL = tag_cornerUL - tag_center;
                    cv::Point2f vec_center_c0 = tag_corner0 - tag_center;
                    cv::Point2f vec_center_c1 = tag_corner1 - tag_center;

                    double sign_c0 = vec_center_cUL.x*vec_center_c0.y-vec_center_cUL.y*vec_center_c0.x;
                    double sign_c1 = vec_center_cUL.x*vec_center_c1.y-vec_center_cUL.y*vec_center_c1.x;
                    // One must be positive and the other negative
                    // Otherwise the two lines are collinear
                    if(sign_c0 * sign_c1 >= 0)
                        continue;

                    int idx0 = j;
                    int idx1 = k;
                    if (sign_c0 > 0)
                    {
                        idx0 = k;
                        idx1 = j;
                    }        

                    t_pi tag;
                    tag.image_points = std::vector<cv::RotatedRect>(12, cv::RotatedRect());
                    tag.image_points[0] = m_ref_tag_vec[i].fitting_image_lines_0[idx1][0];
                    tag.image_points[1] = m_ref_tag_vec[i].fitting_image_lines_0[idx1][1];
                    tag.image_points[2] = m_ref_tag_vec[i].fitting_image_lines_0[idx1][2];
                    tag.image_points[3] = m_ref_tag_vec[i].fitting_image_lines_0[idx1][3];

                    tag.image_points[9] = m_ref_tag_vec[i].fitting_image_lines_0[idx0][3];
                    tag.image_points[10] = m_ref_tag_vec[i].fitting_image_lines_0[idx0][2];
                    tag.image_points[11] = m_ref_tag_vec[i].fitting_image_lines_0[idx0][1];

                    ul_idx_lines_0[j].push_back(int(ul_tag_vec.size()));
                    ul_idx_lines_0[k].push_back(int(ul_tag_vec.size()));
                    ul_tag_vec.push_back(tag);
                }
            }
            // -----------------------LOWER RIGHT ------------------------------------------------------
            // Check for a common lower right corner
            // cross_ratio = lowest
            for(unsigned int j = 0; j < m_ref_tag_vec[i].fitting_image_lines_1.size(); j++)
            {
                for(unsigned int k = j+1; k < m_ref_tag_vec[i].fitting_image_lines_1.size(); k++)
                {
                    bool corners_are_matching = false;
                    bool reorder_j = false;
                    bool reorder_k = false;
                    if (m_ref_tag_vec[i].fitting_image_lines_1[j][0].center == m_ref_tag_vec[i].fitting_image_lines_1[k][0].center)
                    {
                        corners_are_matching = true;
                    }
                    else if (m_ref_tag_vec[i].fitting_image_lines_1[j][3].center == m_ref_tag_vec[i].fitting_image_lines_1[k][0].center)
                    {
                        corners_are_matching = true;
                        reorder_j = true;

                    }
                    else if (m_ref_tag_vec[i].fitting_image_lines_1[j][0].center == m_ref_tag_vec[i].fitting_image_lines_1[k][3].center)
                    {
                        corners_are_matching = true;
                        reorder_k = true;
                    }
                    else if (m_ref_tag_vec[i].fitting_image_lines_1[j][3].center == m_ref_tag_vec[i].fitting_image_lines_1[k][3].center)
                    {
                        corners_are_matching = true;
                        reorder_j = true;
                        reorder_k = true;
                    }

                    if (!corners_are_matching)
                    {
                        continue;
                    }

                    // Index 0 should corresponds to the common corner
                    if (reorder_j)
                    {
                        cv::RotatedRect tmp = m_ref_tag_vec[i].fitting_image_lines_1[j][3];
                        m_ref_tag_vec[i].fitting_image_lines_1[j][3] = m_ref_tag_vec[i].fitting_image_lines_1[j][0];
                        m_ref_tag_vec[i].fitting_image_lines_1[j][0] = tmp;
                        tmp = m_ref_tag_vec[i].fitting_image_lines_1[j][2];
                        m_ref_tag_vec[i].fitting_image_lines_1[j][2] = m_ref_tag_vec[i].fitting_image_lines_1[j][1];
                        m_ref_tag_vec[i].fitting_image_lines_1[j][1] = tmp;
                    }
                    if (reorder_k)
                    {
                        cv::RotatedRect tmp = m_ref_tag_vec[i].fitting_image_lines_1[k][3];
                        m_ref_tag_vec[i].fitting_image_lines_1[k][3] = m_ref_tag_vec[i].fitting_image_lines_1[k][0];
                        m_ref_tag_vec[i].fitting_image_lines_1[k][0] = tmp;
                        tmp = m_ref_tag_vec[i].fitting_image_lines_1[k][2];
                        m_ref_tag_vec[i].fitting_image_lines_1[k][2] = m_ref_tag_vec[i].fitting_image_lines_1[k][1];
                        m_ref_tag_vec[i].fitting_image_lines_1[k][1] = tmp;
                    }

                    // Compute angular ordering (clockwise)
                    cv::Point2f tag_corner0 = m_ref_tag_vec[i].fitting_image_lines_1[j][3].center;
                    cv::Point2f tag_corner1 = m_ref_tag_vec[i].fitting_image_lines_1[k][3].center;
                    cv::Point2f tag_cornerUL = m_ref_tag_vec[i].fitting_image_lines_1[j][0].center;
                    cv::Point2f tag_center = tag_corner1 + 0.5 * (tag_corner0 - tag_corner1);

                    cv::Point2f vec_center_cUL = tag_cornerUL - tag_center;
                    cv::Point2f vec_center_c0 = tag_corner0 - tag_center;
                    cv::Point2f vec_center_c1 = tag_corner1 - tag_center;

                    // Angle from cUL to c0 is negative if sign is positive and vice versa
                    double sign_c0 = vec_center_cUL.x*vec_center_c0.y-vec_center_cUL.y*vec_center_c0.x;
                    double sign_c1 = vec_center_cUL.x*vec_center_c1.y-vec_center_cUL.y*vec_center_c1.x;
                    // One must be positive and the other negative
                    // Otherwise the two lines are collinear
                    if(sign_c0 * sign_c1 >= 0)
                        continue;

                    int idx0 = j;
                    int idx1 = k;
                    if (sign_c0 > 0)
                    {
                        idx0 = k;
                        idx1 = j;
                    }

                    t_pi tag;
                    tag.image_points = std::vector<cv::RotatedRect>(12, cv::RotatedRect());
                    tag.image_points[6] = m_ref_tag_vec[i].fitting_image_lines_1[idx1][0];
                    tag.image_points[7] = m_ref_tag_vec[i].fitting_image_lines_1[idx1][1];
                    tag.image_points[8] = m_ref_tag_vec[i].fitting_image_lines_1[idx1][2];
                    tag.image_points[9] = m_ref_tag_vec[i].fitting_image_lines_1[idx1][3];

                    tag.image_points[3] = m_ref_tag_vec[i].fitting_image_lines_1[idx0][3];
                    tag.image_points[4] = m_ref_tag_vec[i].fitting_image_lines_1[idx0][2];
                    tag.image_points[5] = m_ref_tag_vec[i].fitting_image_lines_1[idx0][1];

                    lr_idx_lines_1[j].push_back(int(lr_tag_vec.size()));
                    lr_idx_lines_1[k].push_back(int(lr_tag_vec.size()));
                    lr_tag_vec.push_back(tag);
                }
            }
            // -----------------------LOWER LEFT or UPPER RIGHT ------------------------------------------------------
            // Check for a common lower left or upper right corner
            // Now, lines could already participate in matchings of ul and lr corners
            // cross_ratio = different
            for(unsigned int j = 0; j < m_ref_tag_vec[i].fitting_image_lines_0.size(); j++)
            {
                for(unsigned int k = 0; k < m_ref_tag_vec[i].fitting_image_lines_1.size(); k++)
                {
                    bool corners_are_matching = false;
                    bool reorder_j = false;
                    bool reorder_k = false;
                    if (m_ref_tag_vec[i].fitting_image_lines_0[j][0].center == m_ref_tag_vec[i].fitting_image_lines_1[k][0].center)
                    {
                        corners_are_matching = true;
                    }
                    else if (m_ref_tag_vec[i].fitting_image_lines_0[j][3].center == m_ref_tag_vec[i].fitting_image_lines_1[k][0].center)
                    {
                        corners_are_matching = true;
                        reorder_j = true;

                    }
                    else if (m_ref_tag_vec[i].fitting_image_lines_0[j][0].center == m_ref_tag_vec[i].fitting_image_lines_1[k][3].center)
                    {
                        corners_are_matching = true;
                        reorder_k = true;
                    }
                    else if (m_ref_tag_vec[i].fitting_image_lines_0[j][3].center == m_ref_tag_vec[i].fitting_image_lines_1[k][3].center)
                    {
                        corners_are_matching = true;
                        reorder_j = true;
                        reorder_k = true;
                    }

                    if (!corners_are_matching)
                    {
                        continue;
                    }

                    // Index 0 should corresponds to the common corner
                    if (reorder_j)
                    {
                        cv::RotatedRect tmp = m_ref_tag_vec[i].fitting_image_lines_0[j][3];
                        m_ref_tag_vec[i].fitting_image_lines_0[j][3] = m_ref_tag_vec[i].fitting_image_lines_0[j][0];
                        m_ref_tag_vec[i].fitting_image_lines_0[j][0] = tmp;
                        tmp = m_ref_tag_vec[i].fitting_image_lines_0[j][2];
                        m_ref_tag_vec[i].fitting_image_lines_0[j][2] = m_ref_tag_vec[i].fitting_image_lines_0[j][1];
                        m_ref_tag_vec[i].fitting_image_lines_0[j][1] = tmp;
                    }
                    if (reorder_k)
                    {
                        cv::RotatedRect tmp = m_ref_tag_vec[i].fitting_image_lines_1[k][3];
                        m_ref_tag_vec[i].fitting_image_lines_1[k][3] = m_ref_tag_vec[i].fitting_image_lines_1[k][0];
                        m_ref_tag_vec[i].fitting_image_lines_1[k][0] = tmp;
                        tmp = m_ref_tag_vec[i].fitting_image_lines_1[k][2];
                        m_ref_tag_vec[i].fitting_image_lines_1[k][2] = m_ref_tag_vec[i].fitting_image_lines_1[k][1];
                        m_ref_tag_vec[i].fitting_image_lines_1[k][1] = tmp;
                    }

                    // Compute angular ordering (clockwise)
                    cv::Point2f tag_corner0 = m_ref_tag_vec[i].fitting_image_lines_0[j][3].center;
                    cv::Point2f tag_corner1 = m_ref_tag_vec[i].fitting_image_lines_1[k][3].center;
                    cv::Point2f tag_cornerUL = m_ref_tag_vec[i].fitting_image_lines_0[j][0].center;
                    cv::Point2f tag_center = tag_corner1 + 0.5 * (tag_corner0 - tag_corner1);

                    cv::Point2f vec_center_cUL = tag_cornerUL - tag_center;
                    cv::Point2f vec_center_c0 = tag_corner0 - tag_center;
                    cv::Point2f vec_center_c1 = tag_corner1 - tag_center;

                    double sign_c0 = vec_center_cUL.x*vec_center_c0.y-vec_center_cUL.y*vec_center_c0.x;
                    double sign_c1 = vec_center_cUL.x*vec_center_c1.y-vec_center_cUL.y*vec_center_c1.x;
                    // One must be positive and the other negative
                    // Otherwise the two lines are collinear
                    if(sign_c0 * sign_c1 >= 0)
                        continue;

                    t_pi tag;
                    tag.image_points = std::vector<cv::RotatedRect>(12, cv::RotatedRect());

                    if (sign_c0 > 0)
                    {
                        // Lower left cornerfinal_tag_vec
                        tag.image_points = std::vector<cv::RotatedRect>(12, cv::RotatedRect());
                        tag.image_points[9] = m_ref_tag_vec[i].fitting_image_lines_0[j][0];
                        tag.image_points[10] = m_ref_tag_vec[i].fitting_image_lines_0[j][1];
                        tag.image_points[11] = m_ref_tag_vec[i].fitting_image_lines_0[j][2];
                        tag.image_points[0] = m_ref_tag_vec[i].fitting_image_lines_0[j][3];

                        tag.image_points[6] = m_ref_tag_vec[i].fitting_image_lines_1[k][3];
                        tag.image_points[7] = m_ref_tag_vec[i].fitting_image_lines_1[k][2];
                        tag.image_points[8] = m_ref_tag_vec[i].fitting_image_lines_1[k][1];


                        // Check if lines participated already in a matching
                        if (ul_idx_lines_0[j].empty() && lr_idx_lines_1[k].empty())
                        {
                            if (TagUnique(final_tag_vec, tag))
                            {
                                m_ref_tag_vec[i].sparse_copy_to(tag);
                                tag.no_matching_lines = 2;
                                final_tag_vec.push_back(tag);
                            }
                        }
                        else if (!ul_idx_lines_0[j].empty() && lr_idx_lines_1[k].empty())
                        {
                            for (unsigned int l=0; l<ul_idx_lines_0[j].size(); l++)
                            {
                                t_pi final_tag;
                                final_tag.image_points = tag.image_points;

                                // Add matching line segment to final tag
                                final_tag.image_points[1] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[1];
                                final_tag.image_points[2] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[2];
                                final_tag.image_points[3] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[3];

                                if (TagUnique(final_tag_vec, final_tag))
                                {
                                    m_ref_tag_vec[i].sparse_copy_to(final_tag);
                                    final_tag.no_matching_lines = 3;
                                    final_tag_vec.push_back(final_tag);
                                }
                            }
                        }
                        else if (ul_idx_lines_0[j].empty() && !lr_idx_lines_1[k].empty())
                        {
                            for (unsigned int l=0; l<lr_idx_lines_1[k].size(); l++)
                            {
                                t_pi final_tag;
                                final_tag.image_points = tag.image_points;

                                // Add matching line segment to final tag
                                final_tag.image_points[3] = lr_tag_vec[lr_idx_lines_1[k][l]].image_points[3];
                                final_tag.image_points[4] = lr_tag_vec[lr_idx_lines_1[k][l]].image_points[4];
                                final_tag.image_points[5] = lr_tag_vec[lr_idx_lines_1[k][l]].image_points[5];

                                if (TagUnique(final_tag_vec, final_tag))
                                {
                                    m_ref_tag_vec[i].sparse_copy_to(final_tag);
                                    final_tag.no_matching_lines = 3;
                                    final_tag_vec.push_back(final_tag);
                                }
                            }
                        }
                        else if (!ul_idx_lines_0[j].empty() && !lr_idx_lines_1[k].empty())
                        {
                            // YEAH buddy. You've got a complete matching
                            for (unsigned int l=0; l<ul_idx_lines_0[j].size(); l++)
                            {
                                for (unsigned int m=0; m<lr_idx_lines_1[k].size(); m++)
                                {
                                    t_pi final_tag;
                                    final_tag.image_points = tag.image_points;

                                    // Add matching line segment from ul to final tag
                                    final_tag.image_points[1] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[1];
                                    final_tag.image_points[2] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[2];
                                    final_tag.image_points[3] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[3];

                                    // Add matching line segment from lr to final tag
                                    final_tag.image_points[4] = lr_tag_vec[lr_idx_lines_1[k][m]].image_points[4];
                                    final_tag.image_points[5] = lr_tag_vec[lr_idx_lines_1[k][m]].image_points[5];

                                    // Check consistency
                                    if (ul_tag_vec[ul_idx_lines_0[j][l]].image_points[3].center != lr_tag_vec[lr_idx_lines_1[k][m]].image_points[3].center ||
                                            ul_tag_vec[ul_idx_lines_0[j][l]].image_points[9].center != lr_tag_vec[lr_idx_lines_1[k][m]].image_points[9].center)
                                        continue;

                                    if (!TagUnique(final_tag_vec, final_tag))
                                        continue;

                                    if (AnglesValid2D(final_tag.image_points))
                                    {
                                        m_ref_tag_vec[i].sparse_copy_to(final_tag);
                                        final_tag.no_matching_lines = 4;
                                        final_tag_vec.push_back(final_tag);
                                    }
                                }
                            }
                        } // End - else
                    } // END - Lower left corner
                    else
                    {
                        // Upper right corner
                        tag.image_points = std::vector<cv::RotatedRect>(12, cv::RotatedRect());
                        tag.image_points[0] = m_ref_tag_vec[i].fitting_image_lines_0[j][3];
                        tag.image_points[1] = m_ref_tag_vec[i].fitting_image_lines_0[j][2];
                        tag.image_points[2] = m_ref_tag_vec[i].fitting_image_lines_0[j][1];
                        tag.image_points[3] = m_ref_tag_vec[i].fitting_image_lines_0[j][0];

                        tag.image_points[4] = m_ref_tag_vec[i].fitting_image_lines_1[k][1];
                        tag.image_points[5] = m_ref_tag_vec[i].fitting_image_lines_1[k][2];
                        tag.image_points[6] = m_ref_tag_vec[i].fitting_image_lines_1[k][3];

                        // Check if lines participated already in a matching
                        if (ul_idx_lines_0[j].empty() && lr_idx_lines_1[k].empty())
                        {
                            if (TagUnique(final_tag_vec, tag))
                            {
                                m_ref_tag_vec[i].sparse_copy_to(tag);
                                tag.no_matching_lines = 2;
                                final_tag_vec.push_back(tag);
                            }
                        }
                        else if (!ul_idx_lines_0[j].empty() && lr_idx_lines_1[k].empty())
                        {
                            for (unsigned int l=0; l<ul_idx_lines_0[j].size(); l++)
                            {
                                t_pi final_tag;
                                final_tag.image_points = tag.image_points;

                                // Add matching line segment to final tag
                                final_tag.image_points[9] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[9];
                                final_tag.image_points[10] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[10];
                                final_tag.image_points[11] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[11];

                                if (TagUnique(final_tag_vec, final_tag))
                                {
                                    m_ref_tag_vec[i].sparse_copy_to(final_tag);
                                    final_tag.no_matching_lines = 3;
                                    final_tag_vec.push_back(final_tag);
                                }
                            }
                        }
                        else if (ul_idx_lines_0[j].empty() && !lr_idx_lines_1[k].empty())
                        {
                            for (unsigned int l=0; l<lr_idx_lines_1[k].size(); l++)
                            {
                                t_pi final_tag;
                                final_tag.image_points = tag.image_points;

                                // Add matching line segment to final tag
                                final_tag.image_points[7] = lr_tag_vec[lr_idx_lines_1[k][l]].image_points[7];
                                final_tag.image_points[8] = lr_tag_vec[lr_idx_lines_1[k][l]].image_points[8];
                                final_tag.image_points[9] = lr_tag_vec[lr_idx_lines_1[k][l]].image_points[9];

                                if (TagUnique(final_tag_vec, final_tag))
                                {
                                    m_ref_tag_vec[i].sparse_copy_to(final_tag);
                                    final_tag.no_matching_lines = 3;
                                    final_tag_vec.push_back(final_tag);
                                }
                            }
                        }
                        else if (!ul_idx_lines_0[j].empty() && !lr_idx_lines_1[k].empty())
                        {
                            // YEAH buddy. You've got a complete matching
                            for (unsigned int l=0; l<ul_idx_lines_0[j].size(); l++)
                            {
                                for (unsigned int m=0; m<lr_idx_lines_1[k].size(); m++)
                                {
                                    t_pi final_tag;
                                    final_tag.image_points = tag.image_points;

                                    // Add matching line segment from ul to final tag
                                    final_tag.image_points[9] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[9];
                                    final_tag.image_points[10] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[10];
                                    final_tag.image_points[11] = ul_tag_vec[ul_idx_lines_0[j][l]].image_points[11];

                                    // Add matching line segment from lr to final tag
                                    final_tag.image_points[7] = lr_tag_vec[lr_idx_lines_1[k][m]].image_points[7];
                                    final_tag.image_points[8] = lr_tag_vec[lr_idx_lines_1[k][m]].image_points[8];

                                    // Check consistency
                                    if (ul_tag_vec[ul_idx_lines_0[j][l]].image_points[3].center != lr_tag_vec[lr_idx_lines_1[k][m]].image_points[3].center ||
                                            ul_tag_vec[ul_idx_lines_0[j][l]].image_points[9].center != lr_tag_vec[lr_idx_lines_1[k][m]].image_points[9].center)
                                        continue;

                                    if (!TagUnique(final_tag_vec, final_tag))
                                        continue;                                                                

                                    if (AnglesValid2D(final_tag.image_points))
                                    {
                                        m_ref_tag_vec[i].sparse_copy_to(final_tag);
                                        final_tag.no_matching_lines = 4;
                                        final_tag_vec.push_back(final_tag);
                                    }
                                }
                            }
                        } // End - else
                    } // END - Upper right corner
                }
            } // End - Check for a common lower left or upper right corner

        } // End - Search for all tag types independently


        // ------------ Refine ellipses ------------------------------------
        int min_matching_lines = 4;
        
        int sobel_winsize = 3;
        float gauss_smooth_sigma = 3.0;
        
        if( sobel_winsize%2==0 )
        {
            sobel_winsize++;
            std::cerr << " Sobel winsize changed to " << sobel_winsize << " (must be odd)" << std::endl;
        }

        // Sobel
        cv::Mat input_image_smooth;
        if(gauss_smooth_sigma > 0.0f)
        {
            cv::GaussianBlur(grayscale_image, input_image_smooth, cv::Size(), gauss_smooth_sigma, gauss_smooth_sigma);
        }
        else
        {
            input_image_smooth=grayscale_image.clone();
        }
        cv::Mat gradient_x(input_image_smooth.rows, input_image_smooth.cols, CV_16S);
        cv::Mat gradient_y(input_image_smooth.rows, input_image_smooth.cols, CV_16S);
        cv::Sobel(input_image_smooth, gradient_x, CV_16S, 1, 0, sobel_winsize);
        cv::Sobel(input_image_smooth, gradient_y, CV_16S, 0, 1, sobel_winsize);
        
        cv::Mat refine_image = image.clone();
        for (unsigned int i=0; i<final_tag_vec.size(); i++)
        {
            if (final_tag_vec[i].no_matching_lines < min_matching_lines)
                continue;
            
            for (unsigned int j=0; j<final_tag_vec[i].image_points.size(); j++)
            {
                cv::Matx33d refined_ellipse;
                
                if (cv::runetag::ipa_Fiducials::ellipserefine(final_tag_vec[i].image_points[j], gradient_x, gradient_y, refined_ellipse))
                {                
                    cv::Point2d ellipse_center = cv::runetag::ipa_Fiducials::ellipseCenter(refined_ellipse);
                    
                    cv::circle( refine_image, final_tag_vec[i].image_points[j].center, 1, CV_RGB(0,255,0) );
                    cv::circle( refine_image, ellipse_center, 1, CV_RGB(0,0,255) );
                    
                    
                    final_tag_vec[i].image_points[j].center.x = ellipse_center.x;
                    final_tag_vec[i].image_points[j].center.y = ellipse_center.y;
                }
                else
                {
                    std::cerr << "Ellipse refine failed" << std::endl;
                }
            }
        }
        if (debug)
        {
            cv::imshow("52 refined ellipses", refine_image);
            cv::waitKey(0);
        }

        if (debug)
        {
            cv::Mat tag_image = image.clone();
            cv::Vec3b rgbValVec[] = {cv::Vec3b(0,0,0), cv::Vec3b(255,255,255),
                    cv::Vec3b(255,0,0), cv::Vec3b(0,255,255), cv::Vec3b(0,255,0)};
            for (unsigned int i=0; i<final_tag_vec.size(); i++)
            {
                if (final_tag_vec[i].no_matching_lines != 4)
                    continue;

                bool connect_points = false;
                for (unsigned int j=0; j<final_tag_vec[i].image_points.size(); j++)
                {
                    cv::Vec3b rgbVal = rgbValVec[final_tag_vec[i].no_matching_lines];
                    if (final_tag_vec[i].image_points[j].center.x != 0)
                    {
                        cv::circle(tag_image, final_tag_vec[i].image_points[j].center, 3, cv::Scalar(0,255,0), -1, CV_AA);
                        if (connect_points)
                        {
                            cv::line(tag_image, final_tag_vec[i].image_points[j-1].center, final_tag_vec[i].image_points[j].center, cv::Scalar(rgbVal[0], rgbVal[1], rgbVal[2]), 1, CV_AA);
                        }
                        connect_points = true;
                    }
                    else
                        connect_points = false;
                }
            }
            cv::imshow("60 Tags", tag_image);
        }

        for (unsigned int i=0; i<final_tag_vec.size(); i++)
        {
            if (final_tag_vec[i].no_matching_lines < min_matching_lines)
                continue;
            
            t_points tag_points;
            tag_points.id = final_tag_vec[i].parameters.m_id;
            tag_points.marker_points = std::vector<cv::Point2f>();
            tag_points.image_points = std::vector<cv::Point2f>();
            
            for (unsigned int j=0; j<final_tag_vec[i].image_points.size(); j++)
            {
                
                tag_points.marker_points.push_back(final_tag_vec[i].marker_points[j]);
                tag_points.image_points.push_back(final_tag_vec[i].image_points[j].center);
            }

            vec_points.push_back(tag_points);
        }
    }
// ------------ END --------------------------------------
        
    if (debug)
    {
        cv::waitKey();
    }

    if (vec_points.empty())
            return RET_FAILED;

    return RET_OK;
}

unsigned long FiducialModelPi::GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose)
{
    std::vector<t_points> vec_points;
    if (GetPoints(image, vec_points) != RET_OK)
        return RET_FAILED;
    
    // ------------ Compute pose --------------------------------------
    
    for (unsigned int i=0; i<vec_points.size(); i++)
    {
        int nPoints = 0;
        for (unsigned int j=0; j<vec_points[i].image_points.size(); j++)
            if (vec_points[i].image_points[j].x != 0)
                nPoints++;

        cv::Mat pattern_coords(nPoints, 3, CV_32F);
        cv::Mat image_coords(nPoints, 2, CV_32F);

        float* p_pattern_coords = 0;
        float* p_image_coords = 0;
        int idx = 0;
        for (unsigned int j=0; j<vec_points[i].image_points.size(); j++)
        {
            if (vec_points[i].image_points[j].x != 0)
            {
                p_pattern_coords = pattern_coords.ptr<float>(idx);
                p_pattern_coords[0] = vec_points[i].marker_points[j].x;
                p_pattern_coords[1] = vec_points[i].marker_points[j].y;
                p_pattern_coords[2] = 0;

                p_image_coords = image_coords.ptr<float>(idx);
                p_image_coords[0] = vec_points[i].image_points[j].x;
                p_image_coords[1] = vec_points[i].image_points[j].y;

                idx++;
            }
        }

        t_pose tag_pose;
        tag_pose.id = vec_points[i].id;
        cv::solvePnP(pattern_coords, image_coords, GetCameraMatrix(), GetDistortionCoeffs(), tag_pose.rot, tag_pose.trans);

        // Apply transformation
        cv::Mat rot_3x3_CfromO;
        cv::Rodrigues(tag_pose.rot, rot_3x3_CfromO);

        cv::Mat reprojection_matrix = GetCameraMatrix();
        if (!ProjectionValid(rot_3x3_CfromO, tag_pose.trans, reprojection_matrix, pattern_coords, image_coords))
            continue;

        ApplyExtrinsics(rot_3x3_CfromO, tag_pose.trans);
        rot_3x3_CfromO.copyTo(tag_pose.rot);
        vec_pose.push_back(tag_pose);
    }
    
    //Fast Pi Tag
    if(m_use_fast_pi_tag)
    {
        //Maximum detection distance
        double max_detection_distance = 5.1;
        for(size_t h = 0; h < vec_pose.size();h++){
            if(vec_pose[h].trans.at<double>(2) > max_detection_distance){
                vec_pose.erase(vec_pose.begin()+h);
                h--;
            }
        }
        //Kick double detected Markers
        double min_marker_distance = 0.01; //minimum distance between two markers
        for(size_t h = 0; h < vec_pose.size();h++){
            for(size_t b = h+1; b < vec_pose.size();b++){
                if(vec_pose[h].id == vec_pose[b].id){
                    double distance_between_markers = cv::sqrt(vec_pose[h].trans.at<double>(0)*vec_pose[b].trans.at<double>(0)+
                            vec_pose[h].trans.at<double>(1)*vec_pose[b].trans.at<double>(1)+
                            vec_pose[h].trans.at<double>(2)*vec_pose[b].trans.at<double>(2) );
                    if( distance_between_markers > min_marker_distance){
                        vec_pose.erase(vec_pose.begin()+b);
                        b--;
                    }
                }
            }
        }
    }
    //Fast Pi Tag
    
    if (vec_points.empty())
        return RET_FAILED;

    return RET_OK;
}

bool FiducialModelPi::AnglesValid2D(std::vector<cv::RotatedRect>& image_points)
{
    // Check angles
    //float max_symtry_deg_diff = 40;
    float min_deg_angle = 20;
    cv::Point2f vec_03 = image_points[3].center - image_points[0].center;
    cv::Point2f vec_36 = image_points[6].center - image_points[3].center;
    cv::Point2f vec_69 = image_points[9].center - image_points[6].center;
    cv::Point2f vec_90 = image_points[0].center - image_points[9].center;

    float size_vec_03 = std::sqrt(vec_03.x*vec_03.x + vec_03.y*vec_03.y);
    float size_vec_36 = std::sqrt(vec_36.x*vec_36.x + vec_36.y*vec_36.y);
    float size_vec_69 = std::sqrt(vec_69.x*vec_69.x + vec_69.y*vec_69.y);
    float size_vec_90 = std::sqrt(vec_90.x*vec_90.x + vec_90.y*vec_90.y);

    vec_03.x /= size_vec_03;
    vec_03.y /= size_vec_03;
    vec_36.x /= size_vec_36;
    vec_36.y /= size_vec_36;
    vec_69.x /= size_vec_69;
    vec_69.y /= size_vec_69;
    vec_90.x /= size_vec_90;
    vec_90.y /= size_vec_90;

    float angle_ur = std::acos((-vec_03.x)*vec_36.x+(-vec_03.y)*vec_36.y)*180.0/CV_PI;
    float angle_lr = std::acos((-vec_36.x)*vec_69.x+(-vec_36.y)*vec_69.y)*180.0/CV_PI;
    float angle_ll = std::acos((-vec_69.x)*vec_90.x+(-vec_69.y)*vec_90.y)*180.0/CV_PI;
    float angle_ul = std::acos((-vec_90.x)*vec_03.x+(-vec_90.y)*vec_03.y)*180.0/CV_PI;

    //if (std::abs(angle_ur-angle_ll) > max_symtry_deg_diff ||
    //        std::abs(angle_ul-angle_lr) > max_symtry_deg_diff)
    //        return false;

    if (std::abs(angle_ur) < min_deg_angle ||
        std::abs(angle_lr) < min_deg_angle ||
        std::abs(angle_ll) < min_deg_angle ||
        std::abs(angle_ul) < min_deg_angle)
        return false;

    return true;
}

bool FiducialModelPi::ProjectionValid(cv::Mat& rot_CfromO, cv::Mat& trans_CfromO,
        cv::Mat& camera_matrix, cv::Mat& pts_in_O, cv::Mat& image_coords)
{
    double max_avg_pixel_error = 5 * m_image_size_factor;   // express relative to a 640x480 pixels camera image;

    // Check angles
    float* p_pts_in_O = 0;
    double* p_pt_in_O = 0;
    double* p_pt_4x1_in_C = 0;
    double* p_pt_3x1_in_C = 0;
    double* p_pt_3x1_2D = 0;
    float* p_image_coords;

    cv::Mat pt_in_O(4, 1, CV_64FC1);
    p_pt_in_O = pt_in_O.ptr<double>(0);
    cv::Mat pt_4x1_in_C;
    cv::Mat pt_3x1_in_C(3, 1, CV_64FC1);
    p_pt_3x1_in_C = pt_3x1_in_C.ptr<double>(0);
    cv::Mat pt_3x1_2D(3, 1, CV_64FC1);
    p_pt_3x1_2D = pt_3x1_2D.ptr<double>(0);

    // Create 4x4 frame CfromO
    cv::Mat frame_CfromO = cv::Mat::zeros(4, 4, CV_64FC1);
    for (int i=0; i<3; i++)
    {
        frame_CfromO.at<double>(i, 3) = trans_CfromO.at<double>(i,0);
        for (int j=0; j<3; j++)
        {
            frame_CfromO.at<double>(i,j) = rot_CfromO.at<double>(i,j);
        }
    }
    frame_CfromO.at<double>(3,3) = 1.0;

    // Check reprojection error
    double dist = 0;
    for (int i=0; i<pts_in_O.rows; i++)
    {
        p_image_coords = image_coords.ptr<float>(i);
        p_pts_in_O = pts_in_O.ptr<float>(i);

        p_pt_in_O[0] = p_pts_in_O[0];
        p_pt_in_O[1] = p_pts_in_O[1];
        p_pt_in_O[2] = p_pts_in_O[2];
        p_pt_in_O[3] = 1;

        cv::Mat pt_4x1_in_C = frame_CfromO * pt_in_O;
        p_pt_4x1_in_C = pt_4x1_in_C.ptr<double>(0);
        p_pt_3x1_in_C[0] = p_pt_4x1_in_C[0]/p_pt_4x1_in_C[3];
        p_pt_3x1_in_C[1] = p_pt_4x1_in_C[1]/p_pt_4x1_in_C[3];
        p_pt_3x1_in_C[2] = p_pt_4x1_in_C[2]/p_pt_4x1_in_C[3];

        pt_3x1_2D = camera_matrix * pt_3x1_in_C;
        pt_3x1_2D /= p_pt_3x1_2D[2];

        dist = std::sqrt((p_pt_3x1_2D[0] - p_image_coords[0])*(p_pt_3x1_2D[0] - p_image_coords[0])
        + (p_pt_3x1_2D[1] - p_image_coords[1])*(p_pt_3x1_2D[1] - p_image_coords[1]));

        if (dist > max_avg_pixel_error)
            return false;
    }

    return true;
}

bool FiducialModelPi::TagUnique(std::vector<t_pi>& tag_vec, t_pi& newTag)
{
    // Insert if not already existing
    bool duplicate = true;        
    for (unsigned int i=0; i<tag_vec.size(); i++)
    {
        duplicate = true;
        for (int j=0; j<12; j++)
        {
            if (tag_vec[i].image_points[j].center !=
                newTag.image_points[j].center)
            {
                duplicate = false;
                break;
            }
        }
        if (duplicate)
            return false;
    }
    return true;
}

unsigned long FiducialModelPi::LoadParameters(std::vector<FiducialPiParameters> pi_tags)
{
        m_ref_tag_vec.clear();
        for(unsigned int i=0; i<pi_tags.size(); i++)
        {
                t_pi ref_tag;
                double tag_size = pi_tags[i].line_width_height;

                ref_tag.parameters = pi_tags[i];

                double d_line0_AB = tag_size * pi_tags[i].d_line0_AB; //AB
                double d_line0_BD = tag_size - tag_size * pi_tags[i].d_line0_AB; //BD
                double d_line0_AC = tag_size * pi_tags[i].d_line0_AC;//AC
                double d_line0_CD = tag_size - tag_size * pi_tags[i].d_line0_AC;//CD
                ref_tag.cross_ration_0 = (d_line0_AB/d_line0_BD)/(d_line0_AC/d_line0_CD);

                double d_line1_AB = tag_size * pi_tags[i].d_line1_AB;
                double d_line1_BD = tag_size - tag_size * pi_tags[i].d_line1_AB;
                double d_line1_AC = tag_size * pi_tags[i].d_line1_AC;
                double d_line1_CD = tag_size - tag_size * pi_tags[i].d_line1_AC;
                ref_tag.cross_ration_1 = (d_line1_AB/d_line1_BD)/(d_line1_AC/d_line1_CD);
        
                // Marker coordinates
                ref_tag.marker_points.push_back(cv::Point2f(0, -0));
                ref_tag.marker_points.push_back(cv::Point2f(float(d_line0_AB), -0));
                ref_tag.marker_points.push_back(cv::Point2f(float(d_line0_AC), -0));
                ref_tag.marker_points.push_back(cv::Point2f(float(tag_size), -0));
                ref_tag.marker_points.push_back(cv::Point2f(float(tag_size), -float(d_line1_AB)));
                ref_tag.marker_points.push_back(cv::Point2f(float(tag_size), -float(d_line1_AC)));
                ref_tag.marker_points.push_back(cv::Point2f(float(tag_size), -float(tag_size)));
                ref_tag.marker_points.push_back(cv::Point2f(float(d_line1_AC), -float(tag_size)));
                ref_tag.marker_points.push_back(cv::Point2f(float(d_line1_AB), -float(tag_size)));
                ref_tag.marker_points.push_back(cv::Point2f(0, -float(tag_size)));
                ref_tag.marker_points.push_back(cv::Point2f(0, -float(d_line0_AC)));
                ref_tag.marker_points.push_back(cv::Point2f(0, -float(d_line0_AB)));

                // Offset
                for(unsigned int j=0; j<ref_tag.marker_points.size(); j++)
                {
                        ref_tag.marker_points[j].x += pi_tags[i].m_offset.x;
                        ref_tag.marker_points[j].y += pi_tags[i].m_offset.y;
                }

                double delta = ref_tag.cross_ration_0/ref_tag.cross_ration_1;
                if(std::abs(delta - 1) < 0.05)
                {
                        std::cerr << "[WARNING] FiducialModelPi::LoadCoordinates" << std::endl;
                        std::cerr << "\t ... Skipping fiducial due to equal cross ratios" << std::endl;
                }
                else if (delta < 1)
                {
                        std::cerr << "[WARNING] FiducialModelPi::LoadCoordinates" << std::endl;
                        std::cerr << "\t ... Skipping fiducial "<< ref_tag.parameters.m_id <<" due to cross ratios" << std::endl;
                        std::cerr << "\t ... Cross ratio 0 must be larger than cross ratio 1" << std::endl;
                }
                else
                {
                        m_ref_tag_vec.push_back(ref_tag);
                }

                if (m_ref_tag_vec.empty())
                {
                        std::cerr << "[ERROR] FiducialModelPi::LoadCoordinates" << std::endl;
                        std::cerr << "\t ... No valid fiducials loaded" << std::endl;
                        return RET_FAILED;
                }
        }
        return RET_OK;
}

unsigned long FiducialModelPi::LoadParameters(std::string directory_and_filename)
{
        std::vector<FiducialPiParameters> vec_pi_parameters;
        std::string tempString = "";
        // Load parameters from file
        std::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( directory_and_filename ));

        if (!p_configXmlDocument->LoadFile())
        {
                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):" << std::endl;
                std::cerr << "\t ... '" << directory_and_filename << std::endl;
                RET_FAILED;
        }
        std::cerr << "INFO - FiducialModelPi::LoadParameters:" << std::endl;
        std::cerr << "\t ... Parsing xml configuration file:" << std::endl;
        std::cerr << "\t ... " << directory_and_filename << std::endl;

        if ( p_configXmlDocument )
        {

//************************************************************************************
//        BEGIN FiducialDetector
//************************************************************************************
                // Tag element "ObjectDetector" of Xml Inifile
                TiXmlElement *p_xmlElement_Root = NULL;
                p_xmlElement_Root = p_configXmlDocument->FirstChildElement( "FiducialDetector" );

                if ( p_xmlElement_Root )
                {
                    
//************************************************************************************
//  BEGIN FiducialDetector->Fast PiTag flag
//************************************************************************************

                    TiXmlElement *p_xmlElement_fpitag = NULL;
                    p_xmlElement_fpitag = p_xmlElement_Root->FirstChildElement("FPITAG");
                    bool fpitag_default = 0;
                    if( p_xmlElement_fpitag ) {

                        // read and save value of attribute
                        if ( p_xmlElement_fpitag->QueryValueAttribute( "value", &m_use_fast_pi_tag) != TIXML_SUCCESS)
                        {
                            std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                            std::cerr << "\t ... Can't find attribute 'value' of tag 'FPITAG'" << std::endl;
                            std::cerr << "\t ... disabling Fast PiTag " << std::endl;
                            m_use_fast_pi_tag = fpitag_default;
                        }

                    } else {

                        std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                        std::cerr << "\t ... Can't find tag 'FPITAG'" << std::endl;
                        std::cerr << "\t ... disabling Fast PiTag " << std::endl;
                        m_use_fast_pi_tag = fpitag_default;
                    }

//************************************************************************************
//END   FiducialDetector->Fast PiTag flag
//************************************************************************************

//************************************************************************************
//        BEGIN FiducialDetector->PI
//************************************************************************************
                        // Tag element "ObjectDetectorParameters" of Xml Inifile

                        for(TiXmlElement* p_xmlElement_Root_FI = p_xmlElement_Root->FirstChildElement("PI");
                                p_xmlElement_Root_FI != NULL;
                                p_xmlElement_Root_FI = p_xmlElement_Root_FI->NextSiblingElement("PI"))
                        {
                                FiducialPiParameters pi_parameters;
//************************************************************************************
//        BEGIN FiducialDetector->PI->ID
//************************************************************************************
                                // Subtag element "ObjectDetectorParameters" of Xml Inifile
                                TiXmlElement *p_xmlElement_Child = NULL;
                                p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "ID" );

                                if ( p_xmlElement_Child )
                                {
                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "value", &pi_parameters.m_id) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'value' of tag 'ID'" << std::endl;
                                                return RET_FAILED;
                                        }
                                }
                                else
                                {
                                        std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                        std::cerr << "\t ... Can't find tag 'ID'" << std::endl;
                                        return RET_FAILED;
                                }


//************************************************************************************
//        BEGIN FiducialDetector->PI->LineWidthHeight
//************************************************************************************
                                // Subtag element "ObjectDetectorParameters" of Xml Inifile
                                p_xmlElement_Child = NULL;
                                p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "LineWidthHeight" );

                                if ( p_xmlElement_Child )
                                {
                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "value", &pi_parameters.line_width_height) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'value' of tag 'LineWidthHeight'" << std::endl;
                                                return RET_FAILED;
                                        }
                                }
                                else
                                {
                                        std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                        std::cerr << "\t ... Can't find tag 'LineWidthHeight'" << std::endl;
                                        return RET_FAILED;
                                }

//************************************************************************************
//        BEGIN FiducialDetector->PI->CrossRatioLine0
//************************************************************************************
                                // Subtag element "ObjectDetectorParameters" of Xml Inifile
                                p_xmlElement_Child = NULL;
                                p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "CrossRatioLine0" );

                                if ( p_xmlElement_Child )
                                {
                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "AB", &pi_parameters.d_line0_AB) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'AB' of tag 'CrossRatioLine0'" << std::endl;
                                                return RET_FAILED;
                                        }

                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "AC", &pi_parameters.d_line0_AC) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'AC' of tag 'CrossRatioLine0'" << std::endl;
                                                return RET_FAILED;
                                        }
                                }
                                else
                                {
                                        std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                        std::cerr << "\t ... Can't find tag 'CrossRatioLine0'" << std::endl;
                                        return RET_FAILED;
                                }

//************************************************************************************
//        BEGIN FiducialDetector->PI->CrossRatioLine1
//************************************************************************************
                                // Subtag element "ObjectDetectorParameters" of Xml Inifile
                                p_xmlElement_Child = NULL;
                                p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "CrossRatioLine1" );

                                if ( p_xmlElement_Child )
                                {
                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "AB", &pi_parameters.d_line1_AB) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'AB' of tag 'CrossRatioLine1'" << std::endl;
                                                return RET_FAILED;
                                        }

                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "AC", &pi_parameters.d_line1_AC) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'AC' of tag 'CrossRatioLine1'" << std::endl;
                                                return RET_FAILED;
                                        }
                                }
                                else
                                {
                                        std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                        std::cerr << "\t ... Can't find tag 'CrossRatioLine1'" << std::endl;
                                        return RET_FAILED;
                                }

//************************************************************************************
//        BEGIN FiducialDetector->PI->Offset
//************************************************************************************
                                // Subtag element "ObjectDetectorParameters" of Xml Inifile
                                p_xmlElement_Child = NULL;
                                p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "Offset" );

                                if ( p_xmlElement_Child )
                                {
                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "x", &pi_parameters.m_offset.x) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'x' of tag 'Offset'" << std::endl;
                                                return RET_FAILED;
                                        }

                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "y", &pi_parameters.m_offset.y) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'y' of tag 'Offset'" << std::endl;
                                                return RET_FAILED;
                                        }
                                }
                                else
                                {
                                        std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                        std::cerr << "\t ... Can't find tag 'Offset'" << std::endl;
                                        return RET_FAILED;
                                }

//************************************************************************************
//        BEGIN FiducialDetector->PI->SharpnessArea
//************************************************************************************
                                // Subtag element "ObjectDetectorParameters" of Xml Inifile
                                p_xmlElement_Child = NULL;
                                p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "SharpnessArea" );

                                if ( p_xmlElement_Child )
                                {
                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "x", &m_general_fiducial_parameters[pi_parameters.m_id].m_sharpness_pattern_area_rect3d.x) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'x' of tag 'SharpnessArea'" << std::endl;
                                                return RET_FAILED;
                                        }

                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "y", &m_general_fiducial_parameters[pi_parameters.m_id].m_sharpness_pattern_area_rect3d.y) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'y' of tag 'SharpnessArea'" << std::endl;
                                                return RET_FAILED;
                                        }

                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "width", &m_general_fiducial_parameters[pi_parameters.m_id].m_sharpness_pattern_area_rect3d.width) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'width' of tag 'SharpnessArea'" << std::endl;
                                                return RET_FAILED;
                                        }

                                        // read and save value of attribute
                                        if ( p_xmlElement_Child->QueryValueAttribute( "height", &m_general_fiducial_parameters[pi_parameters.m_id].m_sharpness_pattern_area_rect3d.height) != TIXML_SUCCESS)
                                        {
                                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                                std::cerr << "\t ... Can't find attribute 'height' of tag 'SharpnessArea'" << std::endl;
                                                return RET_FAILED;
                                        }
                                }
                                else
                                {
                                        std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                        std::cerr << "\t ... Can't find tag 'SharpnessArea'" << std::endl;
                                        return RET_FAILED;
                                }

                                m_general_fiducial_parameters[pi_parameters.m_id].m_offset = pi_parameters.m_offset;

                                vec_pi_parameters.push_back(pi_parameters);

//************************************************************************************
//        END FiducialDetector->Fiducial
//************************************************************************************
                        }
                        
                        if (vec_pi_parameters.empty())
                        {
                                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                                std::cerr << "\t ... Could't find tag 'PI'" << std::endl;
                                return RET_FAILED;
                        }

                }
//************************************************************************************
//        END FiducialDetector
//************************************************************************************
                else
                {
                        std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                        std::cerr << "\t ... Can't find tag 'FiducialDetector'" << std::endl;
                        return RET_FAILED;
                }
        }

        if (LoadParameters(vec_pi_parameters) & RET_FAILED)
        {
                std::cerr << "ERROR - FiducialModelPi::LoadParameters:" << std::endl;
                std::cerr << "\t ... Couldn't set tag parameters'" << std::endl;
                return RET_FAILED;
        }

        return RET_OK;
}
