/**
* RUNETag fiducial markers library
*
* -----------------------------------------------------------------------------
* The MIT License (MIT)
* 
* Copyright (c) 2015 Filippo Bergamasco 
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
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
*/

#include <opencv2/core/core.hpp>
#include "ellipserefine.hpp"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <iostream>

cv::Point2d cv::runetag::ipa_Fiducials::ellipseCenter( const cv::Matx33d& e ) {
    const double& e00 = e(0,0);
    const double& e01 = e(0,1);
    const double& e11 = e(1,1);
    const double& e02 = e(0,2);
    const double& e12 = e(1,2);

    const double b2minac = e01*e01 - e00*e11;

    const double centerx = ( e11*e02 - e01*e12 ) / b2minac;
    const double centery = ( e00*e12 - e01*e02 ) / b2minac;
    return cv::Point2d( centerx, centery );
}

static inline void points( cv::RotatedRect& ellipse, cv::Point2f pt[] )
{
    double _angle = ellipse.angle*CV_PI/180.;
    float a = (float)cos(_angle)*0.5f;
    float b = (float)sin(_angle)*0.5f;
    //Revised by JIA Pei.
    pt[0].x = ellipse.center.x - a*ellipse.size.width - b*ellipse.size.height;
    pt[0].y = ellipse.center.y + b*ellipse.size.width - a*ellipse.size.height;
    pt[1].x = ellipse.center.x + a*ellipse.size.width - b*ellipse.size.height;
    pt[1].y = ellipse.center.y - b*ellipse.size.width - a*ellipse.size.height;
    pt[2].x = 2*ellipse.center.x - pt[0].x;
    pt[2].y = 2*ellipse.center.y - pt[0].y;
    pt[3].x = 2*ellipse.center.x - pt[1].x;
    pt[3].y = 2*ellipse.center.y - pt[1].y;
}

bool cv::runetag::ipa_Fiducials::ellipserefine( const cv::RotatedRect& ellipse, const cv::Mat& gradient_x, const cv::Mat& gradient_y, cv::Matx33d& out ) 
{
    const double major_axis = ellipse.size.width > ellipse.size.height ? ellipse.size.width : ellipse.size.height;
    const double s = major_axis / sqrt(2);

    cv::Point2d center = ellipse.center;

    cv::Point2i topleft( static_cast<int>( ellipse.center.x - major_axis*2.0 ) , static_cast<int>( ellipse.center.y - major_axis*2.0  ) );
    cv::Point2i bottomright( static_cast<int>( ellipse.center.x + major_axis*2.0  ), static_cast<int>( ellipse.center.y + major_axis*2.0  ) );

    if( topleft.x < 0 )
        topleft.x = 0;
    if( topleft.y < 0 )
        topleft.y = 0;
    if( bottomright.x >= gradient_x.cols )
        bottomright.x = gradient_x.cols-1;
    if( bottomright.y >= gradient_x.rows )
        bottomright.y = gradient_x.rows-1;

    cv::RotatedRect rmin( ellipse );
    rmin.size.width *= 0.1f;
    rmin.size.height *= 0.1f;
    cv::RotatedRect rmax( ellipse );
    rmax.size.width *= 2.0f;
    rmax.size.height *= 2.0f;

	double wbox = ellipse.size.width;
    double hbox = ellipse.size.height;
    double angle= ellipse.angle*CV_PI/180.;
    
    if( wbox < hbox )
    {
        wbox = ellipse.size.height;
        hbox = ellipse.size.width;
        angle+=CV_PI/2.;
    }

    double f1_x, f1_y,f2_x,f2_y;
    double focus_len = sqrt(wbox*wbox/4.-hbox*hbox/4.);
    double fx = std::cos(angle)*focus_len;
    double fy = std::sin(angle)*focus_len;
    double center_x = ellipse.center.x;
    double center_y = ellipse.center.y;
    f1_x = center_x - fx;
    f1_y = center_y - fy;
    f2_x = center_x + fx;
    f2_y = center_y + fy;
    double min_fsum = (wbox - wbox/2);
    double max_fsum = (wbox + wbox/2);


    cv::Point2f min_points[4];
    points( rmin, min_points );

    cv::Point2f max_points[4];
    points( rmax, max_points );


    std::vector< cv::Mat > lines;
    std::vector< double > weights;

    // Create lines set
    for( int i=topleft.y; i<bottomright.y; i++ ) {
        for( int j=topleft.x; j<bottomright.x; j++ ) {
            double dk1 = std::sqrt((f1_x-j)*(f1_x-j) + (f1_y-i)*(f1_y-i));
            double dk2 = std::sqrt((f2_x-j)*(f2_x-j) + (f2_y-i)*(f2_y-i));
            if( (dk1 + dk2 < min_fsum) || (dk1 + dk2 > max_fsum) )
                continue;

            const double x = (j - center.x)/s;
            const double y = (i - center.y)/s;

			const double Iui = ((double)gradient_x.at<signed short>( i,j ) / 32768.0);
            const double Ivi = ((double)gradient_y.at<signed short>( i,j ) / 32768.0);

            const double weight = Iui*Iui + Ivi*Ivi;
            if( weight < 1E-10 )
				continue;

            cv::Mat li(3,1,CV_64F );
            li.at<double>(0,0) = Iui;
            li.at<double>(1,0) = Ivi;
            li.at<double>(2,0) = -( Iui*x + Ivi*y );

            lines.push_back( li );
            weights.push_back( weight );
        }
    }
    //std::cout << "Num lines: " << lines.size() << std::endl;
    if( lines.size() < 30 )
        return false;

    cv::Mat A = cv::Mat::zeros(5,5,CV_64F);
    cv::Mat B = cv::Mat::zeros(5,1,CV_64F);
    cv::Mat k = cv::Mat(5,1,CV_64F);
    cv::Mat kt = cv::Mat(1,5,CV_64F);
    for( unsigned int i=0; i<lines.size(); i++ ) {

        const double a = lines[i].at<double>(0,0);
        const double b = lines[i].at<double>(1,0);
        const double c = lines[i].at<double>(2,0);

        k.at<double>(0,0) = kt.at<double>(0,0) = a*a;
        k.at<double>(1,0) = kt.at<double>(0,1) = a*b;
        k.at<double>(2,0) = kt.at<double>(0,2) = b*b;
        k.at<double>(3,0) = kt.at<double>(0,3) = a*c;
        k.at<double>(4,0) = kt.at<double>(0,4) = b*c;

        A = A + weights[i]*k*kt;
        B = B + ( k*(c*c)*(-weights[i]) );
    }


    cv::Mat O(5,1,CV_64F );
    cv::solve( A, B, O, cv::DECOMP_SVD );

    cv::Mat Om( 3,3, CV_64F );
    Om.at<double>(0,0) = O.at<double>(0,0) ;
    Om.at<double>(0,1) = O.at<double>(1,0) /2.0;
    Om.at<double>(0,2) = O.at<double>(3,0) /2.0;

    Om.at<double>(1,0) = O.at<double>(1,0)/2.0;
    Om.at<double>(1,1) = O.at<double>(2,0);
    Om.at<double>(1,2) = O.at<double>(4,0)/2.0;

    Om.at<double>(2,0) = O.at<double>(3,0)/2.0;
    Om.at<double>(2,1) = O.at<double>(4,0)/2.0;
    Om.at<double>(2,2) = 1.0;


    cv::Mat e = Om.inv(); 

    cv::Mat Ainvt(3,3,CV_64F);
    Ainvt.at<double>(0,0) = 1.0/s;
    Ainvt.at<double>(0,1) = 0.0;
    Ainvt.at<double>(0,2) = 0.0;

    Ainvt.at<double>(1,0) = 0.0;
    Ainvt.at<double>(1,1) = 1.0/s;
    Ainvt.at<double>(1,2) = 0.0;

    Ainvt.at<double>(2,0) = -center.x/s;
    Ainvt.at<double>(2,1) = -center.y/s;
    Ainvt.at<double>(2,2) = 1.0;

    cv::Mat Ainv(3,3,CV_64F);
    cv::transpose( Ainvt, Ainv );

	cv::Mat res = (Ainvt*e*Ainv);
    out = res;
	return true;
}
