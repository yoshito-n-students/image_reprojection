/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef __CV_EXTENSION_CALIB3D_CALIB3D_HPP__
#define __CV_EXTENSION_CALIB3D_CALIB3D_HPP__

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <cv_extension/core/affine.hpp>

namespace cv
{

namespace fisheye
{
    enum{
        CALIB_USE_INTRINSIC_GUESS   = 1,
        CALIB_RECOMPUTE_EXTRINSIC   = 2,
        CALIB_CHECK_COND            = 4,
        CALIB_FIX_SKEW              = 8,
        CALIB_FIX_K1                = 16,
        CALIB_FIX_K2                = 32,
        CALIB_FIX_K3                = 64,
        CALIB_FIX_K4                = 128,
        CALIB_FIX_INTRINSIC         = 256
    };

    //! projects 3D points using fisheye model
    CV_EXPORTS void projectPoints(InputArray objectPoints, OutputArray imagePoints, const Affine3d& affine,
				    InputArray K, InputArray D, double alpha = 0, OutputArray jacobian = noArray());

    //! projects points using fisheye model
    CV_EXPORTS void projectPoints(InputArray objectPoints, OutputArray imagePoints, InputArray rvec, InputArray tvec,
        InputArray K, InputArray D, double alpha = 0, OutputArray jacobian = noArray());

    //! distorts 2D points using fisheye model
    CV_EXPORTS void distortPoints(InputArray undistorted, OutputArray distorted, InputArray K, InputArray D, double alpha = 0);

    //! undistorts 2D points using fisheye model
    CV_EXPORTS void undistortPoints(InputArray distorted, OutputArray undistorted,
        InputArray K, InputArray D, InputArray R = noArray(), InputArray P  = noArray());

    //! [PROPRIETARY] undistorts 2D points to 3D using fisheye model
    CV_EXPORTS void undistortPointsTo3D(InputArray distorted, OutputArray undistorted,
        InputArray K, InputArray D, InputArray R = noArray(), InputArray P  = noArray());

    //! computing undistortion and rectification maps for image transform by cv::remap()
    //! If D is empty zero distortion is used, if R or P is empty identity matrixes are used
    CV_EXPORTS void initUndistortRectifyMap(InputArray K, InputArray D, InputArray R, InputArray P,
        const cv::Size& size, int m1type, OutputArray map1, OutputArray map2);

    //! undistorts image, optionally changes resolution and camera matrix. If Knew zero identity matrix is used
    CV_EXPORTS void undistortImage(InputArray distorted, OutputArray undistorted,
        InputArray K, InputArray D, InputArray Knew = cv::noArray(), const Size& new_size = Size());

    //! estimates new camera matrix for undistortion or rectification
    CV_EXPORTS void estimateNewCameraMatrixForUndistortRectify(InputArray K, InputArray D, const Size &image_size, InputArray R,
        OutputArray P, double balance = 0.0, const Size& new_size = Size(), double fov_scale = 1.0);

    //! performs camera calibaration
    CV_EXPORTS double calibrate(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints, const Size& image_size,
        InputOutputArray K, InputOutputArray D, OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs, int flags = 0,
            TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON));

    //! stereo rectification estimation
    CV_EXPORTS void stereoRectify(InputArray K1, InputArray D1, InputArray K2, InputArray D2, const Size &imageSize, InputArray R, InputArray tvec,
        OutputArray R1, OutputArray R2, OutputArray P1, OutputArray P2, OutputArray Q, int flags, const Size &newImageSize = Size(),
        double balance = 0.0, double fov_scale = 1.0);

    //! performs stereo calibaration
    CV_EXPORTS double stereoCalibrate(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints1, InputArrayOfArrays imagePoints2,
                                  InputOutputArray K1, InputOutputArray D1, InputOutputArray K2, InputOutputArray D2, Size imageSize,
                                  OutputArray R, OutputArray T, int flags = CALIB_FIX_INTRINSIC,
                                  TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON));

}

}

#endif
