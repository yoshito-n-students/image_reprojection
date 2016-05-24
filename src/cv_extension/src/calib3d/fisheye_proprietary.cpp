#include "precomp.hpp"
#include "fisheye.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// cv::fisheye::undistortPointsTo3D

void cv::fisheye::undistortPointsTo3D(InputArray distorted, OutputArray undistorted, InputArray K,
                                      InputArray D, InputArray R, InputArray P) {
    // will support only 2-channel data now for points
    CV_Assert(distorted.type() == CV_32FC2 || distorted.type() == CV_64FC2);
    undistorted.create(distorted.size(), CV_MAKETYPE(distorted.depth(), 3));

    CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));
    CV_Assert(R.empty() || R.size() == Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(D.total() == 4 && K.size() == Size(3, 3) &&
              (K.depth() == CV_32F || K.depth() == CV_64F));

    cv::Vec2d f, c;
    if (K.depth() == CV_32F) {
        Matx33f camMat = K.getMat();
        f = Vec2f(camMat(0, 0), camMat(1, 1));
        c = Vec2f(camMat(0, 2), camMat(1, 2));
    } else {
        Matx33d camMat = K.getMat();
        f = Vec2d(camMat(0, 0), camMat(1, 1));
        c = Vec2d(camMat(0, 2), camMat(1, 2));
    }

    Vec4d k = D.depth() == CV_32F ? (Vec4d)*D.getMat().ptr<Vec4f>() : *D.getMat().ptr<Vec4d>();

    cv::Matx33d RR = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3) {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = cv::Affine3d(rvec).rotation();
    } else if (!R.empty() && R.size() == Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    if (!P.empty()) {
        cv::Matx33d PP;
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);
        RR = PP * RR;
    }

    // start undistorting
    const cv::Vec2f* srcf = distorted.getMat().ptr<cv::Vec2f>();
    const cv::Vec2d* srcd = distorted.getMat().ptr<cv::Vec2d>();
    cv::Vec3f* dstf = undistorted.getMat().ptr<cv::Vec3f>();
    cv::Vec3d* dstd = undistorted.getMat().ptr<cv::Vec3d>();

    size_t n = distorted.total();
    int sdepth = distorted.depth();

    for (size_t i = 0; i < n; i++) {
        cv::Vec2d pi = sdepth == CV_32F ? (Vec2d)srcf[i] : srcd[i];      // image point
        cv::Vec3d pw((pi[0] - c[0]) / f[0], (pi[1] - c[1]) / f[1], 1.);  // world point

        double theta_d = sqrt(pw[0] * pw[0] + pw[1] * pw[1]);
        double theta = theta_d;
        if (theta_d > 1e-8) {
            // compensate distortion iteratively
            for (int j = 0; j < 10; j++) {
                double theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2,
                       theta8 = theta6 * theta2;
                theta =
                    theta_d / (1 + k[0] * theta2 + k[1] * theta4 + k[2] * theta6 + k[3] * theta8);
            }
        }

        cv::Vec3d pu;  // undistorted point
        {
            const double angle_rot = theta - std::atan(theta_d);
            const cv::Vec3d axis_rot(-pw[1], pw[0], 0.);
            const cv::Vec3d vec_rot = cv::normalize(axis_rot) / angle_rot;
            cv::Matx33d mat_rot;
            cv::Rodrigues(vec_rot, mat_rot);
            pu = mat_rot * pw;
        }

        // reproject
        Vec3d pr = RR * pu;  // rotated point optionally multiplied by new camera matrix
        Vec3d fi = pr;       // final

        if (sdepth == CV_32F)
            dstf[i] = fi;
        else
            dstd[i] = fi;
    }
}
