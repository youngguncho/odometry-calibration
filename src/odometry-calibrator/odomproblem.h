#ifndef ODOMPROBLEM_H
#define ODOMPROBLEM_H

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <map>

#include "ceres/ceres.h"

#include "transform.h"

using namespace std;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct OdometryGPSError {
    OdometryGPSError(double observed_x, double observed_y, double observed_h, double observed_enc_l, double observed_enc_r)
        : observed_x(observed_x), observed_y(observed_y) , observed_h(observed_h), observed_enc_l(observed_enc_l), observed_enc_r(observed_enc_r) {}

    template <typename T>
    bool operator()(const T* const dia_left,
                    const T* const dia_right,
                    const T* const wh_base,
                    T* residuals) const {

                T diameter_left = dia_left[0];
                T diemeter_right = dia_right[0];
                T wheel_base = wh_base[0];

                T dist_left = observed_enc_l * 3.14 * diameter_left * 0.00048828125;
                T dist_right = observed_enc_r * 3.14 * diemeter_right * 0.00048828125;

                T l_avg = (dist_left + dist_right) * 0.5;
                T l_diff = (dist_right - dist_left);

                T dth = l_diff / wheel_base;
                T dx = l_avg * cos(dth);
                T dy = l_avg * sin(dth);

//                T x_resi;
//                T y_resi;
//                T th_resi;

//                odo_calib::tail2tail_2d_elem(dx, dy, dth, observed_x, observed_y, observed_h, x_resi, y_resi, th_resi);

//                residuals[0] = x_resi;
//                residuals[1] = y_resi;
//                residuals[2] = th_resi;

                residuals[0] = observed_x-dx;
                residuals[1] = observed_y-dy;
                residuals[2] = observed_h-dth;

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y,
                                       const double observed_h,
                                       const double observed_enc_l,
                                       const double observed_enc_r) {
        return (new ceres::AutoDiffCostFunction<OdometryGPSError, 3, 1, 1, 1>(
                    new OdometryGPSError(observed_x, observed_y, observed_h, observed_enc_l, observed_enc_r)));
    }

    double observed_x;
    double observed_y;
    double observed_h;
    double observed_enc_l;
    double observed_enc_r;
};


class OdomProblem
{
public:
    OdomProblem(double d_l, double d_r, double w_b);
    ~OdomProblem();

    // Load data file
    bool LoadFile(const char* filename);

    // Prepare observations
    bool PrepareObservations();

    int num_observations() {return gps_pose_diff_vec_.size();}

    double get_observation_enc(int i, int j) {return encoder_diff_vec_[i][j];}
    double get_observation_gps(int i, int j) {return gps_pose_diff_vec_[i][j];}


private:

    // make initial data for
    bool MakeInitial_();
    bool FindKeys_();

    template<typename T>
    void FscanfOrDie(FILE* fptr, const char* format, T* value) {
        int num_scanned = fscanf(fptr, format, value);
        if (num_scanned != 1) {
            LOG(FATAL) << "Invalid data file";
        }
    }

    double diameter_left_;
    double diameter_right_;
    double wheel_base_;

    double init_diameter_left_;
    double init_diameter_right_;
    double init_wheel_base_;

    int num_odometries_;
    int num_gpses_;
    int num_observations_;

    int64_t* utime_odometries_;
    int64_t* utime_gpses_;
    int64_t* odometries_;
    double* gpses_;
    double* observations_;
    double* parameters_;

    // For raw data
    map < int64_t, vector<int64_t> > encoder_raw_map_;
    map < int64_t, vector<double> > gps_raw_map_;

    // For prepared data
    //
    vector <vector<double>> encoder_diff_vec_;
    vector <vector<double>> gps_pose_diff_vec_;

//    map < int64_t, vector<int64_t> > encoder_diff_map_;
//    map < int64_t, vector<double> > gps_pose_diff_map_;
};

#endif // ODOMPROBLEM_H
