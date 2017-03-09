#ifndef ODOMPROBLEM_H
#define ODOMPROBLEM_H

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <map>

#include "ceres/ceres.h"

using namespace std;

class OdomProblem
{
public:
    OdomProblem(double d_l, double d_r, double w_b);
    ~OdomProblem();

    // Load data file
    bool LoadFile(const char* filename);

    // Prepare observations
    bool PrepareObservations();


private:
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

    map < int64_t, vector<int64_t> > odom_map_;
    map < int64_t, vector<double> > gps_map_;
};

#endif // ODOMPROBLEM_H
