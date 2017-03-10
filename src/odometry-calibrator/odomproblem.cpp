#include "odomproblem.h"

OdomProblem::OdomProblem(double d_l, double d_r, double w_b)
    : diameter_left_(d_l), diameter_right_(d_r), wheel_base_(w_b)
{
    cout << "initialize" << endl;
}

OdomProblem::~OdomProblem()
{
    delete[] odometries_;
    delete[] gpses_;
    delete[] utime_odometries_;
    delete[] utime_gpses_;
}

struct OdometryError {

};

bool OdomProblem::LoadFile(const char *filename) {
    FILE *fptr = fopen(filename, "r");
    if (fptr == NULL) {
        cerr << "File not opened" << endl;
        return false;
    }

    FscanfOrDie(fptr, "%ld", &num_odometries_);
    FscanfOrDie(fptr, "%ld", &num_gpses_);

    utime_odometries_ = new int64_t[num_odometries_];
    utime_gpses_ = new int64_t[num_gpses_];
    odometries_ = new int64_t[2 * num_odometries_];
    gpses_ = new double[3 * num_gpses_];

    // parse odometry first (utime, encoder_left, encoder_right)
    for (int i = 0; i < num_odometries_; ++i) {
        FscanfOrDie(fptr, "%ld", utime_odometries_ + i);
        vector <int64_t> odo_counter;
        for (int j = 0; j < 2; ++j) {
            FscanfOrDie(fptr, "%ld", odometries_ + 2*i + j);
            odo_counter.push_back(odometries_[2*i + j]);
        }
        odom_map_[*(utime_odometries_ + i)] = odo_counter;
    }

    // parse gps pose data (x_global, y_global, th_global)
    for (int i = 0; i < num_gpses_; ++i) {
        FscanfOrDie(fptr, "%ld", utime_gpses_ + i);
        vector <double> gps_pose;
        for (int j = 0; j < 3; ++j) {
            FscanfOrDie(fptr, "%f", gpses_ + 2*i + j);
            gps_pose.push_back(*(gpses_ + 2*i + j));
        }
        gps_map_[*(utime_gpses_ + i)] = gps_pose;
    }

    return true;
}

bool OdomProblem::PrepareObservations()
{
    using value_type = map<int64_t, vector<double>>::value_type;

    // find match odometry list as iterator of map
    vector <map<int64_t, vector<int64_t>>::iterator> match_odo_list;
    for_each(begin(gps_map_), end(gps_map_), [&](value_type const& gps) {
        map<int64_t, vector<int64_t>>::iterator match_iter;
        if (odom_map_.find(gps.first) != end(odom_map_))
            match_iter = odom_map_.find(gps.first);
        else
            match_iter = odom_map_.lower_bound(gps.first);

        match_odo_list.push_back(match_iter);
    });

    // convert dataset to set observation and parameters
    // now we have matched odometry lists
    // then we need to convert odometry and gps to relative poses
    // TODO: build encoder counter diff vector for left and right
    // TODO: write SE2 motion matrix
    // TODO: viewer will be main distributor, and calibrator will be a lib


    return true;
}
