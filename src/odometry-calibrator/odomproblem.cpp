#include "odomproblem.h"
#include "transform.h"

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

struct OdometryGPSError {
    OdometryGPSError(double observed_enc_l, double observed_enc_r, double observed_x, double observed_y, double observed_h)
        : observed_enc_l(observed_enc_l), observed_enc_r(observed_enc_r),
          observed_x(observed_x), observed_y(observed_y), observed_h(observed_h) {}

    template <typename T>
    bool operator ()(const T* const odo_variables,
                     T* residuals) const {

        double diameter_left = odo_variables[0];
        double diemeter_right = odo_variables[1];
        double wheel_base = odo_variables[2];

        double dist_left = observed_enc_l * M_PI * diameter_left / 2048;
        double dist_right = observed_enc_r * M_PI * diemeter_right / 2048;

        double l_avg = (dist_left + dist_right) / 2;
        double l_diff = (dist_right - dist_left);

        double dth = l_diff / wheel_base;
        double dx = l_avg * cos(dth);
        double dy = l_avg * sin(dth);

        double X_odo[3] = {dx, dy, dth};
        double X_gps[3] = {observed_x, observed_y, observed_h};

        odo_calib::tail2tail_2d(X_odo, X_gps, residuals);
//        residuals[0] = 0;
//        residuals[1] = 0;
//        residuals[2] = 0;
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
//    static CostFunction* Create(const double observed_enc_l, const double observed_enc_r,
//                                       const double observed_x, const double observed_y, const double observed_h) {
//        return (new AutoDiffCostFunction<OdometryGPSError, 1, 1, 1>(
//                    new OdometryGPSError(observed_enc_l, observed_enc_r, observed_x, observed_y, observed_h)));
//    }


    double observed_enc_l;
    double observed_enc_r;
    double observed_x;
    double observed_y;
    double observed_h;

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
        encoder_raw_map_[*(utime_odometries_ + i)] = odo_counter;
    }

    // parse gps pose data (x_global, y_global, th_global)
    for (int i = 0; i < num_gpses_; ++i) {
        FscanfOrDie(fptr, "%ld", utime_gpses_ + i);
        vector <double> gps_pose;
        for (int j = 0; j < 3; ++j) {
            FscanfOrDie(fptr, "%lf", gpses_ + 3*i + j);
            gps_pose.push_back(*(gpses_ + 3*i + j));
        }
        gps_raw_map_[*(utime_gpses_ + i)] = gps_pose;
    }

    return true;
}

// yg-tools
// TODO call iterator->second[0] is not working well. first and second[1] are good.
template <typename K, typename V>
using map_iter = typename map<K, vector<V>>::iterator;

template <typename K, typename V>
bool
find_key_in_mapvector (map< K, vector<V>> mapvector, K key, map_iter<K, V> &match_iter)
{
    match_iter = mapvector.find(key);
    if (match_iter == end(mapvector)) {
        match_iter = mapvector.lower_bound(key);
    }

//    return match_iter;
}


bool OdomProblem::PrepareObservations()
{
    // prepare initial state of encoder and GPS
    auto init_gps_iter = begin(gps_raw_map_);
    auto init_encoder_iter = begin(encoder_raw_map_);
    find_key_in_mapvector(encoder_raw_map_, init_gps_iter->first, init_encoder_iter);
    if (init_encoder_iter == end(encoder_raw_map_)) {
        cerr << "Not valid encoder and GPS dataset" << endl;
        return false;
    }
    else {
        init_gps_iter++;
    }

    // find match odometry list as iterator of map
    using value_type = map<int64_t, vector<double>>::value_type;
    auto pre_encoder_iter = init_encoder_iter;
    vector <map<int64_t, vector<int64_t>>::iterator> match_odo_list;

    int64_t pre_gps_time = begin(gps_raw_map_)->first;
    int64_t pre_encoder_time = init_encoder_iter->first;
    for_each(init_gps_iter, end(gps_raw_map_), [&](value_type const& gps) {
        auto match_iter = begin(encoder_raw_map_);
        find_key_in_mapvector(encoder_raw_map_, gps.first, match_iter);

        cout << "Find Odometry: " << match_iter->first << " / " << encoder_raw_map_[match_iter->first][0] << endl;
        match_odo_list.push_back(match_iter);

        // prepare encoder
        int64_t encoder_time = match_iter->first;
        vector <double> encoder_diff;
        double enc_diff_left = (double) encoder_raw_map_[encoder_time][0] - encoder_raw_map_[pre_encoder_time][0];
        double enc_diff_right = (double) encoder_raw_map_[encoder_time][1] - encoder_raw_map_[pre_encoder_time][1];
        encoder_diff.push_back(enc_diff_left);
        encoder_diff.push_back(enc_diff_right);
        encoder_diff_vec_.push_back(encoder_diff);

        // prepare gps
        int64_t gps_time = gps.first;
        vector <double> X_ij = gps_raw_map_[pre_gps_time];
        vector <double> X_ik = gps_raw_map_[gps_time];
        vector <double> X_jk (3, 0);
        odo_calib::tail2tail_2d(X_ij.data(), X_ik.data(), X_jk.data());
        gps_pose_diff_vec_.push_back(X_jk);

        pre_gps_time = gps_time;
        pre_encoder_time = encoder_time;
    });

    // TODO: viewer will be main distributor, and calibrator will be a lib


    return true;
}

bool OdomProblem::MakeInitial_()
{
    return true;
}

bool OdomProblem::FindKeys_()
{
    return true;
}
