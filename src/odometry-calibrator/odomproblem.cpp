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

struct OdometryGPSError {
    OdometryGPSError(double observed_x, double observed_y, double observed_h)
        : observed_x(observed_x), observed_y(observed_y), observed_h(observed_h) {}

    template <typename T>
    bool operator ()(const T* const odo_variables,
                     T* residuals) const {

    }

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
            FscanfOrDie(fptr, "%f", gpses_ + 2*i + j);
            gps_pose.push_back(*(gpses_ + 2*i + j));
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
map_iter<K, V>
find_key_in_mapvector (map< K, vector<V>> mapvector, K key)
{
    auto match_iter = mapvector.find(key);
    if (match_iter == end(mapvector)) {
        match_iter = mapvector.lower_bound(key);
    }

    return match_iter;
}


bool OdomProblem::PrepareObservations()
{
    // prepare initial state of encoder and GPS
    auto init_gps_iter = begin(gps_raw_map_);
    auto init_encoder_iter = find_key_in_mapvector(encoder_raw_map_, init_gps_iter->first);
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
    for_each(init_gps_iter, end(gps_raw_map_), [&](value_type const& gps) {
        auto match_iter = find_key_in_mapvector(encoder_raw_map_, gps.first);

        cout << "Find Odometry: " << match_iter->first << " / " << encoder_raw_map_[match_iter->first][0] << " / " << match_iter->second[1] << endl;
        cout << "Find Odometry: " << match_iter->first << " / " << encoder_raw_map_[match_iter->first][0] << endl;
        match_odo_list.push_back(match_iter);


    });





    // convert dataset to set observation and parameters
    // now we have matched odometry lists
    // then we need to convert odometry and gps to relative poses
    // TODO: build encoder counter diff vector for left and right
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
