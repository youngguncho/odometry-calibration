#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <cmath>
#include <eigen3/Eigen/Core>

namespace odo_calib {

    enum class SE2_DOF {x = 0, y, h};

    template <typename T>
    void
    head2tail_2d (const T* X_ij, const T* X_jk, T* X_ik) {
        T x_ij = X_ij[0]; T y_ij = X_ij[1]; T t_ij = X_ij[2];
        T x_jk = X_jk[0]; T y_jk = X_jk[1]; T t_jk = X_jk[2];

        X_ik[0] = x_jk*cos(t_ij) - y_jk*sin(t_ij) +x_ij;
        X_ik[1] = x_jk*sin(t_ij) + y_jk*cos(t_ij) +y_ij;
        X_ik[2] = t_ij + t_jk;
    }

    template <typename T>
    void
    inverse_2d (const T* X_ij, T* X_ji) {
        T x_ij = X_ij[0]; T y_ij = X_ij[1]; T t_ij = X_ij[2];

        X_ji[0] = -x_ij*cos(t_ij) - y_ij*sin(t_ij);
        X_ji[1] = x_ij*sin(t_ij) - y_ij*cos(t_ij);
        X_ji[2] = -t_ij;
    }

    template <typename T>
    void
    tail2tail_2d (const T* X_ij, const T* X_ik, T* X_jk) {
        T X_ji[3];
        inverse_2d(X_ij, X_ji);
        head2tail_2d(X_ji, X_ik, X_jk);
    }
}


#endif // TRANSFORM_H
