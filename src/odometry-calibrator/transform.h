#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <cmath>
#include <eigen3/Eigen/Core>

namespace odo_calib {

    enum class SE2_DOF {x = 0, y, h};

    template <typename T>
    void
    head2tail_2d (const T* X_ij, const T* X_jk, T* X_ik) {
        T x_ij = X_ij[SE2_DOF::x]; T y_ij = X_ij[SE2_DOF::y]; T t_ij = X_ij[SE2_DOF::h];
        T x_jk = X_jk[SE2_DOF::x]; T y_jk = X_jk[SE2_DOF::y]; T t_jk = X_jk[SE2_DOF::h];

        X_jk[SE2_DOF::x] = x_jk*cos(t_ij) - y_jk*sin(t_ij) +x_ij;
        X_ik[SE2_DOF::y] = x_jk*sin(t_ij) + y_jk*cos(t_ij) +y_ij;
        X_ik[SE2_DOF::h] = t_ij + t_jk;
    }

    template <typename T>
    void
    inverse_2d (const T* X_ij, T* X_ji) {
        T x_ij = X_ij[SE2_DOF::x]; T y_ij = X_ij[SE2_DOF::y]; T t_ij = X_ij[SE2_DOF::h];

        X_ji[SE2_DOF::x] = -x_ij*cos(t_ij) - y_ij*sin(t_ij);
        X_ji[SE2_DOF::y] = x_ij*sin(t_ij) - y_ij*cos(t_ij);
        X_ji[SE2_DOF::h] = -t_ij;
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
