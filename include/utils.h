//
// Created by daichengkai on 1/2/19.
//

#ifndef VOXELMULTIAXISAM_UTILS_H
#define VOXELMULTIAXISAM_UTILS_H

inline void reorient_mesh(const Eigen::MatrixXd &VA, const Eigen::MatrixXi &FA,Eigen::MatrixXd &VB, Eigen::MatrixXi &FB) {
    VB=VA;
    FB=FA;
    for (size_t i = 0; i < VA.rows(); ++i) {
        double z = VA.row(i).y();
        double x = -VA.row(i).x();
        double y = VA.row(i).z();
        VB.row(i).x() = x;
        VB.row(i).y() = y;
        VB.row(i).z() = z;
    }
    for (size_t i = 0; i < FA.rows(); ++i) {
        int x = FA.row(i).x();
        int y = FA.row(i).y();
        int z = FA.row(i).z();
        FB.row(i).x() = x;
        FB.row(i).y() = y;
        FB.row(i).z() = z;
    }

}

inline void
append_mesh(Eigen::MatrixXd &V1, Eigen::MatrixXi &F1, Eigen::MatrixXd &V2, Eigen::MatrixXi &F2, Eigen::MatrixXd &V3,
            Eigen::MatrixXi &F3) {
    auto start_V = V1.rows();
    auto start_F = F1.rows();
    Eigen::MatrixXi F_old = F1;
    Eigen::MatrixXd V_old = V1;
    F3.resize(start_F + F2.rows(), 3);
    V3.resize(start_V + V2.rows(), 3);

    for (auto i = 0; i < V_old.rows(); ++i) {
        V3.row(i) << V_old.row(i);
    }

    for (auto i = 0; i < V2.rows(); ++i) {
        V3.row(i + start_V) << V2.row(i);
    }

    for (auto i = 0; i < F_old.rows(); ++i) {
        F3.row(i) << F_old.row(i);
    }
    for (auto i = 0; i < F2.rows(); ++i) {
        F3.row(i + start_F) << F2(i, 0) + start_V, F2(i, 1) + start_V, F2(i, 2) + start_V;
    }
}

inline void
append_matrix(Eigen::MatrixXd &V1, Eigen::MatrixXd &V2, Eigen::MatrixXd &V3) {
    auto start_V = V1.rows();
    Eigen::MatrixXd V_old = V1;
    V3.resize(start_V + V2.rows(), 3);

    for (auto i = 0; i < V_old.rows(); ++i) {
        V3.row(i) << V_old.row(i);
    }

    for (auto i = 0; i < V2.rows(); ++i) {
        V3.row(i + start_V) << V2.row(i);
    }
}

inline void combine_meshes(std::vector<Eigen::MatrixXd> &V_set, std::vector<Eigen::MatrixXi> &F_set, Eigen::MatrixXd &V,
                           Eigen::MatrixXi &F) {
    int v_size = 0;
    int f_size = 0;
    for (int i = 0; i < V_set.size(); ++i) {
        v_size += V_set[i].rows();
        f_size += F_set[i].rows();
    }

    V.resize(v_size, 3);
    F.resize(f_size, 3);
    int v_count = 0;

    for (int i = 0; i < V_set.size(); ++i) {
        int rows = V_set[i].rows();
        V.block(v_count, 0, rows, 3) = V_set[i];
        v_count += V_set[i].rows();

    }
    int f_count = 0;
    v_count = 0;
    for (int i = 0; i < F_set.size(); ++i) {
        int rows = F_set[i].rows();
        F.block(f_count, 0, rows, 3) = (F_set[i].array() + v_count);

        f_count += F_set[i].rows();
        v_count += V_set[i].rows();
    }
}


inline void center_to_platform(Eigen::MatrixXd &V) {
    Eigen::Vector3d pmin = V.colwise().minCoeff();

    Eigen::RowVector3d center(0, pmin(1), 0);
    for (auto i = 0; i < V.rows(); ++i) {
        V.row(i) -= center;
    }

}


#endif //VOXELMULTIAXISAM_UTILS_H
