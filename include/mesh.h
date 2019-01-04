//
// Created by daichengkai on 29-4-17.
//

#ifndef VOXELFAB_MESH_H
#define VOXELFAB_MESH_H

#include <Eigen/Dense>

class MyMesh {
public:
    MyMesh(const Eigen::MatrixXd &_V, const Eigen::MatrixXi &_F) :
            vectices(_V), faces(_F) {};

    MyMesh(MyMesh *mesh) :
            vectices(mesh->vectices), faces(mesh->faces) {};

    MyMesh(void) {};

    ~MyMesh() = default;

public:

    Eigen::MatrixXd vectices;

    Eigen::MatrixXi faces;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif //VOXELFAB_MESH_H
