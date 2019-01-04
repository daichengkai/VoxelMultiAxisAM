//
// Created by daichengkai on 9-5-17.
//

#ifndef VOXELFAB_TRIMMING_H
#define VOXELFAB_TRIMMING_H

#include <igl/remove_unreferenced.h>
#include <igl/copyleft/cgal/trim_with_solid.h>
#include <iostream>


void
trimming(Eigen::MatrixXd &VA, Eigen::MatrixXi &FA, Eigen::MatrixXd &VB, Eigen::MatrixXi &FB, Eigen::MatrixXd &VC,
         Eigen::MatrixXi &FC);


#endif //VOXELFAB_TRIMMING_H
