//
// Created by daichengkai on 9-5-17.
//

#include "trimming.h"


void
trimming(Eigen::MatrixXd &VA, Eigen::MatrixXi &FA, Eigen::MatrixXd &VB, Eigen::MatrixXi &FB, Eigen::MatrixXd &VC,
         Eigen::MatrixXi &FC) {


    Eigen::MatrixXd N;
    Eigen::VectorXi J, I;
    igl::copyleft::cgal::trim_with_solid(VA, FA, VB, FB, VC, FC, I, J);
    std::cout << "\nTrim Completed" << std::endl;
    Eigen::MatrixXi NFC(0, 3);
    for (unsigned i = 0; i < I.rows(); ++i) {
        if (!I[i]) {
            Eigen::MatrixXi tempF(NFC.rows() + 1, NFC.cols());
            tempF << NFC,
                    FC.row(i);
            NFC = tempF;
        }
    }

    Eigen::MatrixXd NV;
    Eigen::MatrixXi NF;
    Eigen::VectorXi NI;
    igl::remove_unreferenced(VC, NFC, NV, NF, NI);

    VC = NV;
    FC = NF;
}
