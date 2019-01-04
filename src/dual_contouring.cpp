//
// Created by daichengkai on 3-5-17.
//

#include "dual_contouring.h"

void DualContouring::LoadIsovalueSet(VoxelGrid* voxel_set) {

    triangle_count=0;

    isovalue_set.clear();

    std::vector<voxel* > voxels;
    voxel_set->getAllVoxels(voxels);
    for (int i = 0; i < voxels.size(); ++i) {
        FieldNode *node =new FieldNode;
        node->x = voxels[i]->x;
        node->y = voxels[i]->z;
        node->z = voxels[i]->y;
        if (voxels[i]->layer>0)
            node->value = voxels[i]->layer;
        else
            node->value = -1;
        node->position=Eigen::Vector3d(-voxels[i]->position(0),voxels[i]->position(2),voxels[i]->position(1));

        isovalue_set.push_back(node);
    }
    res=Eigen::Vector3i(voxel_set->res(0),voxel_set->res(2),voxel_set->res(1));
    bGrid.resize(isovalue_set.size());
    for(int i=0;i<bGrid.size();++i)
        bGrid[i]=false;

    std::cout<<"Field Nodes size is "<<isovalue_set.size()<<std::endl;

    maxx=0;minx=1000;
    maxy=0;miny=1000;
    maxz=0;minz=1000;
    for(int i=0;i<isovalue_set.size();++i){
        if(isovalue_set[i]->value>=1){
            if(isovalue_set[i]->x>=maxx)
                maxx=isovalue_set[i]->x;
            if(isovalue_set[i]->y>=maxy)
                maxy=isovalue_set[i]->y;
            if(isovalue_set[i]->z>=maxz)
                maxz=isovalue_set[i]->z;
            if(isovalue_set[i]->x<=minx)
                minx=isovalue_set[i]->x;
            if(isovalue_set[i]->y<=miny)
                miny=isovalue_set[i]->y;
            if(isovalue_set[i]->z<=minz)
                minz=isovalue_set[i]->z;
        }
    }

    max_layer=0;
    for(int i=0;i<isovalue_set.size();++i){
        if(isovalue_set[i]->value>max_layer)
            max_layer=isovalue_set[i]->value;
    }

    voxels.clear();
}

void
DualContouring::GetRelatedSet(double isovalue, int &x_min, int &x_max, int &y_min, int &y_max, int &z_min, int &z_max) {
    int low_value = (int) floor(isovalue);
    int high_value = (int) ceil(isovalue);

    std::vector<Eigen::Vector3i> indices;
    Eigen::MatrixXi indices_mat;
    for (size_t i = 0; i < isovalue_set.size(); ++i) {

        if (isovalue_set[i]->value == low_value)
            indices.push_back(Eigen::Vector3i(isovalue_set[i]->x, isovalue_set[i]->y, isovalue_set[i]->z));


    }

    indices_mat.resize(indices.size(), 3);
    for (size_t i = 0; i < indices.size(); ++i) {
        indices_mat.row(i) = indices[i];
    }

    x_min = indices_mat.colwise().minCoeff()(0);
    y_min = indices_mat.colwise().minCoeff()(1);
    z_min = indices_mat.colwise().minCoeff()(2);
    x_max = indices_mat.colwise().maxCoeff()(0);
    y_max = indices_mat.colwise().maxCoeff()(1);
    z_max = indices_mat.colwise().maxCoeff()(2);

    if(x_min-1>=minx) x_min=x_min-1;
    if(y_min-1>=miny) y_min=y_min-1;
    if(z_min-1>=minz) z_min=z_min-1;
    if(x_max+1<maxx) x_max=x_max+1;
    if(y_max+1<maxy) y_max=y_max+1;
    if(z_max+1<maxz) z_max=z_max+1;

}

bool DualContouring::DoContouring(double isovalue,Eigen::MatrixXd &vertices, Eigen::MatrixXi &faces){

    int x_min, x_max, y_min, y_max, z_min, z_max;
    std::vector<Eigen::Vector3d> field_nodes_vec;
    std::vector<Eigen::Vector3d> intersect_nodes_vec;
    std::vector<Eigen::Vector3d> test_points;

    index_map vindex;



    std::vector<Eigen::Vector3d> dc_vertices;
    Eigen::Vector3i x_dir(1, 0, 0);
    Eigen::Vector3i y_dir(0, 1, 0);
    Eigen::Vector3i z_dir(0, 0, 1);
    std::vector<Eigen::Vector3i> dirs = {x_dir, y_dir, z_dir};
    std::vector<Eigen::Vector3i> dc_faces;
    const int far_edge[3][2] = {{3,7}, {5,7}, {6,7}};

    this->GetRelatedSet(isovalue, x_min, x_max, y_min, y_max, z_min, z_max);



    for (int zz = z_min; zz <= z_max; zz++) {
        for (int yy = y_min; yy <= y_max; yy++) {
            for (int xx = x_min; xx <= x_max; xx++) {
                FieldNode *node = this->GetNode(xx, yy, zz);
                if (this->IsSafeNode(node)) {
                    field_nodes_vec.push_back(node->position);

                    std::vector<Eigen::Vector3d> intersect_points;
                    std::vector<Eigen::Vector3d> normals;


                    if (this->GetHermiteData(isovalue, node, intersect_points, normals)) {

                        Eigen::Vector3d dc_vertex;
                        //this->SolveQEF(intersect_points,normals,dc_vec);
                        this->GetMassPoint(intersect_points, dc_vertex);

                        vindex[std::make_tuple(xx, yy, zz)] = dc_vertices.size();
                        dc_vertices.push_back(dc_vertex);

                    }
                }
            }
        }
    }

    for (int zz = z_min; zz <= z_max; zz++) {
        for (int yy = y_min; yy <= y_max; yy++) {
            for (int xx = x_min; xx <= x_max; xx++) {
                Eigen::Vector3i id(xx, yy, zz);
                index_t current_t = this->MakeIndex(id);
                if (!this->FindIndex(current_t, vindex))
                    continue;
                else {
                    bool inside[8];

                    for (int i=0; i<8; ++i) {
                        int x_neighbor = xx + cube_neighbor[i][0];
                        int y_neighbor = yy + cube_neighbor[i][1];
                        int z_neighbor = zz + cube_neighbor[i][2];
                        inside[i] = (this->GetNode(x_neighbor,y_neighbor,z_neighbor)->value<=(int)floor(isovalue));
                    }

                    for (int ai = 0; ai < 3; ++ai) {
                        auto &&e = far_edge[ai];
                        if (inside[e[0]] == inside[e[1]]) {
                            continue;  // Not a crossing
                        }

                        index_t v1, v2, v3;

                        if (ai == 0) {
                            v1 = this->MakeIndex(id + Eigen::Vector3i(0, 0, 1));
                            v2 = this->MakeIndex(id + Eigen::Vector3i(0, 1, 0));
                            v3 = this->MakeIndex(id + Eigen::Vector3i(0, 1, 1));
                        } else if (ai == 1) {
                            v1 = this->MakeIndex(id + Eigen::Vector3i(0, 0, 1));
                            v2 = this->MakeIndex(id + Eigen::Vector3i(1, 0, 0));
                            v3 = this->MakeIndex(id + Eigen::Vector3i(1, 0, 1));
                        } else {
                            v1 = this->MakeIndex(id + Eigen::Vector3i(0, 1, 0));
                            v2 = this->MakeIndex(id + Eigen::Vector3i(1, 0, 0));
                            v3 = this->MakeIndex(id + Eigen::Vector3i(1, 1, 0));
                        }

                        if (this->FindIndex(v1, vindex) && this->FindIndex(v2, vindex) &&
                            this->FindIndex(v3, vindex)) {
                            Eigen::Vector3i t0_old = Eigen::Vector3i(vindex[current_t], vindex[v3], vindex[v1]);
                            Eigen::Vector3i t1_old = Eigen::Vector3i(vindex[current_t], vindex[v2], vindex[v3]);
                            Eigen::Vector3i t0_old_all=Eigen::Vector3i(this->GetTupleIndex(current_t), this->GetTupleIndex(v1), this->GetTupleIndex(v3));
                            Eigen::Vector3i t1_old_all=Eigen::Vector3i(this->GetTupleIndex(current_t), this->GetTupleIndex(v3), this->GetTupleIndex(v2));

                            Eigen::Vector3i t0;
                            Eigen::Vector3i t1;
                            Eigen::Vector3i t0_all;
                            Eigen::Vector3i t1_all;
                            if (inside[e[0]] != (ai == 1))  // xor
                            {
                                t0=t0_old.reverse();
                                t1=t1_old.reverse();
                                t0_all=t0_old_all.reverse();
                                t1_all=t1_old_all.reverse();

                            }
                            else{
                                t0=t0_old;
                                t1=t1_old;
                                t0_all=t0_old_all;
                                t1_all=t1_old_all;
                            }

                            dc_faces.push_back(t0);
                            dc_faces.push_back(t1);

                        }
                    }
                }
            }
        }
    }


    vertices.resize(dc_vertices.size(), 3);
    for (size_t i = 0; i < dc_vertices.size(); ++i) {
        vertices.row(i) = dc_vertices[i];
    }
    faces.resize(dc_faces.size(), 3);

    for (size_t i = 0; i < dc_faces.size(); ++i) {
        faces.row(i) = dc_faces[i];
    }
    return true;


}

bool
DualContouring::GetHermiteData(double isovalue, FieldNode *node, std::vector<Eigen::Vector3d> &intersect_points,
                               std::vector<Eigen::Vector3d> &normals) {
    int x = node->x;
    int y = node->y;
    int z = node->z;

    std::vector<FieldNode *> cube_vertices;
    intersect_points.clear();

    for (size_t i = 0; i < 8; ++i) {
        int x_neighbor = x + cube_neighbor[i][0];
        int y_neighbor = y + cube_neighbor[i][1];
        int z_neighbor = z + cube_neighbor[i][2];

        cube_vertices.push_back(this->GetNode(x_neighbor, y_neighbor, z_neighbor));
    }


    for (size_t i = 0; i < 12; ++i) {
        if ((cube_vertices[neighbor_edges[i][0]]->value > isovalue &&
             cube_vertices[neighbor_edges[i][1]]->value > isovalue) ||
            (cube_vertices[neighbor_edges[i][0]]->value < isovalue &&
             cube_vertices[neighbor_edges[i][1]]->value < isovalue))
            continue;
        else {
            double alpha;
            Eigen::Vector3d intersect_point;
            if(cube_vertices[neighbor_edges[i][0]]->value>cube_vertices[neighbor_edges[i][1]]->value) {
                double iso_value_diff=isovalue-cube_vertices[neighbor_edges[i][1]]->value;

                alpha = iso_value_diff/(cube_vertices[neighbor_edges[i][0]]->value - cube_vertices[neighbor_edges[i][1]]->value);
                intersect_point =   alpha * cube_vertices[neighbor_edges[i][0]]->position +
                                    (1-alpha) * cube_vertices[neighbor_edges[i][1]]->position;
            }
            else{
                double iso_value_diff=isovalue-cube_vertices[neighbor_edges[i][0]]->value;

                alpha = iso_value_diff/(cube_vertices[neighbor_edges[i][1]]->value - cube_vertices[neighbor_edges[i][0]]->value);

                intersect_point = (1-alpha) * cube_vertices[neighbor_edges[i][0]]->position +
                                  alpha * cube_vertices[neighbor_edges[i][1]]->position;
            }


            intersect_points.push_back(intersect_point);
//            Eigen::Vector3d grad1, grad2;
//            this->GetGradient(grad1, cube_vertices[neighbor_edges[i][0]]);
//            this->GetGradient(grad2, cube_vertices[neighbor_edges[i][1]]);
//            Eigen::Vector3d normal = (1 - alpha) * grad1 + alpha * grad2;
//            normals.push_back(normal);

        }


    }



    if (intersect_points.size() >= 3)
        return true;
    else
        return false;

}

void DualContouring::GetGradient(Eigen::Vector3d &grad, FieldNode *node) {

    int x = node->x;
    int y = node->y;
    int z = node->z;
    int x_lower, x_upper, y_lower, y_upper, z_lower, z_upper;
    x_lower = x - 1;
    x_upper = x + 1;
    y_lower = y - 1;
    y_upper = y + 1;
    z_lower = z - 1;
    z_upper = z + 1;
    double x_gradient, y_gradient, z_gradient;
    if ((this->GetNode(x_lower, y, z)->value == -1 || x_lower < minx) &&
        (this->GetNode(x_upper, y, z)->value == -1 || x_upper > maxx))
        x_gradient = 0;
    if (this->GetNode(x_lower, y, z)->value == -1 || x_lower < 0)
        x_gradient = -10.0;
    else if (this->GetNode(x_upper, y, z)->value == -1 || x_upper > maxx)
        x_gradient = 10.0;
    else
        x_gradient = (double) (this->GetNode(x_upper, y, z)->value + this->GetNode(x_lower, y, z)->value -
                               2 * node->value) / 2.0;

    if ((this->GetNode(x, y_lower, z)->value == -1 || y_lower < miny) &&
        (this->GetNode(x, y_upper, z)->value == -1 || y_upper > maxy))
        y_gradient = 0;
    if (this->GetNode(x, y_lower, z)->value == -1 || y_lower < miny)
        y_gradient = -10.0;
    else if (this->GetNode(x, y_upper, z)->value == -1 || y_upper > maxy)
        y_gradient = 10.0;
    else
        y_gradient = (double) (this->GetNode(x, y_upper, z)->value + this->GetNode(x, y_lower, z)->value -
                               2 * node->value) / 2.0;


    if ((this->GetNode(x, y, z_lower)->value == -1 || z_lower < minz) &&
        (this->GetNode(x, y, z_upper)->value == -1 || z_upper > maxz))
        z_gradient = 0;
    if (this->GetNode(x, y, z_lower)->value == -1 || z_lower < minz)
        z_gradient = -10.0;
    else if (this->GetNode(x, y, z_upper)->value == -1 || z_upper > maxz)
        z_gradient = 10.0;
    else
        z_gradient = (double) (this->GetNode(x, y, z_upper)->value + this->GetNode(x, y, z_lower)->value -
                               2 * node->value) / 2.0;

    grad = Eigen::Vector3d(x_gradient, y_gradient, z_gradient);
    return;

}

void DualContouring::SolveQEF(std::vector<Eigen::Vector3d> &intersect_points, std::vector<Eigen::Vector3d> &normals,
                              Eigen::Vector3d &solution) {
    if (intersect_points.size() != normals.size()) {
        std::cout << "intersect points size is not equal to normals size" << std::endl;
        exit(-1);
    }
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    b.resize(normals.size());
    A.resize(normals.size(), 3);

    for (size_t i = 0; i < normals.size(); ++i)
        A.row(i) = normals[i];
    for (size_t i = 0; i < intersect_points.size(); ++i)
        b(i) = intersect_points[i].dot(normals[i]);


    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXd sol = svd.solve(b);

    if (sol.rows() != 3) {
        std::cout << "lstsq has some problem" << std::endl;
        exit(-1);
    }
    solution = Eigen::Vector3d(sol(0), sol(1), sol(2));
}

FieldNode *DualContouring::GetNode(int x, int y, int z) {
    return isovalue_set[x + res(0) * (z + res(2) * y)];
}

bool DualContouring::IsSafeNode(FieldNode *node) {
    int x = node->x;
    int y = node->y;
    int z = node->z;

    for (size_t i = 0; i < 8; ++i) {
        int x_neighbor = x + cube_neighbor[i][0];
        int y_neighbor = y + cube_neighbor[i][1];
        int z_neighbor = z + cube_neighbor[i][2];
        if (x_neighbor > maxx || y_neighbor > maxy || z_neighbor > maxz) return false;
        else if (x_neighbor < minx || y_neighbor <miny || z_neighbor <minz) return false;
        else if (this->GetNode(x_neighbor, y_neighbor, z_neighbor)->value == -1) return false;
    }
    return true;

}

void DualContouring::GetMassPoint(std::vector<Eigen::Vector3d> &intersect_points, Eigen::Vector3d &mass_point) {

    Eigen::Vector3d sum(0., 0., 0.);
    for (size_t i = 0; i < intersect_points.size(); ++i) {
        sum += intersect_points[i];
    }
    mass_point = sum / double(intersect_points.size());


}

bool DualContouring::FindIndex(std::tuple<int, int, int> &index, index_map &vindex) {
    auto it = vindex.find(index);
    if (it == vindex.end()) return false;
    else return true;

}


index_t DualContouring::MakeIndex(Eigen::Vector3i v) {
    return std::make_tuple(v(0), v(1), v(2));
}

int DualContouring::GetIndex(int x,int y,int z){
    return x + res(0) * (z + res(2) * y);
}

int DualContouring::GetTupleIndex(index_t &tuple){
    int x=std::get<0>(tuple);
    int y=std::get<1>(tuple);
    int z=std::get<2>(tuple);
    return x + res(0) * (z + res(2) * y);


}

