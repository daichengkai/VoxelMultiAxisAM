//
// Created by daichengkai on 3-5-17.
//

#ifndef VOXELFAB_DUAL_CONTOURING_H
#define VOXELFAB_DUAL_CONTOURING_H

#include <string>
#include <Eigen/SVD>
#include "hash_tuple.h"
#include <unordered_map>
#include "kdtree.h"
#include "voxel.h"

typedef struct FieldNode {
    Eigen::Vector3d position;
    int x;
    int y;
    int z;
    int value;
    bool occupied=false;
    bool processed=false;
};

typedef std::tuple<int, int,int> index_t;
typedef std::unordered_map<std::tuple<int, int,int>, size_t, hash_tuple::hash<std::tuple<int, int,int>>> index_map;

class DualContouring {
public:
    DualContouring() {};

    ~DualContouring() {
        for (size_t i = 0; i < isovalue_set.size(); ++i)
            delete (isovalue_set[i]);
    };

public:
    void LoadIsovalueSet(VoxelGrid* voxel_set);

    bool DoContouring(double isovalue,Eigen::MatrixXd &vertices, Eigen::MatrixXi &faces);

private:

    void GetRelatedSet(double isovalue,int &x_min,int &x_max, int &y_min, int &y_max, int &z_min, int &z_max);

    bool GetRelatedSet(double isovalue, std::vector<Eigen::Vector3i> &related_indices);

    bool GetHermiteData(double isovalue, FieldNode *node, std::vector<Eigen::Vector3d> &intersect_points,
                        std::vector<Eigen::Vector3d> &normals);

    void GetMassPoint(std::vector<Eigen::Vector3d> &intersect_points,Eigen::Vector3d &mass_point);

    FieldNode* GetNode(int x,int y,int z);

    bool IsSafeNode(FieldNode* node);

    int GetIndex(int x,int y,int z);

    int GetTupleIndex(index_t &tuple);

    void GetGradient(Eigen::Vector3d &grad, FieldNode *node);

    void SolveQEF(std::vector<Eigen::Vector3d> &intersect_points, std::vector<Eigen::Vector3d> &normals,
                  Eigen::Vector3d &solution);

    bool FindIndex(std::tuple<int, int,int> &index, index_map &vindex);

    index_t MakeIndex(Eigen::Vector3i v);

    void enlargeRelatedSet(std::vector<Eigen::Vector3i> &related_indices);


public:
    int max_layer;

private:
    std::vector<FieldNode *> isovalue_set;
    Eigen::Vector3i res;
    kdtree *kd_tree;
    std::vector<bool> bGrid;
    size_t triangle_count;
    index_map tindex;
    int maxx;int minx;
    int maxy;int miny;
    int maxz;int minz;



};

const int cube_neighbor[8][3] = {{0, 0, 0},
                                 {0, 0, 1},
                                 {0, 1, 0},
                                 {0, 1, 1},
                                 {1, 0, 0},
                                 {1, 0, 1},
                                 {1, 1, 0},
                                 {1, 1, 1}};

const int neighbor_edges[12][2] = {{0, 1},
                                   {0, 2},
                                   {0, 4},
                                   {1, 3},
                                   {1, 5},
                                   {2, 3},
                                   {2, 6},
                                   {3, 7},
                                   {4, 5},
                                   {4, 6},
                                   {5, 7},
                                   {6, 7}};

const int neighbors[6][3] = {{0,  0,  1},
                             {0,  0,  -1},
                             {0,  1,  0},
                             {0,  -1, 0},
                             {1,  0,  0},
                             {-1, 0,  1}};

const int cube_neighbor_vertex[26][3] = {{-1, -1, -1},
                                         {-1, -1, 0},
                                         {-1, -1, 1},
                                         {0, -1, -1},
                                         {0, -1, 0},
                                         {0, -1, 1},
                                         {1, -1, -1},
                                         {1, -1, 0},
                                         {1, -1, 1},

                                         {-1, 0, -1},
                                         {-1, 0, 0},
                                         {-1, 0, 1},
                                         {0, 0, -1},
//                                         {0, 0, 0},
                                         {0, 0, 1},
                                         {1, 0, -1},
                                         {1, 0, 0},
                                         {1, 0, 1},

                                         {-1, 1, -1},
                                         {-1, 1, 0},
                                         {-1, 1, 1},
                                         {0, 1, -1},
                                         {0, 1, 0},
                                         {0, 1, 1},
                                         {1, 1, -1},
                                         {1, 1, 0},
                                         {1, 1, 1}
};
//const int cube_neighbor_vertex[18][3] = {{-1, 0,  0},
//                                   {1,  0,  0},
//                                   {0,  1,  0},
//                                   {0,  -1, 0},
//                                   {0,  0,  -1},
//                                   {0,  0,  1},
//                                   {-1, 1,  0},
//                                   {-1, -1, 0},
//                                   {1,  -1, 0},
//                                   {1,  1,  0},
//                                   {-1, 0,  1},
//                                   {-1, 0,  -1},
//                                   {1,  0,  -1},
//                                   {1,  0,  1},
//                                   {0,  -1, 1},
//                                   {0,  -1, -1},
//                                   {0,  1,  -1},
//                                   {0,  1,  1}};

#endif //VOXELFAB_DUAL_CONTOURING_H
