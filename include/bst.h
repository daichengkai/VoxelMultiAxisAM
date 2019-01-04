//
// Created by daichengkai on 16-10-17.
//

#ifndef VOXELFAB_BST_H
#define VOXELFAB_BST_H

#include <vector>
#include <algorithm>
#include <iostream>

class TreeNode {
public:
    TreeNode(void) {
        leftChild = rightChild = NULL;
    };

    virtual ~TreeNode(void) {
        if (leftChild != NULL) delete leftChild;
        if (rightChild != NULL) delete rightChild;
    };

    int index;
    TreeNode *leftChild, *rightChild;
    TreeNode *opposite;

    TreeNode *parent;
    std::vector<int> sets;

    bool IsLeaveNode() { return (leftChild == NULL) && (rightChild == NULL); }
    bool IsRootNode() { return parent==NULL; }


};

struct voxel;
class VoxelGrid;

class BSTree {
public:
    BSTree(void) {
        tree_root = new TreeNode;
    };

    virtual ~BSTree(void) { delete tree_root;};

    void BSTreeFromVoxelGrid(VoxelGrid *voxel_set,std::vector<voxel*> &all_voxels,std::vector<voxel*> &current_front,std::vector<voxel*> &solid_voxels);

    void ConstructBSNode(TreeNode *tree_node,TreeNode *parent);

    TreeNode* LookBack(TreeNode* node);

    bool getSafeNodes(std::vector<int> &indice);


private:


    TreeNode *tree_root;

    VoxelGrid *voxel_set_;

    std::vector<voxel*> voxels_;

    std::vector<voxel*> current_front_;

    std::vector<voxel*> solid_voxels_;

    std::vector<int> safe_indices_;

    std::vector<bool> visited_flag;

    int level=0;

    int count=0;

    void removeIndex(TreeNode* node,std::vector<int> remove_index);

    void Traverse(TreeNode *node);

    bool check(std::vector<int> &indices);

};



#endif //VOXELFAB_BST_H
