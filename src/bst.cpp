//
// Created by daichengkai on 16-10-17.
//

#include "bst.h"
#include "voxel.h"


void
BSTree::BSTreeFromVoxelGrid(VoxelGrid *voxel_set, std::vector<voxel *> &all_voxels, std::vector<voxel *> &current_front,
                            std::vector<voxel *> &solid_voxels) {

    std::vector<voxel *> voxels;

    std::vector<int> sets;

    for (int i = 0; i < all_voxels.size(); ++i)
        sets.push_back(i);

    voxel_set_ = voxel_set;

    tree_root->sets = sets;

    tree_root->parent = NULL;

    voxels_ = all_voxels;

    current_front_ = current_front;

    solid_voxels_ = solid_voxels;
    level=0;
    
    count=0;
    std::vector<int> empty_indices;
    visited_flag=std::vector<bool>(solid_voxels.size(),false);
    Traverse(tree_root);

    std::cout<<count<<" "<<sets.size()<<std::endl;

}

void BSTree::ConstructBSNode(TreeNode *tree_node, TreeNode *parent) {

    tree_node->parent = parent;


    if (tree_node->sets.size() < 2)
        return;


    std::vector<int> indices = tree_node->sets;

    std::vector<int> left;
    std::vector<int> right;

    voxel_set_->seperateVoxelsHalf(voxels_, indices, left, right);

    tree_node->leftChild = new TreeNode;

    tree_node->leftChild->sets = left;

    this->ConstructBSNode(tree_node->leftChild, tree_node);

    tree_node->rightChild = new TreeNode;

    tree_node->rightChild->sets = right;

    this->ConstructBSNode(tree_node->rightChild, tree_node);

}


void BSTree::Traverse(TreeNode *node) {


    if (!node)
        return;

    std::vector<int> indices;
////
//    std::cout<<"safe_indices"<<std::endl;
//    for(int i=0;i<safe_indices_.size();++i){
//        std::cout<<safe_indices_[i]<<" ";
//    }
//    std::cout<<std::endl;
//
//    std::cout<<"node_set"<<std::endl;
//    for(int i=0;i<node->sets.size();++i){
//        std::cout<<node->sets[i]<<" ";
//    }
//    std::cout<<std::endl;

    for (int i = 0; i < safe_indices_.size(); ++i) {
        indices.push_back(safe_indices_[i]);
    }
    for (int i = 0; i < node->sets.size(); ++i) {
        indices.push_back(node->sets[i]);
    }

    bool checked = this->check(indices);

    if(node->sets.size()<2){
        count++;
    }

    if (!checked) {
        for (int i = 0; i < node->sets.size(); ++i) {
            safe_indices_.push_back(node->sets[i]);
        }

        for(int i=0;i<node->sets.size();++i){
            visited_flag[node->sets[i]]=true;
        }

//        std::cout<<"bbb"<<std::endl;
//
//        for(int i=0;i<safe_indices_.size();++i){
//            std::cout<<safe_indices_[i]<<" ";
//        }
//        std::cout<<std::endl;
//        std::cout<<"ccc"<<std::endl;

        return;
    } else {


        node->leftChild = new TreeNode;
        node->rightChild = new TreeNode;

        if(node->sets.size()<2){
            node->leftChild=NULL;
            node->rightChild=NULL;
        }else{
            std::vector<int> left_indices;
            std::vector<int> right_indices;
            voxel_set_->seperateVoxelsHalf(voxels_, node->sets, left_indices, right_indices);
//            while(!voxel_set_->seperateVoxelsHalf(voxels_, node->sets, left_indices, right_indices,level))
//                level+=1;


//            voxel_set_->seperateVoxelsHalf(voxels_, node->sets, left_indices, right_indices);

            node->leftChild->sets=left_indices;
            node->rightChild->sets=right_indices;

        }



//        std::cout<<"goes left"<<std::endl;
        this->Traverse(node->leftChild);
//        std::cout<<"goes right"<<std::endl;

        this->Traverse(node->rightChild);
    }

}


bool BSTree::check(std::vector<int> &indices) {
//    std::cout<<"start checking"<<std::endl;
//    for(int i=0;i<indices.size();++i){
//        std::cout<<indices[i]<<" ";
//    }
//    std::cout<<std::endl;


    std::vector<voxel *> voxels_to_check;
    for (int i = 0; i < indices.size(); ++i) {
        voxels_to_check.push_back(voxels_[indices[i]]);
    }

    Eigen::MatrixXd test_v;
    voxel_set_->getVoxelsPos(voxels_to_check, test_v);

    std::vector<voxel *> unprocessed;
    for(int i=0;i<visited_flag.size();++i){
        if(!visited_flag[i])
            unprocessed.push_back(solid_voxels_[i]);
    }

    std::vector<voxel *> shadows;
    unsigned count;
//    count = voxel_set_->checkSetShadowed(current_front_, voxels_to_check, solid_voxels_, shadows);
////    std::cout<<"count "<<count<<std::endl;
    count = voxel_set_->checkSetShadowed(current_front_, voxels_to_check, unprocessed, shadows);

    if (count)
        return true;
    else
        return false;

}

void BSTree::removeIndex(TreeNode *node, std::vector<int> remove_index) {
    if (!node)
        return;
    else {

        for (int i = 0; i < remove_index.size(); ++i)
            node->sets.erase(std::remove(node->sets.begin(), node->sets.end(), remove_index[i]), node->sets.end());

        this->removeIndex(node->parent, remove_index);
    }
}

TreeNode *BSTree::LookBack(TreeNode *node) {
    if (!node->parent)
        return NULL;
    if (node->parent->sets.size())
        return node->parent;
    else
        LookBack(node->parent);

}

bool BSTree::getSafeNodes(std::vector<int> &indice) {
    if (!safe_indices_.size())
        return false;
    else {
        indice = safe_indices_;
        return true;
    }
}
