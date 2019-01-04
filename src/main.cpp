#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>

#include "bsp_tree.h"
#include "voxel.h"
#include "dual_contouring.h"
#include "trimming.h"
#include "file_parser.h"
#include "utils.h"

unsigned max_layer;
unsigned max_peeling_layer;

unsigned current_layer = 1;
unsigned int from_index = 0;
unsigned int to_index = 1;

VoxelGrid *voxel_set;
bool is_growed = false;
Eigen::MatrixXd V;
Eigen::MatrixXi F;

Eigen::MatrixXd V_Platform;
Eigen::MatrixXi F_Platform;
Eigen::MatrixXd C_Platform;

Eigen::MatrixXd V_All;
Eigen::MatrixXi F_All;
Eigen::MatrixXd C_All;

Eigen::MatrixXd voxels_v;
Eigen::MatrixXi voxels_f;
Eigen::MatrixXd voxels_c;

Eigen::MatrixXd convex_v;
Eigen::MatrixXi convex_f;

std::string platform_path = "../platform.obj";


int layer;

bool pre_draw(igl::opengl::glfw::Viewer &viewer) {

    viewer.data_list[0].set_mesh(V_Platform, F_Platform);
    viewer.data_list[0].set_colors(Eigen::RowVector3d(211. / 255., 211. / 255., 211. / 255.));
    viewer.core.align_camera_center(V_Platform, F_Platform);
    return false;

}

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) {
    if (key == '-') {
        if (current_layer >= 1) {
            viewer.data_list[1].clear();

            current_layer -= 1;
            std::cout << "current layer " << current_layer << std::endl;

            Eigen::MatrixXd voxels_mat;
            Eigen::MatrixXd voxels_with_platform;
            std::vector<voxel *> voxels;
            std::vector<voxel *> previous_voxels;
            if (!is_growed) {
                voxel_set->getPeeling(current_layer, voxels);
                voxel_set->getPeeling(current_layer - 1, previous_voxels);
            }
            if (is_growed) {
                voxel_set->getGrowing(current_layer, voxels);
                voxel_set->getGrowing(current_layer - 1, previous_voxels);
            }

            voxel_set->voxelMesh(voxels, voxels_v, voxels_f, voxels_c, is_growed, false);

            if (previous_voxels.size()) {
                voxel_set->getVoxelsPos(previous_voxels, voxels_mat);
                append_matrix(voxels_mat, V_Platform, voxels_with_platform);

                voxel_set->generateConvexHull(voxels_with_platform, convex_v, convex_f);
                for (size_t i = 0; i < convex_f.rows(); ++i) {
                    viewer.data().add_edges
                            (
                                    convex_v.row(convex_f(i, 0)),
                                    convex_v.row(convex_f(i, 1)),
                                    Eigen::RowVector3d(0., 0., 0.)
                            );
                    viewer.data().add_edges
                            (
                                    convex_v.row(convex_f(i, 1)),
                                    convex_v.row(convex_f(i, 2)),
                                    Eigen::RowVector3d(0., 0., 0.)
                            );
                    viewer.data().add_edges
                            (
                                    convex_v.row(convex_f(i, 2)),
                                    convex_v.row(convex_f(i, 0)),
                                    Eigen::RowVector3d(0., 0., 0.)
                            );
                }
            }


            viewer.data_list[1].set_mesh(voxels_v, voxels_f);
            viewer.data_list[1].set_colors(voxels_c);
            voxels.clear();

        }

    } else if (key == '=') {
        if (current_layer < max_layer) {
            viewer.data_list[1].clear();

            Eigen::MatrixXd voxels_mat;
            Eigen::MatrixXd voxels_with_platform;
            std::vector<voxel *> voxels;
            std::vector<voxel *> previous_voxels;

            current_layer += 1;
            std::cout << "current layer " << current_layer << std::endl;
            if (!is_growed) {
                voxel_set->getPeeling(current_layer, voxels);
                voxel_set->getPeeling(current_layer - 1, previous_voxels);
            }
            if (is_growed) {
                voxel_set->getGrowing(current_layer, voxels);
                voxel_set->getGrowing(current_layer - 1, previous_voxels);
            }

            voxel_set->voxelMesh(voxels, voxels_v, voxels_f, voxels_c, is_growed, false);

            if (previous_voxels.size()) {
                voxel_set->getVoxelsPos(previous_voxels, voxels_mat);
                append_matrix(voxels_mat, V_Platform, voxels_with_platform);

                voxel_set->generateConvexHull(voxels_with_platform, convex_v, convex_f);
                for (size_t i = 0; i < convex_f.rows(); ++i) {
                    viewer.data().add_edges
                            (
                                    convex_v.row(convex_f(i, 0)),
                                    convex_v.row(convex_f(i, 1)),
                                    Eigen::RowVector3d(0., 0., 0.)
                            );
                    viewer.data().add_edges
                            (
                                    convex_v.row(convex_f(i, 1)),
                                    convex_v.row(convex_f(i, 2)),
                                    Eigen::RowVector3d(0., 0., 0.)
                            );
                    viewer.data().add_edges
                            (
                                    convex_v.row(convex_f(i, 2)),
                                    convex_v.row(convex_f(i, 0)),
                                    Eigen::RowVector3d(0., 0., 0.)
                            );
                }
            }


            viewer.data_list[1].set_mesh(voxels_v, voxels_f);
            viewer.data_list[1].set_colors(voxels_c);
            voxels.clear();
        }


    }

    return false;
}

int main(int argc, char *argv[]) {

    igl::opengl::glfw::Viewer viewer;

    igl::opengl::glfw::imgui::ImGuiMenu menu;

    viewer.plugins.push_back(&menu);

    igl::read_triangle_mesh(platform_path, V_Platform,
                            F_Platform);

    menu.callback_draw_viewer_menu = [&]() {

        if (ImGui::CollapsingHeader("Model", ImGuiTreeNodeFlags_DefaultOpen)) {


            ImGui::Columns(2, nullptr, false);
            if (ImGui::Button("Load", ImVec2(-1, 0))) {
                std::string filename;
                filename = igl::file_dialog_open();

                size_t last_dot = filename.rfind('.');
                if (last_dot == std::string::npos) {

                    std::cerr << "Error: No file extension found in " <<
                              filename << std::endl;
                    return false;
                }
                std::string extension = filename.substr(last_dot + 1);
                if (extension != "off" && extension != "OFF"
                    && extension != "obj" && extension != "OBJ"
                    && extension != "stl" && extension != "STL"
                    && extension != "ply" && extension != "PLY") {
                    std::cerr << "Error: filename " << filename << " is not a mesh file" << std::endl;
                    return false;
                } else {
                    igl::read_triangle_mesh(filename, V, F);
                    viewer.append_mesh();
                    viewer.data_list[1].set_mesh(V, F);
                    viewer.data_list[1].set_colors(Eigen::RowVector3d(211. / 255., 211. / 255., 211. / 255.));

                }
            }

            ImGui::NextColumn();

            if (ImGui::Button("Clear", ImVec2(-1, 0))) {
                viewer.data_list[1].clear();;
            }

            ImGui::Columns(1);

            if (ImGui::Button("Center to Platform", ImVec2(-1, 0))) {
                center_to_platform(V);
                viewer.data_list[1].clear();
                viewer.data_list[1].set_mesh(V, F);
                viewer.data_list[1].set_colors(Eigen::RowVector3d(211. / 255., 211. / 255., 211. / 255.));
            }
        }

        if (ImGui::CollapsingHeader("Accumulation Field ", ImGuiTreeNodeFlags_DefaultOpen)) {
            static float voxel_size = 1.0;
            ImGui::InputFloat("Voxel Size", &voxel_size, 0.1f, 0, 2);

            enum Method {
                is_greedy = 0, is_peeling
            };
            static Method method = is_greedy;
            ImGui::Combo("Method", (int *) (&method), "SP_GCFA\0SP_PG_GCFA\0\0");

            if (ImGui::Button("Generate", ImVec2(-1, 0))) {
                if (!V.rows()) {
                    std::cout << "No mesh found, please import first" << std::endl;
                } else {

                    BSPTree *bsp_tree = new BSPTree();

                    MyMesh *mesh = new MyMesh(V, F);
                    bsp_tree->BSPTreeFromMesh(mesh);
                    std::cout << "---------------------------------" << std::endl;
                    std::cout << "BSP Tree Generated..." << std::endl;
                    std::cout << "---------------------------------" << std::endl;
                    voxel_set = new VoxelGrid(V, voxel_size, 5);
                    voxel_set->Voxelization(bsp_tree);
                    std::cout << std::endl;
                    std::cout << "Enlarging" << std::endl;
                    voxel_set->Enlarge();

                    voxel_set->loadPlatform(platform_path);
                    std::vector<voxel *> missing_voxels;

                    if (method) {

                        max_peeling_layer = voxel_set->generatePeelingOrder();
                        std::cout << std::endl;
                        std::cout << "Peeling order generated" << std::endl;

                        max_layer = voxel_set->generateGrowingOrder(missing_voxels);
                        std::cout << "Missing " << missing_voxels.size() << " voxels" << std::endl;

                        std::cout << std::endl;
                        std::cout << "Field generation completed" << std::endl;

                        current_layer = max_layer;

                        is_growed = true;

                    } else {

                        max_layer = voxel_set->generateGreedyGrowingOrder(missing_voxels);
                        std::cout << "Missing " << missing_voxels.size() << " voxels" << std::endl;

                        std::cout << std::endl;
                        std::cout << "Field generation completed" << std::endl;

                        current_layer = max_layer;
                        is_growed = true;
                    }

                    std::vector<voxel *> voxels;
                    voxel_set->getGrowing(current_layer, voxels);
                    voxel_set->voxelMesh(voxels, voxels_v, voxels_f, voxels_c, is_growed, true);
                    viewer.data_list[1].clear();
                    viewer.data_list[1].set_mesh(voxels_v, voxels_f);
                    viewer.data_list[1].set_colors(voxels_c);
                }
            }
        }

        if (ImGui::CollapsingHeader("Surface Layer ", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Contouring", ImVec2(-1, 0))) {
                DualContouring *dc;
                dc = new DualContouring();
                dc->LoadIsovalueSet(voxel_set);
                int max_layer = dc->max_layer;
                Eigen::MatrixXd reorient_V;
                Eigen::MatrixXi reorient_F;
                reorient_mesh(V,F,reorient_V,reorient_F);

                int layer_count = 0;

                for (int i = 1; i < max_layer - 1; ++i) {

                    double isovalue = i + 0.001;

                    Eigen::MatrixXd VDC;
                    Eigen::MatrixXi FDC;
                    bool success;
                    success = dc->DoContouring(isovalue, VDC, FDC);
                    if (!success || FDC.rows() < 2)
                        continue;
                    else {
                        std::string filename =
                                "../layers/" + std::to_string(layer_count) + ".obj";
                        Eigen::MatrixXd VT;
                        Eigen::MatrixXi FT;

                        trimming(VDC, FDC, reorient_V, reorient_F, VT, FT);
                        if (FT.rows() < 2)
                            continue;
                        igl::write_triangle_mesh(filename, VT, FT);
                        std::cout << "layer " << i << " generated" << std::endl;
                        layer_count++;
                    }
                }
                std::cout << "Surface generation completed" << std::endl;

            }

        }
    };


    viewer.data().show_lines = true;
    viewer.data().face_based = true;
    viewer.callback_pre_draw = &pre_draw;
    viewer.core.background_color = Eigen::Vector4f(50. / 255., 50. / 255., 50. / 255., 1.0);
    viewer.callback_key_down = &key_down;

    viewer.launch();
}

