#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <Eigen/Core>
#include <igl/unproject.h>
#include <igl/unproject_ray.h>
#include <iostream>
#include "ImGuizmo.h"
#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

igl::opengl::glfw::Viewer viewer;
Eigen::MatrixXd V;

int dimension = 5;
int split = 4;
int selected_point = -1;

Eigen::MatrixXd create_points(int split = 1)
{
    int size = dimension * split - (split-1);
    Eigen::MatrixXd V_(size*size, 3);
    float ratio = static_cast<float>(dimension) / static_cast<float>(size);
    if (size != dimension)
        ratio = static_cast<float>(dimension) / static_cast<float>(dimension*split);

    for (int i = 0; i < size; ++i)
        for (int j = 0; j < size; ++j)
            V_.row(i * size + j) << j*ratio, i*ratio, 0;
    return V_;
}

void show_selected_point()
{
    static int selected_point_layer = viewer.append_mesh();

    viewer.data(selected_point_layer).clear();
    if (selected_point != -1) {
        viewer.data(selected_point_layer).set_points(V.row(selected_point), Eigen::RowVector3d(1, 1, 0));
        viewer.data(selected_point_layer).point_size = 10;
    }
}

void show_points()
{
    static int points_layer = viewer.append_mesh();

    viewer.data(points_layer).clear();
    viewer.data(points_layer).set_points(V, Eigen::RowVector3d(1, 0, 0));
    viewer.data(points_layer).point_size = 6;
}

Eigen::MatrixXi generate_surface(int size = dimension)
{
    Eigen::MatrixXi F(2 * (size - 1) * (size - 1), 3);

    for (int i = 0; i < size - 1; ++i) {
        for (int j = 0; j < size - 1; ++j) {
            int idx = i * (size - 1) + j;
            F.row(2 * idx) << i * size + j, i * size + j + 1, (i + 1) * size + j + 1;
            F.row(2 * idx + 1) << i * size + j, (i + 1) * size + j + 1, (i + 1) * size + j;
        }
    }
    return F;
}

void show_surface()
{
    static int surface_layer = viewer.append_mesh();
    Eigen::MatrixXd V_spline = create_points(split);
    Eigen::MatrixXi F_spline = generate_surface(dimension * split - (split - 1));

    viewer.data(surface_layer).clear();
    viewer.data(surface_layer).set_mesh(V_spline, F_spline);
    viewer.data(surface_layer).show_lines = true;
    viewer.data(surface_layer).show_faces = true;
    viewer.data(surface_layer).set_colors(Eigen::RowVector3d(0.0, 0.8, 0.0));
}

int find_corresponding_index(int index) {
    int size_small = dimension;
    int size_large = dimension * split - (split - 1);

    // Récupérer les coordonnées (row, col) dans la grille small
    int row_small = index / size_small;
    int col_small = index % size_small;

    // Appliquer le facteur d'échelle pour obtenir (row, col) dans la grille large
    int row_large = row_small * (size_large - 1) / (size_small - 1);
    int col_large = col_small * (size_large - 1) / (size_small - 1);

    // Convertir en index linéarisé dans la grille large
    return row_large * size_large + col_large;
}

bool key_down_callback(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier)
{
    if (selected_point == -1)
        return false;

    double step = 0.5;

    switch (key) {
        case 'W': // Up
        case 'w':
            V(selected_point, 1) += step;
            break;
        case 'S': // Down
        case 's':
            V(selected_point, 1) -= step;
            break;
        case 'A': // Left
        case 'a':
            V(selected_point, 0) -= step;
            break;
        case 'D': // Right
        case 'd':
            V(selected_point, 0) += step;
            break;
        case 'Q': // Forward (Z-axis)
        case 'q':
            V(selected_point, 2) += step;
            break;
        case 'E': // Backward (Z-axis)
        case 'e':
            V(selected_point, 2) -= step;
            break;
        default:
            return false;
    }

    show_selected_point();
    show_points();
    show_surface();

    return true; // Key handled
}

bool mouse_down_callback(igl::opengl::glfw::Viewer &viewer, int button, int modifier)
{
    if (button != 0 || !(modifier & GLFW_MOD_ALT))
        return false;

    Eigen::Vector3f pos, dir;
    igl::unproject_ray(
        Eigen::Vector2f(viewer.current_mouse_x, viewer.core().viewport[3] - viewer.current_mouse_y),
        viewer.core().view,
        viewer.core().proj,
        viewer.core().viewport,
        pos,
        dir);

    float min_distance = std::numeric_limits<float>::max();

    for (int i = 0; i < V.rows(); ++i) {
        Eigen::Vector3f point = V.row(i).cast<float>();
        Eigen::Vector3f diff = point - pos;
        Eigen::Vector3f cross_product = diff.cross(dir);
        float distance = cross_product.norm() / dir.norm();

        if (distance < min_distance) {
            min_distance = distance;
            selected_point = i;
        }
    }

    if (selected_point != -1) {
        std::cout << "Point index on controle grid: " << selected_point << std::endl;
        std::cout << "Point index on surface: " << find_corresponding_index(selected_point) << std::endl;

        show_selected_point();
        show_points();
        show_surface();
    }

    return true;
}

int main(int argc, char *argv[])
{
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    viewer.launch_init();

    ImGui_ImplGlfw_InitForOpenGL(viewer.window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    V = create_points();

    show_points();
    show_surface();

    viewer.callback_mouse_down = &mouse_down_callback;
    viewer.callback_key_down = &key_down_callback; // Register keyboard callback

    while (!glfwWindowShouldClose(viewer.window)) {
        viewer.draw();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwPollEvents();
        glfwSwapBuffers(viewer.window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
