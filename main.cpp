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
#include <Eigen/Dense>
#include <vector>
#include <igl/writeOBJ.h>

igl::opengl::glfw::Viewer viewer;
Eigen::MatrixXd V_surface;
Eigen::MatrixXd V;

int dimension = 10;
int split = 5;
int selected_point = -1;

Eigen::MatrixXd create_points(int split = 1, float z = 0.3)
{
    int size = dimension * split - (split-1);
    Eigen::MatrixXd V_(size*size, 3);
    float ratio = static_cast<float>(dimension) / static_cast<float>(size);
    if (size != dimension)
        ratio = static_cast<float>(dimension) / static_cast<float>(dimension*split);

    for (int i = 0; i < size; ++i)
        for (int j = 0; j < size; ++j)
            V_.row(i * size + j) << j*ratio, i*ratio, z;
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

int find_corresponding_index(int index) {
    int size_small = dimension;
    int size_large = dimension * split - (split - 1);

    int row_small = index / size_small;
    int col_small = index % size_small;

    int row_large = row_small * (size_large - 1) / (size_small - 1);
    int col_large = col_small * (size_large - 1) / (size_small - 1);

    return row_large * size_large + col_large;
}

double N(int i, int k, double u, const Eigen::VectorXd &knots)
{
    if (k == 1)
        return (knots(i) <= u && u <= knots(i + 1)) ? 1.0 : 0.0;  // Inclut u == knots(i+1)

    double coef1 = 0.0, coef2 = 0.0;

    if (knots(i + k - 1) != knots(i)) // Évite division par 0
        coef1 = (u - knots(i)) / (knots(i + k - 1) - knots(i)) * N(i, k - 1, u, knots);

    if (knots(i + k) != knots(i + 1)) // Évite division par 0
        coef2 = (knots(i + k) - u) / (knots(i + k) - knots(i + 1)) * N(i + 1, k - 1, u, knots);

    return coef1 + coef2;
}

Eigen::VectorXd generate_knots(int num_ctrl_pts, int degree)
{
    int num_knots = num_ctrl_pts + degree + 1;
    Eigen::VectorXd knots(num_knots);

    // Première et dernière valeur répétée `degree + 1` fois
    for (int i = 0; i <= degree; ++i) {
        knots(i) = 0;
        knots(num_knots - 1 - i) = num_ctrl_pts - degree;
    }

    // Remplissage uniforme des nœuds internes
    for (int i = 1; i < num_ctrl_pts - degree; ++i) {
        knots(i + degree) = i;
    }

    return knots;
}
Eigen::MatrixXd compute_b_spline_surface()
{
    int degree = 3; // Degré cubique
    int size = dimension * split - (split - 1);

    // Générer les nœuds correctement
    Eigen::VectorXd knots_u = generate_knots(dimension, degree);
    Eigen::VectorXd knots_v = generate_knots(dimension, degree);

    Eigen::MatrixXd surface(size * size, 3);

    for (int ui = 0; ui < size; ++ui) {
        for (int vi = 0; vi < size; ++vi) {
            double u = std::min(static_cast<double>(ui) / (size - 1) * (dimension - degree), static_cast<double>(dimension - degree));
            double v = std::min(static_cast<double>(vi) / (size - 1) * (dimension - degree), static_cast<double>(dimension - degree));

            Eigen::Vector3d point(0,0,0);

            for (int i = 0; i < dimension; ++i) {
                for (int j = 0; j < dimension; ++j) {
                    double Bu = N(i, degree + 1, u, knots_u);
                    double Bv = N(j, degree + 1, v, knots_v);
                    point += Bu * Bv * V.row(i * dimension + j);
                }
            }

            surface.row(ui * size + vi) = point;
        }
    }

    return surface;
}

Eigen::MatrixXi generate_surface(int size = dimension)
{
    Eigen::MatrixXi F(2 * (size - 1) * (size - 1), 3);
    int f = 0;

    for (int ui = 0; ui < size - 1; ++ui) {
        for (int vi = 0; vi < size - 1; ++vi) {
            int idx = ui * size + vi;

            F.row(f++) << idx, idx + 1, idx + size;
            F.row(f++) << idx + 1, idx + size + 1, idx + size;
        }
    }

    return F;
}


void show_surface()
{
    static int surface_layer = viewer.append_mesh();
    V_surface = compute_b_spline_surface();
    static Eigen::MatrixXi F_spline = generate_surface(dimension * split - (split - 1));

    viewer.data(surface_layer).clear();
    viewer.data(surface_layer).set_mesh(V_surface, F_spline);
    viewer.data(surface_layer).show_lines = true;
    viewer.data(surface_layer).show_faces = true;
    viewer.data(surface_layer).set_colors(Eigen::RowVector3d(0.0, 0.8, 0.0));
}

bool key_down_callback(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier)
{
    if (selected_point == -1)
        return false;

    double step = 0.2;

    switch (key) {
        case 'W': V(selected_point, 1) += step; V_surface(find_corresponding_index(selected_point), 1) += step; break;
        case 'S': V(selected_point, 1) -= step; V_surface(find_corresponding_index(selected_point), 1) -= step; break;
        case 'A': V(selected_point, 0) -= step; V_surface(find_corresponding_index(selected_point), 0) -= step; break;
        case 'D': V(selected_point, 0) += step; V_surface(find_corresponding_index(selected_point), 0) += step; break;
        case 'Q': V(selected_point, 2) += step; V_surface(find_corresponding_index(selected_point), 2) += step; break;
        case 'E': V(selected_point, 2) -= step; V_surface(find_corresponding_index(selected_point), 2) -= step; break;
        default: return false;
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
    V_surface = create_points(split, 0);

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
