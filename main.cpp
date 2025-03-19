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
Eigen::MatrixXi F;

int dimension = 8;
int selected_point = -1;

Eigen::MatrixXd create_points()
{
    Eigen::MatrixXd V(dimension*dimension, 3);

    for (int i = 0; i < dimension; ++i)
        for (int j = 0; j < dimension; ++j)
            V.row(i * dimension + j) << j, i, 0;
    return V;
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

void show_surface()
{
    static int surface_layer = viewer.append_mesh();

    viewer.data(surface_layer).clear();
    viewer.data(surface_layer).set_mesh(V, F);
    viewer.data(surface_layer).show_lines = true;
    viewer.data(surface_layer).show_faces = true;
    viewer.data(surface_layer).set_colors(Eigen::RowVector3d(0.0, 0.8, 0.0));
}

double N(int i, int k, double u, const Eigen::VectorXd &knots)
{
    if (k == 1)
        return (knots(i) <= u && u < knots(i + 1)) ? 1.0 : 0.0;

    double coef1 = 0.0, coef2 = 0.0;

    if (knots(i + k - 1) != knots(i)) // Vérifie la division par 0
        coef1 = (u - knots(i)) / (knots(i + k - 1) - knots(i)) * N(i, k - 1, u, knots);

    if (knots(i + k) != knots(i + 1)) // Vérifie la division par 0
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
        knots(i + degree) = static_cast<double>(i);
    }

    return knots;
}

double b_spline_basis(int i, int k, double t, const Eigen::VectorXd& knots) {
    if (k == 0)
        return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;

    double a = (t - knots[i]) / (knots[i + k] - knots[i]);
    double b = (knots[i + k + 1] - t) / (knots[i + k + 1] - knots[i + 1]);

    double term1 = (knots[i + k] - knots[i] == 0) ? 0 : a * b_spline_basis(i, k - 1, t, knots);
    double term2 = (knots[i + k + 1] - knots[i + 1] == 0) ? 0 : b * b_spline_basis(i + 1, k - 1, t, knots);

    return term1 + term2;
}

Eigen::MatrixXd compute_b_spline_surface(int num_u = 20, int num_v = 20) {
    int m = sqrt(V.rows());  // Nombre de points de contrôle dans la direction U
    int n = sqrt(V.rows());  // Nombre de points de contrôle dans la direction V

    Eigen::VectorXd knots_u = Eigen::VectorXd::LinSpaced(m + 4, 0, 1);
    Eigen::VectorXd knots_v = Eigen::VectorXd::LinSpaced(n + 4, 0, 1);

    Eigen::MatrixXd surface(dimension * dimension, 3);
    int index = 0;

    for (int ui = 0; ui < num_u; ++ui) {
        double u = static_cast<double>(ui) / (num_u - 1);

        for (int vi = 0; vi < num_v; ++vi) {
            double v = static_cast<double>(vi) / (num_v - 1);
            Eigen::Vector3d point(0, 0, 0);

            for (int i = 0; i < m; ++i) {
                for (int j = 0; j < n; ++j) {
                    double Bu = b_spline_basis(i, 3, u, knots_u);
                    double Bv = b_spline_basis(j, 3, v, knots_v);

                    point += Bu * Bv * V.row(i * n + j).transpose();
                }
            }
            surface.row(index++) = point;
        }
    }

    return surface;
}


// Eigen::MatrixXd compute_b_spline_surface()
// {
//     int degree = 3; // Degré cubique

//     // Générer les nœuds correctement
//     Eigen::VectorXd knots_u = generate_knots(dimension, degree);
//     Eigen::VectorXd knots_v = generate_knots(dimension, degree);

//     Eigen::MatrixXd V_(dimension * dimension, 3);

//     for (int ui = 0; ui < dimension; ++ui) {
//         for (int vi = 0; vi < dimension; ++vi) {
//             double u = static_cast<double>(ui) / (dimension - 1) * (dimension - degree);
//             double v = static_cast<double>(vi) / (dimension - 1) * (dimension - degree);

//             Eigen::Vector3d point(0, 0, 0);

//             for (int i = 0; i < dimension; ++i) {
//                 for (int j = 0; j < dimension; ++j) {
//                     double Bu = N(i, degree + 1, u, knots_u);
//                     double Bv = N(j, degree + 1, v, knots_v);
//                     point += Bu * Bv * V.row(i * dimension + j);
//                 }
//             }

//             V_.row(ui * dimension + vi) = point;
//         }
//     }

//     return V_;
// }

Eigen::MatrixXi generate_faces()
{
    Eigen::MatrixXi F(2 * (dimension - 1) * (dimension - 1), 3);

    for (int i = 0; i < dimension - 1; ++i) {
        for (int j = 0; j < dimension - 1; ++j) {
            int idx = i * (dimension - 1) + j;
            F.row(2 * idx) << i * dimension + j, i * dimension + j + 1, (i + 1) * dimension + j + 1;
            F.row(2 * idx + 1) << i * dimension + j, (i + 1) * dimension + j + 1, (i + 1) * dimension + j;
        }
    }
    return F;
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

    // V = compute_b_spline_surface();
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
        std::cout << "Closest point index: " << selected_point << std::endl;
        std::cout << "Closest point coordinates: " << V.row(selected_point) << std::endl;

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
    F = generate_faces();

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
