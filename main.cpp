

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

int dimension = 8;

Eigen::MatrixXd control_points(dimension*dimension, 3);

void create_points()
{
    control_points.resize(dimension * dimension, 3);

    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
            int index = i * dimension + j;
            float x = (float)j / (dimension - 1);
            float y = (float)i / (dimension - 1);
            control_points.row(index) << x, y, 0; // Génération en grille régulière
        }
    }
}

int selected_point = -1;


void create_lattice()
{
    static int grid_layer = viewer.append_mesh();
    if (grid_layer == -1) grid_layer = viewer.append_mesh();
    viewer.data(grid_layer).clear();
    viewer.data(grid_layer).set_points(control_points, Eigen::RowVector3d(1, 0, 0));
    viewer.data(grid_layer).point_size = 6;

    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension-1; ++j) {
            viewer.data(grid_layer).add_edges(
                control_points.row(i * dimension + j),
                control_points.row(i * dimension + j + 1),
                Eigen::RowVector3d(0, 0, 0)
            );
            viewer.data(grid_layer).add_edges(
                control_points.row(j * dimension + i),
                control_points.row((j + 1) * dimension + i),
                Eigen::RowVector3d(0, 0, 0)
            );
        }
    }
}

void show_selected_point()
{
    static int selected_point_layer = -1;
    if (selected_point_layer == -1) selected_point_layer = viewer.append_mesh();
    viewer.data(selected_point_layer).clear();
    // selected_point_layer = viewer.append_mesh();
    if (selected_point != -1) {
        viewer.data(selected_point_layer).set_points(control_points.row(selected_point), Eigen::RowVector3d(1, 1, 0));
        viewer.data(selected_point_layer).point_size = 10;
    }
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

Eigen::MatrixXd compute_b_spline_surface(const Eigen::MatrixXd &control_points, int num_u, int num_v)
{
    int degree = 3; // Degré cubique

    // Générer les nœuds correctement
    Eigen::VectorXd knots_u = generate_knots(dimension, degree);
    Eigen::VectorXd knots_v = generate_knots(dimension, degree);

    Eigen::MatrixXd surface(num_u * num_v, 3);

    for (int ui = 0; ui < num_u; ++ui)
    {
        for (int vi = 0; vi < num_v; ++vi)
        {
            double u = static_cast<double>(ui) / (num_u - 1) * (dimension - degree);
            double v = static_cast<double>(vi) / (num_v - 1) * (dimension - degree);

            Eigen::Vector3d point(0, 0, 0);

            for (int i = 0; i < dimension; ++i)
            {
                for (int j = 0; j < dimension; ++j)
                {
                    double Bu = N(i, degree + 1, u, knots_u);
                    double Bv = N(j, degree + 1, v, knots_v);
                    point += Bu * Bv * control_points.row(i * dimension + j);
                }
            }

            surface.row(ui * num_v + vi) = point;
        }
    }

    return surface;
}

Eigen::MatrixXi generate_faces(int num_u, int num_v)
{
    Eigen::MatrixXi F(2 * (num_u - 1) * (num_v - 1), 3);
    int f = 0;

    for (int ui = 0; ui < num_u - 1; ++ui) {
        for (int vi = 0; vi < num_v - 1; ++vi) {
            int idx = ui * num_v + vi;

            F.row(f++) << idx, idx + 1, idx + num_v;
            F.row(f++) << idx + 1, idx + num_v + 1, idx + num_v;
        }
    }

    return F;
}

void display_b_spline_surface()
{
    static int surface_layer = viewer.append_mesh();
    if (surface_layer == -1) surface_layer = viewer.append_mesh();

    int num_u = 20;
    int num_v = 20;

    Eigen::MatrixXd surface = compute_b_spline_surface(control_points, num_u, num_v);
    Eigen::MatrixXi faces = generate_faces(num_u, num_v);

    viewer.data(surface_layer).clear();
    viewer.data(surface_layer).set_mesh(surface, faces);
    viewer.data(surface_layer).set_face_based(true);
    // viewer.data(surface_layer).show_lines = false;
    viewer.data(surface_layer).set_colors(Eigen::RowVector3d(0, 0.8, 0));
}

bool key_down_callback(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier)
{
    if (selected_point == -1) {
        return false; // No point selected
    }

    // Define the movement step size
    double step = 0.1;

    // Move the selected point based on the key pressed
    switch (key) {
        case 'W': // Up
        case 'w':
            control_points(selected_point, 1) += step;
            break;
        case 'S': // Down
        case 's':
            control_points(selected_point, 1) -= step;
            break;
        case 'A': // Left
        case 'a':
            control_points(selected_point, 0) -= step;
            break;
        case 'D': // Right
        case 'd':
            control_points(selected_point, 0) += step;
            break;
        case 'Q': // Forward (Z-axis)
        case 'q':
            control_points(selected_point, 2) += step;
            break;
        case 'E': // Backward (Z-axis)
        case 'e':
            control_points(selected_point, 2) -= step;
            break;
        default:
            return false; // Key not handled
    }

    // Update the lattice and selected point visualization
    viewer.data().clear();
    create_lattice();
    show_selected_point();
    display_b_spline_surface();

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
    for (int i = 0; i < control_points.rows(); ++i) {
        Eigen::Vector3f point = control_points.row(i).cast<float>();
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
        std::cout << "Closest point coordinates: " << control_points.row(selected_point) << std::endl;
        show_selected_point();
        display_b_spline_surface();
    }

    return true;
}

void draw_gizmo()
{
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

    if (selected_point != -1) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 1>(0, 3) = control_points.row(selected_point).cast<float>();

        ImGuizmo::Manipulate(
            viewer.core().view.data(), viewer.core().proj.data(),
            ImGuizmo::TRANSLATE, ImGuizmo::LOCAL, transform.data());

        control_points.row(selected_point) = transform.block<3, 1>(0, 3).cast<double>();
    }
}

// // Compute the uniform cubic B-Spline basis function
// double bspline_basis(int i, int p, double u, const std::vector<double>& knots)
// {
//     if (p == 0) {
//         return (u >= knots[i] && u < knots[i + 1]) ? 1.0 : 0.0;
//     }
//
//     double left = (knots[i + p] - knots[i] != 0) ?
//         (u - knots[i]) / (knots[i + p] - knots[i]) * bspline_basis(i, p - 1, u, knots) : 0.0;
//
//     double right = (knots[i + p + 1] - knots[i + 1] != 0) ?
//         (knots[i + p + 1] - u) / (knots[i + p + 1] - knots[i + 1]) * bspline_basis(i + 1, p - 1, u, knots) : 0.0;
//
//     return left + right;
// }

// Eigen::MatrixXd compute_bspline_surface(const Eigen::MatrixXd& control_points, int resolution)
// {
//     int num_u = 4; // Number of control points in the u direction
//     int num_v = 4; // Number of control points in the v direction
//     int p = 3;     // Degree of the B-Spline (cubic)
//     int q = 3;     // Degree of the B-Spline (cubic)
//
//     // Uniform knot vectors
//     std::vector<double> knots_u = {0, 1, 2, 3, 4, 5, 6, 7};
//     std::vector<double> knots_v = {0, 1, 2, 3, 4, 5, 6, 7};
//
//     Eigen::MatrixXd surface_points(resolution * resolution, 3);
//
//     double step_u = (knots_u[num_u] - knots_u[p]) / (resolution - 1);
//     double step_v = (knots_v[num_v] - knots_v[q]) / (resolution - 1);
//
//     for (int i = 0; i < resolution; ++i) {
//         double u = knots_u[p] + i * step_u;
//
//         for (int j = 0; j < resolution; ++j) {
//             double v = knots_v[q] + j * step_v;
//
//             Eigen::Vector3d point(0, 0, 0);
//
//             for (int k = 0; k < num_u; ++k) {
//                 for (int l = 0; l < num_v; ++l) {
//                     double basis_u = bspline_basis(k, p, u, knots_u);
//                     double basis_v = bspline_basis(l, q, v, knots_v);
//                     point += control_points.row(k * num_v + l) * basis_u * basis_v;
//                 }
//             }
//
//             surface_points.row(i * resolution + j) = point;
//         }
//     }
//
//     std::cout << "Surface Points (first 5):" << std::endl;
//     for (int i = 0; i < 5; ++i) {
//         std::cout << surface_points.row(i) << std::endl;
//     }
//
//     return surface_points;
// }

// void render_bspline_surface(const Eigen::MatrixXd& surface_points, int resolution)
// {
//     static int surface_layer = viewer.append_mesh();
//
//     // Create a grid of vertices and faces
//     Eigen::MatrixXd V = surface_points;
//     Eigen::MatrixXi F((resolution - 1) * (resolution - 1), 4);
//
//     for (int i = 0; i < resolution - 1; ++i) {
//         for (int j = 0; j < resolution - 1; ++j) {
//             int idx = i * (resolution - 1) + j;
//             F.row(idx) << i * resolution + j,
//                           i * resolution + j + 1,
//                           (i + 1) * resolution + j + 1,
//                           (i + 1) * resolution + j;
//         }
//     }
//     // Set the mesh data
//
//     viewer.data(surface_layer).clear();
//     viewer.data(surface_layer).set_mesh(V, F);
//     viewer.data(surface_layer).set_colors(Eigen::RowVector3d(0.2, 0.6, 0.9));
// }

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

    create_points();
    create_lattice();

    display_b_spline_surface();

    // Compute and render the B-Spline surface
    // int resolution = 4; // Resolution of the surface grid
    // Eigen::MatrixXd surface_points = compute_bspline_surface(control_points, resolution);
    // render_bspline_surface(surface_points, resolution);

    viewer.callback_mouse_down = &mouse_down_callback;
    viewer.callback_key_down = &key_down_callback; // Register keyboard callback

    while (!glfwWindowShouldClose(viewer.window)) {
        viewer.draw();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        draw_gizmo();

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
