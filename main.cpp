

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

Eigen::MatrixXd control_points(16, 3);

void create_points()
{
    control_points <<
    0, 0, 0,
    1, 0, 0,
    2, 0, 0,
    3, 0, 0,
    0, 1, 0,
    1, 1, 1,
    2, 1, 1,
    3, 1, 0,
    0, 2, 0,
    1, 2, 1,
    2, 2, 1,
    3, 2, 0,
    0, 3, 0,
    1, 3, 0,
    2, 3, 0,
    3, 3, 0;
}

int selected_point = -1;


void create_lattice()
{
    static int grid_layer = viewer.append_mesh();
    viewer.data(grid_layer).clear();
    viewer.data(grid_layer).set_points(control_points, Eigen::RowVector3d(1, 0, 0));
    viewer.data(grid_layer).point_size = 10;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            viewer.data(grid_layer).add_edges(
                control_points.row(i * 4 + j),
                control_points.row(i * 4 + j + 1),
                Eigen::RowVector3d(0, 0, 0)
            );
            viewer.data(grid_layer).add_edges(
                control_points.row(j * 4 + i),
                control_points.row((j + 1) * 4 + i),
                Eigen::RowVector3d(0, 0, 0)
            );
        }
    }
}

void show_selected_point()
{
    static int selected_point_layer = -1;
    viewer.data(selected_point_layer).clear();
    selected_point_layer = viewer.append_mesh();
    if (selected_point != -1) {
        viewer.data(selected_point_layer).clear();
        viewer.data(selected_point_layer).set_points(control_points.row(selected_point), Eigen::RowVector3d(1, 1, 0));
        viewer.data(selected_point_layer).point_size = 13;
    }
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
