#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject.h>
#include <igl/unproject_ray.h>

#include "sources.hpp"

bool key_down_callback(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier)
{
    if (selected_point == -1)
        return false;

    switch (key) {
        case 'W': V(selected_point, 1) += step; break;
        case 'S': V(selected_point, 1) -= step; break;
        case 'A': V(selected_point, 0) -= step; break;
        case 'D': V(selected_point, 0) += step; break;
        case 'Q': V(selected_point, 2) += step; break;
        case 'E': V(selected_point, 2) -= step; break;
        case 'H': display_help(); break;
        case 'C': show_control_points = !show_control_points; break;
        case 'B': show_surface = !show_surface; break;
        default: return false;
    }

    show_selected_point();
    show_points();
    render_surface();

    return true;
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

    if (selected_point != -1) show_selected_point();

    return true;
}
