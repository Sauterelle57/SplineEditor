#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_ray.h>
#include <Eigen/Core>
#include <iostream>

void draw_lattice(igl::opengl::glfw::Viewer &viewer, Eigen::MatrixXd &V, Eigen::MatrixXi &E, int size_x = 10, int size_y = 10)
{
    V.resize((size_x + 1) * (size_y + 1), 3);
    std::vector<Eigen::RowVector2i> edges;

    for (int x = 0, index = 0; x <= size_x; ++x)
    {
        for (int y = 0; y <= size_y; ++y)
        {
            V.row(index) << x, 0, y;

            if (y < size_y)
                edges.push_back(Eigen::RowVector2i(index, index + 1));

            if (x < size_x)
                edges.push_back(Eigen::RowVector2i(index, index + (size_y + 1)));

            ++index;
        }
    }

    E.resize(edges.size(), 2);
    for (int i = 0; i < edges.size(); ++i)
    {
        E.row(i) = edges[i];
    }

    viewer.data().set_points(V, Eigen::RowVector3d(0, 0, 0));
    viewer.data().set_edges(V, E, Eigen::RowVector3d(0.3, 0.3, 0.3));
    viewer.data().point_size = 5;
    viewer.data().line_width = 1;

    viewer.core().background_color = Eigen::Vector4f(0.7, 0.7, 0.7, 1);
}

void add_point(igl::opengl::glfw::Viewer &viewer, Eigen::Vector3f pos)
{
    Eigen::MatrixXd P(1, 3);
    P << pos.x(), pos.y(), pos.z();

    int point_layer = viewer.append_mesh();
    viewer.data(point_layer).set_points(P, Eigen::RowVector3d(1, 0, 0));
    viewer.data(point_layer).point_size = 12;

    std::cout << "Added red point at (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n";
}


bool mouse_down_callback(igl::opengl::glfw::Viewer &viewer, int button, int modifier)
{
    if (button != 0) // We only want the left click
        return false;

    if (!(modifier & GLFW_MOD_ALT)) // Check if the Alt key is pressed
        return false;

    Eigen::Vector3f pos, dir;
    igl::unproject_ray(
        Eigen::Vector2f(viewer.current_mouse_x, viewer.core().viewport[3] - viewer.current_mouse_y),
        viewer.core().view,
        viewer.core().proj,
        viewer.core().viewport,
        pos,
        dir);

    // Intersection with the plane y=0
    if (dir.y() == 0)
        return false; // The ray is parallel to the plane

    float t = -pos.y() / dir.y();
    Eigen::Vector3f intersection = pos + t * dir;

    add_point(viewer, intersection);

    return true; // To indicate that we captured the click
}

int main()
{
    igl::opengl::glfw::Viewer viewer;
    Eigen::MatrixXd V;
    Eigen::MatrixXi E;

    draw_lattice(viewer, V, E, 10, 10);

    viewer.callback_mouse_down = &mouse_down_callback;

    viewer.launch();
}
