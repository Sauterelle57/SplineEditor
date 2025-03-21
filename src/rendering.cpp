#include "sources.hpp"

void show_selected_point()
{
    static int selected_point_layer = viewer.append_mesh();

    viewer.data(selected_point_layer).clear();
    if (!show_control_points) return;

    if (selected_point != -1) {
        viewer.data(selected_point_layer).set_points(V.row(selected_point), Eigen::RowVector3d(1, 1, 0));
        viewer.data(selected_point_layer).point_size = 10;
    }
}

void show_points()
{
    static int points_layer = viewer.append_mesh();

    viewer.data(points_layer).clear();
    if (!show_control_points) return;

    viewer.data(points_layer).set_points(V, Eigen::RowVector3d(1, 0, 0));
    viewer.data(points_layer).point_size = 6;
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

void render_surface()
{
    static int surface_layer = viewer.append_mesh();

    viewer.data(surface_layer).clear();
    if (!show_surface) return;

    V_surface = compute_b_spline_surface();
    static Eigen::MatrixXi F_spline = generate_surface(dimension * split - (split - 1));

    viewer.data(surface_layer).set_mesh(V_surface, F_spline);
    viewer.data(surface_layer).show_lines = true;
    viewer.data(surface_layer).show_faces = true;
    viewer.data(surface_layer).set_colors(Eigen::RowVector3d(0.0, 0.8, 0.0));
}
