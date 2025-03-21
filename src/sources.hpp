#ifndef _SOURCES_HPP
#define _SOURCES_HPP

#include "../cmake/imgui_impl_glfw.h"
#include "../cmake/imgui_impl_opengl3.h"
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>

inline Eigen::MatrixXd V;
inline Eigen::MatrixXd V_surface;
inline igl::opengl::glfw::Viewer viewer;

inline int split = 2;
inline double step = 0.2;
inline int dimension = 6;
inline int selected_point = -1;
inline bool show_surface = true;
inline bool show_control_points = true;

void show_points();
void display_help();
void render_surface();
void show_selected_point();
Eigen::MatrixXi generate_surface(int size);
Eigen::MatrixXd compute_b_spline_surface();
bool mouse_down_callback(igl::opengl::glfw::Viewer &viewer, int button, int modifier);
bool key_down_callback(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier);

#endif // _SOURCES_HPP