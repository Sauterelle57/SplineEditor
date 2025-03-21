#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <iostream>
#include <imgui.h>
#include <GLFW/glfw3.h>

#include "sources.hpp"

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

void init()
{
    V = create_points();
    V_surface = create_points(split);
    show_points();
    render_surface();
}

int main(int argc, char *argv[])
{
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    if (argc >= 2) {
        if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
            display_help();
            return 0;
        }
        split = std::stoi(argv[1]);
    }
    if (argc >= 3) {
        step = std::stod(argv[2]);
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    viewer.launch_init();

    ImGui_ImplGlfw_InitForOpenGL(viewer.window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    init();

    viewer.callback_mouse_down = &mouse_down_callback;
    viewer.callback_key_down = &key_down_callback;

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
