#include <iostream>

void display_help()
{
    std::cout << "#################################################################" << std::endl;
    std::cout << "#  Usage: " << "./SplineEditor" << "[split] [step]\t\t\t\t#" << std::endl;
    std::cout << "#\t\t\t\t\t\t\t\t#" << std::endl;
    std::cout << "#  " << "- split:\tThe number of points between two control points\t#" << std::endl;
    std::cout << "#  " << "- step:\tThe distance to move a point\t\t\t#" << std::endl;
    std::cout << "#  " << "- Tab+click:\tSelect a point\t\t\t\t\t#" << std::endl;
    std::cout << "#  " << "- W:\t\tMove selected point up\t\t\t\t#" << std::endl;
    std::cout << "#  " << "- A:\t\tMove selected point left\t\t\t#" << std::endl;
    std::cout << "#  " << "- S:\t\tMove selected point down\t\t\t#" << std::endl;
    std::cout << "#  " << "- D:\t\tMove selected point right\t\t\t#" << std::endl;
    std::cout << "#  " << "- Q:\t\tMove selected point forward\t\t\t#" << std::endl;
    std::cout << "#  " << "- E:\t\tMove selected point backward\t\t\t#" << std::endl;
    std::cout << "#  " << "- H:\t\tDisplay this help message\t\t\t#" << std::endl;
    std::cout << "#  " << "- C:\t\tDisplay / Hide control points\t\t\t#" << std::endl;
    std::cout << "#  " << "- B:\t\tDisplay / Hide surface\t\t\t\t#" << std::endl;
    std::cout << "#################################################################" << std::endl;
}
