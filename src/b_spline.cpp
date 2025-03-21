#include "sources.hpp"

double N(int i, int k, double u, const Eigen::VectorXd &knots)
{
    if (k == 1)
        return (knots(i) <= u && u <= knots(i + 1)) ? 1.0 : 0.0;

    double coef1 = 0.0, coef2 = 0.0;

    if (knots(i + k - 1) != knots(i))
        coef1 = (u - knots(i)) / (knots(i + k - 1) - knots(i)) * N(i, k - 1, u, knots);
    if (knots(i + k) != knots(i + 1))
        coef2 = (knots(i + k) - u) / (knots(i + k) - knots(i + 1)) * N(i + 1, k - 1, u, knots);

    return coef1 + coef2;
}

Eigen::VectorXd generate_knots(int num_ctrl_pts, int degree)
{
    int num_knots = num_ctrl_pts + degree + 1;
    Eigen::VectorXd knots(num_knots);

    for (int i = 0; i <= degree; ++i) {
        knots(i) = 0;
        knots(num_knots - 1 - i) = num_ctrl_pts - degree;
    }
    for (int i = 1; i < num_ctrl_pts - degree; ++i) {
        knots(i + degree) = i;
    }

    return knots;
}

Eigen::MatrixXd compute_b_spline_surface()
{
    int degree = 3;
    int size = dimension * split - (split - 1);

    Eigen::VectorXd knots_u = generate_knots(dimension, degree);
    Eigen::VectorXd knots_v = generate_knots(dimension, degree);
    Eigen::MatrixXd surface(size * size, 3);

    for (int ui = 0; ui < size; ++ui) {
        for (int vi = 0; vi < size; ++vi) {
            double u = static_cast<double>(ui) / (size - 1) * (dimension - 3);
            double v = static_cast<double>(vi) / (size - 1) * (dimension - 3);

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
