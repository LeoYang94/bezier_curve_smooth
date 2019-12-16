#include <iostream>
#include <memory>
#include <string>

#include "gnu_draw.h"
#include <bezier/Bezier.hpp>

using Bezier = robotics::Bezier<double, 2>;

std::vector<std::pair<double, double>>
PolyLineToPlotLine(const Bezier::VecPointType &points) {
  std::vector<std::pair<double, double>> plot_line;
  for (const auto &point : points) {
    plot_line.emplace_back(point[0], point[1]);
  }
  return plot_line;
}

int main(int argc, char *argv[]) {

  Bezier::PointType p1(10.83, 6.44);
  Bezier::PointType p2(27.99, 9.75);
  Bezier::PointType p3(43.91, 14.3);
  Bezier::PointType p4(67.48, 24.84);
  Bezier::PointType p5(75.34, 46.97);
  Bezier::PointType p6(65.33, 86.25);

  Bezier::VecPointType pV{p1, p2, p3, p4, p5, p6};
  Bezier::Ptr bezier = std::make_shared<Bezier>(5, pV);
  auto coeffV = bezier->binomialCoeffs();

  const Bezier::Tangent tan = bezier->tangent(0.7);
  const Bezier::Normal nor = bezier->normal(0.7);
  double curvature = bezier->curvature(0.7);

  std::cout << "tangent vector: \n" << tan << "\n";
  std::cout << "normal vector: \n" << nor << "\n";
  std::cout << "dot product: " << tan.dot(nor) << "\n";
  std::cout << "curvature: " << curvature << "\n";

  std::cout << "Original control points: \n";
  for (const auto &p : bezier->controlPoints()) {
    std::cout << p[0] << "," << p[1] << "\n";
  }

  std::cout << "Trajectory: \n";
  Bezier::VecPointType trajectory = bezier->trajectory(6);
  for (const auto &p : trajectory) {
    std::cout << p[0] << "," << p[1] << "\n";
  }

  std::cout << *bezier << "\n";

  auto plot_line_origin = PolyLineToPlotLine(bezier->controlPoints());
  auto plot_line_smoothed = PolyLineToPlotLine(trajectory);

  Gnuplot gp;
  GnuDraw::GnuPlotInit(&gp);
  GnuDraw::Plot(&gp);
  GnuDraw::DrawOrietedLine(plot_line_origin, 1, BLUE, &gp, "origin path");
  GnuDraw::DrawOrietedLine(plot_line_smoothed, 1, MAGENTA, &gp, "smoothed path");
  // GnuDraw::DrawOrietedLine(plot_line_smoothed1, 1, LIGHTBLUE, &gp);
  GnuDraw::EndPlot(&gp);
  sleep(10000);
  return 0;
}
