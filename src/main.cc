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

bool ReadDataFromFile(const std::string &x_file_name,
                      const std::string &y_file_name,
                      std::vector<std::pair<double, double>> *const raw_path) {
  std::ifstream x_file(x_file_name);
  std::ifstream y_file(y_file_name);
  std::string x_str, y_str;
  std::vector<double> x;
  std::vector<double> y;
  while (x_file >> x_str) {
    x.emplace_back(std::stod(x_str));
  }
  while (y_file >> y_str) {
    y.emplace_back(std::stod(y_str));
  }
  x_file.close();
  y_file.close();
  if (x.size() != y.size()) {
    std::cerr << "x y size error" << std::endl;
    return false;
  }
  for (uint32_t i = 0; i < x.size(); ++i) {
    raw_path->emplace_back(std::make_pair(x[i], y[i]));
  }
  return true;
}

int main(int argc, char *argv[]) {
  using Point = std::vector<std::pair<double, double>>;
  Point res;
  Point raw_path;
  Point control_points;
  if (!ReadDataFromFile("/home/leo/code/bezier_curve_smooth/data/data1x.txt",
                        "/home/leo/code/bezier_curve_smooth/data/data1y.txt",
                        &raw_path)) {
    std::cout << "wrong data!" << std::endl;
    return -1;
  }
  //保存到每个6个中
  std::vector<Point> SegmentSix;
  for (uint32_t i = 0; i < 501;) {
    Point p;
    int begin_index = i;
    if (i != 0) {
      p.emplace_back(raw_path[begin_index - 1].first,
                     raw_path[begin_index - 1].second);
    } else {
      p.emplace_back(raw_path[0].first, raw_path[0].second);
    }
    for (size_t k = 0; k < 5; ++k) {
      p.emplace_back(raw_path[begin_index].first, raw_path[begin_index].second);
      begin_index++;
    }
    SegmentSix.emplace_back(p);
    i += 5;
  }

  for (uint32_t i = 0; i < SegmentSix.size(); ++i) {
    Bezier::PointType p1(SegmentSix[i][0].first, SegmentSix[i][0].second);
    Bezier::PointType p2(SegmentSix[i][1].first, SegmentSix[i][1].second);
    Bezier::PointType p3(SegmentSix[i][2].first, SegmentSix[i][2].second);
    Bezier::PointType p4(SegmentSix[i][3].first, SegmentSix[i][3].second);
    Bezier::PointType p5(SegmentSix[i][4].first, SegmentSix[i][4].second);
    Bezier::PointType p6(SegmentSix[i][5].first, SegmentSix[i][5].second);
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
    std::cout << "Trajectory: \n";
    auto current_control_points = PolyLineToPlotLine(bezier->controlPoints());
    Bezier::VecPointType trajectory = bezier->trajectory(20);
    auto plot_line_smoothed = PolyLineToPlotLine(trajectory);
    if (i != 0) {
      plot_line_smoothed.pop_back();
    }
    res.insert(res.end(), plot_line_smoothed.begin(), plot_line_smoothed.end());
    control_points.insert(control_points.end(), current_control_points.begin(),
                          current_control_points.end());
  }
  std::cout << "control_points.size():" << control_points.size() << std::endl;
  std::cout << "res_points.size():" << res.size() << std::endl;

  Gnuplot gp;
  GnuDraw::GnuPlotInit(&gp);
  GnuDraw::Plot(&gp);
  GnuDraw::DrawOrietedLine(control_points, 1, BLUE, &gp, "origin path");
  GnuDraw::DrawOrietedLine(res, 1, MAGENTA, &gp, "smoothed path");
  // GnuDraw::DrawOrietedLine(plot_line_smoothed1, 1, LIGHTBLUE, &gp);
  GnuDraw::EndPlot(&gp);
  sleep(10000);
  return 0;
}
