/***************************************************************************
 *
 * Copyright (c) 2019 JD.com, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file local_routing_run_conf.cc
 **/
#pragma once
#include <fstream>
#include <utility>
#include "gnuplot-iostream.h"
//#include "component/proto/hdmap/jhdm_nav.pb.h"

enum Color {
  RED = 1,
  GREEN = 2,
  BLUE = 3,
  MAGENTA = 4,
  LIGHTBLUE = 5,
  YELLOW = 6,
  BLACK = 7,
  ORANGE = 8,
  GREY = 9,
  FORESTGREEN = 10,
  CYAN = 11,
  DARKYELLOW = 12,
  GOLD = 13,
  NARY = 14,
  KHAKI = 15,
  CORAL = 16,
  PLUM = 17,
  CHARTEUSE = 18,
  MEDIUM_PURPLE3 = 19,
  BROWN4 = 20,
};

enum LineType {
  SOLID = 1,
  DOTTED = 2,

};

class GnuDraw {
 public:
  GnuDraw() = default;
  static std::string TypeChoose(const LineType& type) {
    std::string types;
    switch (type) {
      case SOLID:
        types = "  linestyle 1 ";
        break;
      case DOTTED:
        types = " linestyle 0 ";
        break;
    }
    return types;
  }
  static std::string ColorChoose(const Color& color) {
    std::string colors;
    switch (color) {
      case RED:
        colors = " lt rgb 'red' ";
        break;
      case GREEN:
        colors = " lt rgb 'green' ";
        break;
      case BLUE:
        colors = " lt rgb 'blue' ";
        break;
      case MAGENTA:
        colors = " lt rgb 'magenta' ";
        break;
      case LIGHTBLUE:
        colors = " lt rgb 'light-blue' ";
        break;
      case YELLOW:
        colors = " lt rgb 'yellow' ";
        break;
      case BLACK:
        colors = " lt rgb 'black' ";
        break;
      case ORANGE:
        colors = " lt rgb 'orange' ";
        break;
      case GREY:
        colors = " lt rgb 'grey' ";
        break;
      case FORESTGREEN:
        colors = " lt rgb 'forest-green' ";
        break;

      case CYAN:
        colors = " lt rgb 'cyan' ";
        break;
      case DARKYELLOW:
        colors = " lt rgb 'dark-yellow' ";
        break;
      case GOLD:
        colors = " lt rgb 'gold' ";
        break;
      case NARY:
        colors = " lt rgb 'navy' ";
        break;
      case KHAKI:
        colors = " lt rgb 'khaki' ";
        break;
      case CORAL:
        colors = " lt rgb 'coral' ";
        break;
      case PLUM:
        colors = " lt rgb 'plum' ";
        break;
      case CHARTEUSE:
        colors = " lt rgb 'chartreuse' ";
        break;
      case MEDIUM_PURPLE3:
        colors = " lt rgb 'mediumpurple3' ";
        break;
      case BROWN4:
        colors = " lt rgb 'brown4' ";
        break;
    }
    return colors;
  }

  static void GnuPlotInit(Gnuplot* const gp, bool open = true) {
    //    *gp<<"clear\n";
    if (open) {
      *gp << "set size ratio -1\n";
    }
    //  *gp << "set xrange [-500:0]\n";
    //  *gp << "set yrange [-200:500]\n"
    //   gp << "set autoscale\n";
    *gp << "set xlabel 'Xlable'\n";
    *gp << "set ylabel 'Xlable'\n";
  }

  static void SetCircle(double x, double y, double raduis, Gnuplot* const gp) {
    *gp << " set object 1 circle at " + std::to_string(x) + "," + std::to_string(y) + " size " +
               std::to_string(raduis) + " fc rgb 'magenta' \n";
  }

  static void Plot(Gnuplot* const gp) {
    *gp << "plot ";
  }

  static void EndPlot(Gnuplot* const gp) {
    *gp << std::endl;
  }
  static void DrawMsg(double x, double y, std::string msg, Gnuplot* const gp) {
    *gp << " set label '" + msg + "' at " + std::to_string(x) + "," + std::to_string(y) +
               " textcolor rgb 'black' offset 0.5,0.5 \n";
  }
  static void DrawPoint(const std::pair<double, double>& point, const Color& color,
                        const double size, Gnuplot* const gp, std::string name = "") {
    Named(name);
    std::vector<std::pair<double, double>> points;
    points.emplace_back(point);
    *gp << gp->file1d(points)
        << "with points " + ColorChoose(color) + " pt 7 ps " + std::to_string(size) + name + ",";
  }
  static void DrawPointWithHeading(double x, double y, double theta, const Color& color,
                                   double width, Gnuplot* const gp, std::string name = "") {
    std::vector<std::pair<double, double>> points;
    points.emplace_back(x, y);
    points.emplace_back(x + cos(theta), y + sin(theta));

    DrawOrietedLine(points, width, color, gp, name);
  }
  static void DrawPoints(const std::vector<std::pair<double, double>>& points, const Color& color,
                         const double size, Gnuplot* const gp, std::string name = "") {
    Named(name);
    *gp << gp->file1d(points)
        << "with points " + ColorChoose(color) + " pt 7 ps " + std::to_string(size) + name + ",";
  }

  static void DrawOrietedLine(const std::vector<std::pair<double, double>>& points, double width,
                              const Color& color, Gnuplot* const gp, std::string name = "") {
    Named(name);
    std::vector<boost::tuple<double, double, double, double>> tuple_points;
    for (uint32_t i = 0; i < points.size() - 1; ++i) {
      auto now = points[i];
      auto next = points[i + 1];
      double dx = next.first - now.first;
      double dy = next.second - now.second;
      tuple_points.push_back(boost::make_tuple(now.first, now.second, dx, dy));
    }
    *gp << gp->file1d(tuple_points)
        << " with vectors " + ColorChoose(color) + " lw " + std::to_string(width) + " " + name +
               ",";
  }

  static void DrawLine(const std::vector<std::pair<double, double>>& points, const Color& color,
                       const double size, Gnuplot* const gp, std::string name = "") {
    Named(name);
    *gp << gp->file1d(points)
        << "with linespoints  " + ColorChoose(color) + " lw " + std::to_string(size) +
               " pt 7 ps  0.1 " + name + ",";
  }

  static void DrawLineWithType(const std::vector<std::pair<double, double>>& points,
                               const LineType& type, const Color& color, const double size,
                               Gnuplot* const gp, std::string name = "") {
    Named(name);
    *gp << gp->file1d(points)
        << "with linespoints  " + TypeChoose(type) + ColorChoose(color) + " lw " +
               std::to_string(size) + " pt 7 ps  0.1 " + name + ",";
  }

  static void Named(std::string& name) {
    if (name.empty()) {
      name = " notitle ";
    } else {
      name = " title '" + name + "' ";
      std::cout << name << std::endl;
    }
  }

  /////////////////////////////////////hd map ///////////////////////////////////////////////////
#if 0

  static void SetHdMapMsg(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& roads = hd_map.road();

    for (const auto& road : roads) {
      uint32_t index = static_cast<uint32_t>(road.shape().vertex_size() / 2);
      double x = road.shape().vertex(index).x();
      double y = road.shape().vertex(index).y();
      *gp << " set label ' " + road.id() + " ' at " + std::to_string(x) + "," + std::to_string(y) +
                 "textcolor rgb 'green' offset 0.5,0.5 \n";
    }

    const auto& lanes = hd_map.lane();
    for (const auto& lane : lanes) {
      double x1 = lane.center_line().vertex().begin()->x();
      double y1 = lane.center_line().vertex().begin()->y();
      double x2 = lane.center_line().vertex().rbegin()->x();
      double y2 = lane.center_line().vertex().rbegin()->y();
      *gp << " set label ' " + lane.from_node_id() + " ' at " + std::to_string(x1) + "," +
                 std::to_string(y1) + " \n";

      *gp << " set label ' " + lane.to_node_id() + " ' at " + std::to_string(x2) + "," +
                 std::to_string(y2) + "  \n";
    }
    //    const auto& signals = hd_map.signal();
    //    for (const auto& signal : signals) {
    //      double x = signal.position().x();
    //      double y = signal.position().y();
    //      *gp << " set label '" + signal.id() + "' at " + std::to_string(x) + "," +
    //      std::to_string(y) +
    //                 " textcolor rgb 'red' offset 0.5,0.5 \n";
    //    }
  }

  /********************************* 1 road  **************************************/
  static void DrawRoad(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& roads = hd_map.road();
    for (const auto& road : roads) {
      std::vector<std::pair<double, double>> points;
      for (const auto& point : road.shape().vertex()) {
        points.emplace_back(point.x(), point.y());
      }
      DrawOrietedLine(points, 0.3, MEDIUM_PURPLE3, gp);
    }
    for (const auto& road : roads) {
      for (const auto& line : road.left_boundary()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, BLACK, 3, gp);
      }
      for (const auto& line : road.right_boundary()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, BLACK, 3, gp);
      }
    }
  }

  /********************************* 2 lane_group  ******************/

  /********************************* 3 lane  **************************************/
  static void DrawLane(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& lanes = hd_map.lane();
    for (const auto& lane : lanes) {
      std::vector<std::pair<double, double>> points;
      for (const auto& point : lane.center_line().vertex()) {
        points.emplace_back(point.x(), point.y());
      }

      DrawOrietedLine(points, 0.3, BLUE, gp);
    }
    //    for (const auto& lane : lanes) {
    //      for (const auto& left_boundary : lane.left_boundary()) {
    //        std::vector<std::pair<double, double>> points;
    //        for (const auto& point : left_boundary.shape().vertex()) {
    //          points.emplace_back(std::make_pair(point.x(), point.y()));
    //        }
    //        if (left_boundary.height() > 0) {  // 实线
    //          DrawLineWithType(points, SOLID, PLUM, 1.5, gp);
    //        } else {  //虚线
    //          DrawLineWithType(points, DOTTED, PLUM, 1.5, gp);
    //        }
    //      }
    //      for (const auto& left_boundary : lane.right_boundary()) {
    //        std::vector<std::pair<double, double>> points;
    //        for (const auto& point : left_boundary.shape().vertex()) {
    //          points.emplace_back(std::make_pair(point.x(), point.y()));
    //        }
    //        if (left_boundary.height() > 0) {  // 实线
    //          DrawLineWithType(points, SOLID, PLUM, 1.5, gp);
    //        } else {  //虚线
    //          DrawLineWithType(points, DOTTED, PLUM, 1.5, gp);
    //        }
    //      }
    //    }
  }

  /********************************* 4 clear_area  ******************/
  static void DrawClearArea(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& clear_areas = hd_map.clear_area();
    for (const auto& clear_area : clear_areas) {
      for (const auto& line : clear_area.shape().polyline()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, RED, 3, gp);
      }
    }
  }

  /********************************* 5 crosswalk  *******************/
  static void DrawCrosswalk(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& crosswalks = hd_map.crosswalk();
    for (const auto& crosswalk : crosswalks) {
      for (const auto& line : crosswalk.shape().polyline()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, GREY, 6, gp);
      }
    }
  }

  /********************************* 6 free_area  *******************/
  static void DrawFreeArea(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& free_areas = hd_map.free_area();
    for (const auto& free_area : free_areas) {
      for (const auto& line : free_area.shape().polyline()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, GREEN, 3, gp);
      }
    }
  }

  /********************************* 7 gate  **************************************/
  static void DrawGate(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& gates = hd_map.gate();
    for (const auto& gate : gates) {
      std::vector<std::pair<double, double>> points;
      const auto& temp = gate.shape().vertex();
      for (const auto& point : temp) {
        points.emplace_back(std::make_pair(point.x(), point.y()));
      }
      DrawLine(points, RED, 3, gp);
    }
  }

  /********************************* 8 intersection  ****************/
  static void DrawIntersection(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& intersections = hd_map.intersection();
    for (const auto& intersection : intersections) {
      for (const auto& line : intersection.shape().polyline()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, PLUM, 3, gp);
      }
    }
  }

  /********************************* 9 parking  *********************/
  static void DrawParking(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& parkings = hd_map.parking();
    for (const auto& parking : parkings) {
      for (const auto& line : parking.shape().polyline()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, ORANGE, 3, gp);
      }
    }
  }

  /********************************* 10 pillar  *********************/
  static void DrawPillar(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    std::vector<std::pair<double, double>> infrastructures_points;
    const auto& pillars = hd_map.pillar();
    for (const auto& pillar : pillars) {
      for (const auto& line : pillar.shape().polyline()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, RED, 3, gp);
      }
    }
  }

  /********************************* 11 safety_island  **************/
  static void DrawSafetyIsland(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& safety_islands = hd_map.safety_island();
    for (const auto& safety_island : safety_islands) {
      for (const auto& line : safety_island.shape().polyline()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, MEDIUM_PURPLE3, 3, gp);
      }
    }
  }

  /********************************* 12 signal  *********************/
  static void DrawSignal(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& signals = hd_map.signal();
    std::vector<std::pair<double, double>> signal_points;
    for (const auto& signal : signals) {
      double x = signal.position().x();
      double y = signal.position().y();
      signal_points.emplace_back(x, y);
    }
    if (!signal_points.empty()) {
      DrawPoints(signal_points, RED, 3, gp);
    }
  }

  /********************************* 13 speed_bump  *****************/
  static void DrawSpeedHump(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& speed_humps = hd_map.speed_bump();
    for (const auto& speed_hump : speed_humps) {
      for (const auto& line : speed_hump.shape().polyline()) {
        std::vector<std::pair<double, double>> points;
        for (const auto& point : line.vertex()) {
          points.emplace_back(std::make_pair(point.x(), point.y()));
        }
        DrawLine(points, CORAL, 3, gp);
      }
    }
  }

  /********************************* 14 stop_line  ******************/
  static void DrawStopLine(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp) {
    const auto& stop_lines = hd_map.stop_line();
    for (const auto& stop_line : stop_lines) {
      std::vector<std::pair<double, double>> points;
      const auto& temp = stop_line.shape().vertex();
      for (const auto& point : temp) {
        points.emplace_back(std::make_pair(point.x(), point.y()));
      }
      DrawLine(points, MAGENTA, 3, gp);
    }
  }

//  /********************************  16 other  **************************************/
//  static void DrawHdMap(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp,
//                        jdx::routing_run_conf::proto::routing_run_conf& routing_run_conf_) {
//    if (routing_run_conf_.draw_road()) DrawRoad(hd_map, gp);
//    if (routing_run_conf_.draw_lane()) DrawLane(hd_map, gp);
//    if (routing_run_conf_.draw_clear_area()) DrawClearArea(hd_map, gp);
//    if (routing_run_conf_.draw_cross_walk()) DrawCrosswalk(hd_map, gp);
//    if (routing_run_conf_.draw_free_area()) DrawFreeArea(hd_map, gp);
//    if (routing_run_conf_.draw_gate()) DrawGate(hd_map, gp);
//    if (routing_run_conf_.draw_intersection()) DrawIntersection(hd_map, gp);
//    if (routing_run_conf_.draw_parking()) DrawParking(hd_map, gp);
//    if (routing_run_conf_.draw_pillar()) DrawPillar(hd_map, gp);
//    if (routing_run_conf_.draw_safety_island()) DrawSafetyIsland(hd_map, gp);
//    if (routing_run_conf_.draw_signal()) DrawSignal(hd_map, gp);
//    if (routing_run_conf_.draw_speed_hump()) DrawSpeedHump(hd_map, gp);
//    if (routing_run_conf_.draw_stop_line()) DrawStopLine(hd_map, gp);
//
//  }
//    static void DrawHdMap(const jdx::hdmap::proto::Map& hd_map, Gnuplot* const gp
//                        ) {
//    DrawRoad(hd_map, gp);
//     DrawLane(hd_map, gp);
////     DrawClearArea(hd_map, gp);
////    DrawCrosswalk(hd_map, gp);
////    DrawFreeArea(hd_map, gp);
////    DrawGate(hd_map, gp);
////    DrawIntersection(hd_map, gp);
////     DrawParking(hd_map, gp);
////     DrawPillar(hd_map, gp);
////     DrawSafetyIsland(hd_map, gp);
////    DrawSignal(hd_map, gp);
////     DrawSpeedHump(hd_map, gp);
////     DrawStopLine(hd_map, gp);
//
//  }

#endif
};
