/**
 * @file chart_to_s57.cpp
 * @brief S-57 导出的命令行入口。
 */

#include <iostream>
#include <string>

#include "s57_chart.hpp"
#include "usv_map/parser.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: chart_to_s57 INPUT.xml OUTPUT.000\n";
    return 1;
  }
  try {
    const auto chart = usv_map::load_chart(argv[1]);
    const auto result = usv_map_tools::export_s57(chart, argv[2]);
    std::cout << "S-57 3.1: " << argv[2] << '\n';
    for (const auto & [group, count] : result.feature_counts) {
      std::cout << "  " << group << ": " << count << '\n';
    }
    if (result.invalid_source_areas > 0) {
      std::cout << "Source contains " << result.invalid_source_areas <<
        " non-simple areas; original outlines preserved.\n";
    }
    std::cout << "Read-back verification passed. Simulation chart; not a surveyed ENC.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "S-57 export failed: " << error.what() << '\n';
    return 1;
  }
}
