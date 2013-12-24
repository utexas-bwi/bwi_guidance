#include<fstream>

#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/graph.h>
#include <bwi_mapper/map_utils.h>

int main(int argc, char** argv) {

  bwi_mapper::MapLoader mapper(argv[1]);
  bwi_mapper::Graph graph;
  nav_msgs::OccupancyGrid map;
  mapper.getMap(map);
  bwi_mapper::readGraphFromFile(argv[2], map.info, graph);

  int ids[] = {5, 8, 7, 2, 15, 23, 1, 27, 5, 46, 3, 32, 10, 30};
  int num_ids = 14;
  for (int i = 0; i < num_ids; ++i) {
    bwi_mapper::Point2f p = 
      bwi_mapper::getLocationFromGraphId(ids[i], graph);
    p = bwi_mapper::toMap(p, map.info);
    std::cout << "For " << ids[i] << std::endl;
    if (i % 2 == 0) {
      std::cout << "      - start_x: " << p.x << std::endl;
      std::cout << "        start_y: " << p.y << std::endl;
    } else {
      std::cout << "        goal_x: " << p.x << std::endl;
      std::cout << "        goal_y: " << p.y << std::endl;
    }
  }

  return 0;
}
