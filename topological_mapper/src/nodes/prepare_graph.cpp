#include <topological_mapper/topological_mapper.h>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace topological_mapper;

void drawElementsFile(const std::string& elements_file, cv::Mat& image,
    const Graph& graph) {

  std::ifstream fin(elements_file.c_str());
  YAML::Parser parser(fin);

  YAML::Node node;
  std::vector<std::pair<size_t, size_t> > edges;
  std::vector<std::pair<size_t, float> > arrows;
  std::vector<int> circles, squares;
  while(parser.GetNextDocument(node)) {

    // Get Edges
    const YAML::Node& edges_node = node["edges"];
    for(unsigned i = 0; i < edges_node.size(); ++i) {
      std::pair<size_t, size_t> edge;
      edges_node[i][0] >> edge.first;
      edges_node[i][1] >> edge.second;
      edges.push_back(edge);
    }

    // Get Circles
    const YAML::Node& circles_node = node["circles"];
    for(unsigned i = 0; i < circles_node.size(); ++i) {
      int circle;
      circles_node[i] >> circle;
      circles.push_back(circle);
    }

    // Get Squares
    const YAML::Node& squares_node = node["squares"];
    for(unsigned i = 0; i < squares_node.size(); ++i) {
      int square;
      squares_node[i] >> square;
      squares.push_back(square);
    }

    // Get Directional arrows
    const YAML::Node& arrows_node = node["arrows"];
    for(unsigned i = 0; i < arrows_node.size(); ++i) {
      std::pair<size_t, float> arrow;
      arrows_node[i][0] >> arrow.first;
      size_t direction;
      arrows_node[i][1] >> direction;
      arrow.second = ((2 * M_PI) / 16) * direction + M_PI/2;
      arrows.push_back(arrow);
    }
  }

  topological_mapper::drawGraph(image, graph, 0, 0, true, false, edges);
  for(unsigned i = 0; i < circles.size(); ++i) {
    drawCircleOnGraph(image, graph, circles[i]);  
  }
  for(unsigned i = 0; i < squares.size(); ++i) {
    drawSquareOnGraph(image, graph, squares[i]);  
  }
  for(unsigned i = 0; i < arrows.size(); ++i) {
    drawArrowOnGraph(image, graph, arrows[i]);  
  }

}

int main(int argc, char** argv) {

  if (argc < 4) {
    std::cerr << "USAGE: " << argv[0] 
        << " <yaml-map-file> <yaml-graph-file> <yaml-elements-file>" << std::endl;
    return -1;
  }

  topological_mapper::TopologicalMapper mapper(argv[1]);
  topological_mapper::Graph graph;
  nav_msgs::MapMetaData info;
  mapper.getMapInfo(info);
  topological_mapper::readGraphFromFile(argv[2], info, graph);

  cv::Mat image;
  mapper.drawMap(image);
  drawElementsFile(argv[3], image, graph);

  cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
  cv::imshow("Display window", image);                
  cv::imwrite("out.png", image);                

  cv::waitKey(0); // Wait for a keystroke in the window
  return 0;
}

