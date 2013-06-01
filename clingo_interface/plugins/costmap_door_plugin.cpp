#include <clingo_interface/costmap_door_plugin.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(clingo_interface::CostmapDoorPlugin, costmap_2d::Layer)

namespace clingo_interface {

  CostmapDoorPlugin::~CostmapDoorPlugin() {}

  void CostmapDoorPlugin::onInitialize() {

    /* Read parameters */
    ros::NodeHandle nh("~/" + name_);
    nh.getParam("map_file", map_file_);
    nh.getParam("door_file", door_yaml_file_);
    nh.getParam("location_file", location_file_);

    /* Initialize information about the doors */
    readDoorFile(door_yaml_file_, doors_);
    readLocationFile(location_file_, locations_);

    /* Initialize map information*/
    /* TODO we can get rid of this */
    mapper_.reset(new topological_mapper::MapLoader(map_file_));
    nav_msgs::OccupancyGrid grid;
    mapper_->getMap(grid);
    info_ = grid.info;
    
    matchSize();
    costmap_current_ = false;

  }

  void CostmapDoorPlugin::updateBounds(
      double origin_x, double origin_y, double origin_yaw, 
      double* min_x, double* min_y, double* max_x, double* max_y) {

    boost::mutex::scoped_lock lock(door_plugin_mutex_);
    // I am not sure what the best way to handle bounds are for now. Let's
    // recompute the entire costmap for now
    *min_x = bound_left_;
    *min_y = bound_down_;
    *max_x = bound_right_;
    *max_y = bound_up_; 
      
    bound_left_ = bound_down_ = std::numeric_limits<float>::max();
    bound_right_ = bound_up_ = -std::numeric_limits<float>::max();

  }

  void CostmapDoorPlugin::updateCosts(
      costmap_2d::Costmap2D& master_grid,
      int min_i, int min_j, int max_i, int max_j) {

    boost::mutex::scoped_lock lock(door_plugin_mutex_);
    unsigned char* master_array = master_grid.getCharMap();
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        int index = master_grid.getIndex(i, j);
        unsigned char old_cost = master_array[index];
        unsigned char cost = plugin_layer_value_[index];

        if (old_cost == costmap_2d::NO_INFORMATION && cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          master_array[index] = cost;
        else
          master_array[index] = std::max(old_cost, cost);
      }
    }
    costmap_current_ = true;
  }

  bool CostmapDoorPlugin::isDiscretized() {
    return true;
  }

  void CostmapDoorPlugin::matchSize() {
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    unsigned int size_x = costmap->getSizeInCellsX();
    unsigned int size_y = costmap->getSizeInCellsY();
    plugin_layer_value_.resize(size_x * size_y, costmap_2d::FREE_SPACE);

  }

  void CostmapDoorPlugin::onFootprintChanged() {}
  
} /* clingo_interface */
