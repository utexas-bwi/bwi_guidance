#include <clingo_interface/costmap_door_plugin.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(clingo_interface::CostmapDoorPlugin, costmap_2d::Layer)

namespace clingo_interface {

  virtual CostmapDoorPlugin::~CostmapDoorPlugin() {}

  virtual void CostmapDoorPlugin::onInitialize() {

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
    map_frame_id_ = "/map";
    //map_frame_id_ = grid.header.frame_id; //TODO
    
    matchSize();
    costmap_current_ = false;

  }

  virtual void CostmapDoorPlugin::updateBounds(
      double origin_x, double origin_y, double origin_yaw, 
      double* min_x, double* min_y, double* max_x, double* max_y) {

    // I am not sure what the best way to handle bounds are for now. Let's
    // recompute the entire costmap for now
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();

  }

  virtual void CostmapDoorPlugin::updateCosts(
      costmap_2d::Costmap2D& master_grid,
      int min_i, int min_j, int max_i, int max_j) {

    boost::mutex::scoped_lock lock(door_plugin_mutex_);
    unsigned char* master_array = master_grid.getCharMap();
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        int index = master_grid.getIndex(i, j);
        old_cost = master_array(index);
        cost = plugin_layer_value_[index];

        if (old_cost == NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE)
          grid[index] = cost;
        else
          grid[index] = std::max(old_cost, cost);
      }
    }
    costmap_current_ = true;
  }

  virtual bool CostmapDoorPlugin::isDiscretized() {
    return true;
  }

  virtual void matchSize() {
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    unsigned int size_x = costmap->getSizeInCellsX();
    unsigned int size_y = costmap->getSizeInCellsY();
    plugin_layer_value_.resize(size_x * size_y, costmap_2d::FREE_SPACE);

  }

  virtual void onFootprintChanged() {}
  
} /* clingo_interface */
