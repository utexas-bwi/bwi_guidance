#ifndef BWI_GUIDANCE_SOLVER_PERSON_MODEL_IROS14
#define BWI_GUIDANCE_SOLVER_PERSON_MODEL_IROS14

#include <nav_msgs/OccupancyGrid.h>
#include <rl_pursuit/planning/Model.h>
#include <stdint.h>

#include <bwi_guidance_solver/structures_iros14.h>
#include <bwi_guidance_solver/utils.h>
#include <bwi_mapper/graph.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_guidance {

  class PersonModelIROS14 : public Model<StateIROS14, ActionIROS14> {

    public:

      PersonModelIROS14(const bwi_mapper::Graph& graph, const
          nav_msgs::OccupancyGrid& map, size_t goal_idx, int max_robots_in_use
          = 1, int action_vertex_visibility_depth = 0, float visibility_range =
          0.0f, bool allow_goal_visibility = false, float human_speed = 1.0,
          float robot_speed = 0.75);

      /* Functions inherited from PredictiveModel */
      virtual ~PersonModelIROS14() {};

      /* Functions inherited from Model */
      virtual void setState(const StateIROS14 &state);
      virtual void takeAction(const ActionIROS14 &action, float &reward, 
          StateIROS14 &state, bool &terminal);
      virtual void getFirstAction(const StateIROS14 &state, ActionIROS14 &action);
      virtual bool getNextAction(const StateIROS14 &state, ActionIROS14 &action);
      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

      void initializeRNG(UIGenPtr uigen, URGenPtr ugen, PIGenPtr pgen);
      void addRobots(StateIROS14& state, int n);

      /* Debugging only */
      void drawCurrentState(cv::Mat& image);

      /* Private functions that are public only for testing */
      void moveRobots(float time);
      void printDistanceToDestination(int idx);
      void getActionsAtState(const StateIROS14 &state,
          std::vector<ActionIROS14>& actions);

    private:

      /* Mapped state for generative model */
      StateIROS14 current_state_;
      UIGenPtr uigen_;
      URGenPtr ugen_;
      PIGenPtr pgen_;

      /* StateIROS14 space cache */
      std::map<int, std::vector<int> > adjacent_vertices_map_;
      std::map<int, std::vector<int> > visible_vertices_map_;
      std::map<int, std::vector<int> > action_vertices_map_;

      /* Actions */
      bool isTerminalState(const StateIROS14& state) const;

      /* Next states and transitions */
      float takeActionAtCurrentState(const ActionIROS14 &a);

      /* Helper Functions */
      float getTrueDistanceTo(const RobotStateIROS14& state, int destination);
      int selectBestRobotForTask(int destination, float time_to_destination);
      bool isRobotDirectionAvailable(float& robot_dir);
      int generateNewGoalFrom(int idx);

      /* Action generation caching */
      StateIROS14 get_action_state_;
      std::vector<ActionIROS14> get_actions_;
      int get_actions_counter_;

      /* Goal Caching */
      void cacheNewGoalsByDistance();
      std::vector<std::vector<std::vector<int> > > goals_by_distance_;

      /* Path Caching */
      std::vector<std::vector<std::vector<size_t> > > shortest_paths_;
      std::vector<std::vector<float> > shortest_distances_;
      void cacheShortestPaths();

      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(adjacent_vertices_map_);
        ar & BOOST_SERIALIZATION_NVP(visible_vertices_map_);
        ar & BOOST_SERIALIZATION_NVP(action_vertices_map_);
        ar & num_vertices_;
      }

      bwi_mapper::Graph graph_;
      nav_msgs::OccupancyGrid map_;

      bool initialized_;
      unsigned int num_vertices_;
      int max_robots_in_use_;

      size_t goal_idx_;
      bool allow_goal_visibility_;
      float human_speed_;
      float robot_speed_;

  };
  
} /* bwi_guidance */

BOOST_CLASS_TRACKING(bwi_guidance::PersonModelIROS14, 
    boost::serialization::track_never)

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_PERSON_MODEL_IROS14 */
