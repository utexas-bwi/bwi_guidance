#ifndef BWI_GUIDANCE_SOLVER_PERSON_MODEL_IROS14
#define BWI_GUIDANCE_SOLVER_PERSON_MODEL_IROS14

#include <nav_msgs/OccupancyGrid.h>
#include <bwi_rl/planning/Model.h>
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

  const int ROBOT_HOME_BASE[] = {27, 25, 23, 37, 36, 45, 13, 42, 43, 8};

  class PersonModelIROS14 : public Model<StateIROS14, ActionIROS14> {

    public:

      PersonModelIROS14(const bwi_mapper::Graph& graph, 
          const nav_msgs::OccupancyGrid& map, size_t goal_idx, 
          float frame_rate = 0.0f, int max_robots_in_use = 1, 
          int action_vertex_visibility_depth = 0, 
          int action_vertex_adjacency_depth = 2, float visibility_range = 0.0f,
          bool allow_goal_visibility = false, float human_speed = 1.0,
          float robot_speed = 0.75, float utility_multiplier = 0.0f,
          bool use_shaping_reward = true, bool discourage_bad_assignments = false);

      /* Functions inherited from PredictiveModel */
      virtual ~PersonModelIROS14() {};

      /* Functions inherited from Model */
      virtual void takeAction(const StateIROS14 &state, const ActionIROS14 &action, float &reward, 
          StateIROS14 &next_state, bool &terminal, int &depth_count);
      virtual void getFirstAction(const StateIROS14 &state, ActionIROS14 &action);
      virtual bool getNextAction(const StateIROS14 &state, ActionIROS14 &action);
      virtual void getAllActions(const StateIROS14 &state, std::vector<ActionIROS14>& actions);
      virtual std::string generateDescription(unsigned int indentation = 0) {
        return std::string("stub");
      }

      void initializeRNG(UIGenPtr uigen, URGenPtr ugen, PIGenPtr pgen);
      void addRobots(StateIROS14& state, int n);
      int selectBestRobotForTask(int destination, float time_to_destination,
          bool& reach_in_time);
      void getLossesInPreviousTransition(float& time_loss, float& utility_loss); 

      /* Debugging only */
      void drawState(const StateIROS14& state, cv::Mat& image);

      /* Private functions that are public only for testing */
      bool isRobotDirectionAvailable(const StateIROS14& state, int& robot_dir);
      bool moveRobots(StateIROS14& state, float time);
      void printDistanceToDestination(int idx);
      void getActionsAtState(const StateIROS14 &state,
          std::vector<ActionIROS14>& actions);
      void setFrameVector(boost::shared_ptr<std::vector<StateIROS14> >& frame_vector);
      void changeRobotDirectionIfNeeded(RobotStateIROS14& state, 
          int current_destination, int to_destination);

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
      float getTrueDistanceTo(RobotStateIROS14& state, 
          int current_destination, int to_destination, 
          bool change_robot_state = false);
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

      float frame_rate_;
      boost::shared_ptr<std::vector<StateIROS14> > frame_vector_;
      bool initialized_;
      unsigned int num_vertices_;
      int max_robots_in_use_;
      float utility_multiplier_;
      bool use_shaping_reward_;
      bool discourage_bad_assignments_;

      float previous_action_time_loss_;
      float previous_action_utility_loss_;

      size_t goal_idx_;
      bool allow_goal_visibility_;
      float human_speed_;
      float robot_speed_;

  };
  
} /* bwi_guidance */

BOOST_CLASS_TRACKING(bwi_guidance::PersonModelIROS14, 
    boost::serialization::track_never)

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_PERSON_MODEL_IROS14 */
