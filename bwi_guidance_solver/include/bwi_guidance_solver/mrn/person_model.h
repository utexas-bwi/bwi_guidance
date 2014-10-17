#ifndef BWI_GUIDANCE_SOLVER_PERSON_MODEL_H
#define BWI_GUIDANCE_SOLVER_PERSON_MODEL_H

#include <nav_msgs/OccupancyGrid.h>
#include <bwi_rl/planning/Model.h>
#include <stdint.h>

#include <bwi_guidance_solver/mrn/structures.h>
#include <bwi_mapper/graph.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_guidance_solver {

  namespace mrn {
    
    const unsigned int DEFAULT_MAX_ROBOTS = 10;

    // TODO: This should be specified with the map.
    const int ROBOT_HOME_BASE[] = {27, 25, 23, 37, 36, 45, 13, 42, 43, 8};

    class PersonModel : public Model<State, Action> {

      public:

        PersonModel(const bwi_mapper::Graph& graph, 
                    const nav_msgs::OccupancyGrid& map, size_t goal_idx, 
                    float frame_rate = 0.0f, int max_robots_in_use = 1, 
                    int action_vertex_visibility_depth = 0, 
                    int action_vertex_adjacency_depth = 2, float visibility_range = 0.0f,
                    float human_speed = 1.0,
                    float robot_speed = 0.75, float utility_multiplier = 0.0f,
                    bool use_shaping_reward = true, bool discourage_bad_assignments = false);

        /* Functions inherited from PredictiveModel */
        virtual ~PersonModel() {};

        /* Functions inherited from Model */
        virtual void takeAction(const State &state, const Action &action, float &reward, 
                                State &next_state, bool &terminal, int &depth_count, boost::shared_ptr<RNG> rng);
        virtual void getFirstAction(const State &state, Action &action);
        virtual bool getNextAction(const State &state, Action &action);
        virtual void getAllActions(const State &state, std::vector<Action>& actions);
        virtual std::string generateDescription(unsigned int indentation = 0) {
          return std::string("stub");
        }

        /* Some model specific functions, useful while evaluating. */
        void takeAction(const State &state, 
                        const Action &action, 
                        float &reward, 
                        State &next_state, 
                        bool &terminal, 
                        int &depth_count,
                        boost::shared_ptr<RNG> &rng,
                        float &time_loss,
                        float &utility_loss,
                        std::vector<State> &frame_vector);

        void addRobots(State& state, int n, boost::shared_ptr<RNG> &rng);
        int selectBestRobotForTask(const State& state,
                                   int destination, 
                                   float time_to_destination,
                                   bool& reach_in_time);

        /* Debugging only */
        void drawState(const State& state, cv::Mat& image);

        /* Private functions that are public only for testing */
        bool isRobotDirectionAvailable(const State& state, int& robot_dir);
        bool moveRobots(State& state, float time, boost::shared_ptr<RNG> &rng);
        void printDistanceToDestination(int idx);
        void getActionsAtState(const State &state,
                               std::vector<Action>& actions);
        void setFrameVector(boost::shared_ptr<std::vector<State> >& frame_vector);
        void changeRobotDirectionIfNeeded(RobotState& state, 
                                          int current_destination, int to_destination);

      private:

        /* State space cache */
        std::map<int, std::vector<int> > adjacent_vertices_map_;
        std::map<int, std::vector<int> > visible_vertices_map_;
        std::map<int, std::vector<int> > action_vertices_map_;

        /* Actions */
        bool isTerminalState(const State& state) const;

        /* Helper Functions */
        float getTrueDistanceTo(RobotState& state, 
                                int current_destination, int to_destination, 
                                bool change_robot_state = false);
        int generateNewGoalFrom(int idx, boost::shared_ptr<RNG> &rng);

        /* Action generation caching */
        State get_action_state_;
        std::vector<Action> get_actions_;
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
        unsigned int num_vertices_;
        int max_robots_in_use_;
        float utility_multiplier_;
        bool use_shaping_reward_;
        bool discourage_bad_assignments_;
        size_t goal_idx_;
        bool allow_goal_visibility_;
        float human_speed_;
        float robot_speed_;

        boost::shared_ptr<std::vector<State> > frame_vector_;

    };
  } /* mrn */
  
} /* bwi_guidance_solver */

BOOST_CLASS_TRACKING(bwi_guidance_solver::mrn::PersonModel, boost::serialization::track_never)

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_PERSON_MODEL_H */
