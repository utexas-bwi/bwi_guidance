#ifndef STRUCTURES_HDY3OBT2
#define STRUCTURES_HDY3OBT2

#include <stdint.h>
#include <cstddef>
#include <ostream>

#include <bwi_guidance_solver/common.h>
#include <bwi_mapper/graph.h>

namespace boost {
  namespace serialization {
    class access;
  }
}

namespace bwi_guidance_solver {

  namespace irm {
  
    /* Actions */

    enum ActionType {
      DO_NOTHING = 0,
      DIRECT_PERSON = 1,
      PLACE_ROBOT = 2
    };

    struct Action {
      Action();
      Action(ActionType a, size_t g);
      ActionType type;
      size_t graph_id; // with PLACE ROBOT, identifies the direction pointed to

      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
          ar & type;
          ar & graph_id;
        }
    };

    bool operator==(const Action& l, const Action& r);
    bool operator<(const Action& l, const Action& r);
    std::ostream& operator<<(std::ostream& stream, const Action& a);

    /* States */

    struct State {
      int graph_id; // ~50
      int direction; // ~8 
      int num_robots_left; // ~6
      int robot_direction; // ~8
      int visible_robot; // ~10

      friend class boost::serialization::access;
      template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
          ar & graph_id;
          ar & direction;
          ar & num_robots_left;
          ar & robot_direction;
          ar & visible_robot;
        }
    };

    bool operator<(const State& l, const State& r); 
    bool operator==(const State& l, const State& r);
    std::ostream& operator<<(std::ostream& stream, const State& s);

    struct StateHash { 
      StateHash() {}
      size_t operator()(const State& key) const {
        size_t seed = 0;
        boost::hash_combine(seed, key.graph_id);
        boost::hash_combine(seed, key.direction);
        boost::hash_combine(seed, key.num_robots_left);
        boost::hash_combine(seed, key.robot_direction);
        boost::hash_combine(seed, key.visible_robot);
        return seed;
      }
    }; 

  } /* irm - InstantaneousRobotMotion */

} /* bwi_guidance_solver */

#endif /* end of include guard: STRUCTURES_HDY3OBT2 */
