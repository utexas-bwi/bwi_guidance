#ifndef BWI_GUIDANCE_SOLVER_MRN_ABSTRACT_MAPPING_H
#define BWI_GUIDANCE_SOLVER_MRN_ABSTRACT_MAPPING_H

#include <boost/shared_ptr.hpp>

#include <bwi_guidance_solver/mrn/structures.h>
#include <bwi_rl/planning/StateMapping.h>

namespace bwi_guidance_solver {

  namespace mrn {
    
    class AbstractMapping : public StateMapping<State> {

      public:
        typedef boost::shared_ptr<AbstractMapping> Ptr;

        virtual void map(State &state);
    };

  } /* mrn */
  
} /* bwi_guidance_solver */

#endif /* end of include guard: BWI_GUIDANCE_SOLVER_MRN_ABSTRACT_MAPPING_H */


