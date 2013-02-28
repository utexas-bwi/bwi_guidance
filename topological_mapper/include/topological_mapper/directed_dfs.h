/**
 * \file  directed_dfs.h
 * \brief  A specific implementation of Directed DFS (DFS with a priority queue)
 *         to find whether 2 points are close in obstacle space. Priority is 
 *         done on using the Euclidean distance to the goal as a heuristic.
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2013, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 02/27/2013 04:28:26 PM piyushk $
 *
 **/

namespace topological_mapper {

  /**
   * \class DirectedDFS
   * \brief The directed DFS class. Checks whether 2 points are close inside the
   *        map in obstacle space. Uses euclidean distance to goal to guide the
   *        search
   */
  class DirectedDFS {

    public:
      
      /**
       * \brief   Constructor initializing underlying map
       * \param   map OccupancyGrid representing underlying map 
       */
      DirectedDFS(const nav_msgs::OccupancyGrid& map) : map_(map) {}

      
      /**
       * \brief   Non-recusrive start point for performing DFS
       * \param   depth DFS attempts an 8 connected search. This is the max 
       *          pixel depth that we perform the search to.
       * \return  bool true if path found within depth, false otherwise
       */
      bool searchForPath(const Point2d &start, const Point2d &goal, 
          uint32_t depth);

    private:

      /**
       * \brief   Recusrive function performing DFS
       * \param   depth DFS attempts an 8 connected search. This is the max 
       *          pixel depth that we perform the search to.
       * \return  bool true if path found within depth, false otherwise
       */
      bool searchForPath(const Point2d &start, const Point2d &goal, uint32_t depth, std::vector<bool> &visited) {

        //std::cout << start.x << " " << start.y << std::endl;

        // Termination crit
        if (start.x == goal.x && start.y == goal.y) {
          return true;
        }
        if (depth == 0) {
          return false;
        }

        uint32_t start_idx = MAP_IDX(map_.info.width, start.x, start.y);
        visited[start_idx] = true;

        std::vector<Point2d> neighbours;
        getOrderedNeighbours(start, goal, visited, neighbours);
        for (size_t i = 0; i < neighbours.size(); ++i) {
          Point2d& n = neighbours[i];
          // Check if it has been visited again - quite likely that one of the previous loop iterations have covered this already
          uint32_t n_idx = MAP_IDX(map_.info.width, n.x, n.y);
          if (visited[n_idx]) {
            continue;
          }
          bool success = searchForPath(n, goal, depth - 1, visited);
          if (success)
            return true;
        }

        return false; // disconnected components
      }

      void getOrderedNeighbours(const Point2d &from, const Point2d &goal, const std::vector<bool> &visited, std::vector<Point2d> &neighbours) {
        size_t neighbour_count = 8;
        int32_t x_offset[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        int32_t y_offset[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        neighbours.clear();
        for (size_t i = 0; i < neighbour_count; ++i) {
          // Check if neighbours are still on map
          Point2d p;
          p.x = (int)from.x + x_offset[i];
          p.y = (int)from.y + y_offset[i];
          //std::cout << " " << p.x << " " << p.y << std::endl;
          if (p.x >= map_.info.width || p.y >= map_.info.height) { //covers negative case as well (unsigned)
            continue;
          }
          uint32_t map_idx = MAP_IDX(map_.info.width, p.x, p.y);
          if (visited[map_idx] || map_.data[map_idx] == 0) {
            //std::cout << " neighbour " << p.x << " " << p.y << " thrown: " << visited[map_idx] << std::endl;
            continue;
          }
          p.distance_from_ref = sqrt((p.x - goal.x)*(p.x - goal.x) + (p.y - goal.y)*(p.y - goal.y));
          neighbours.push_back(p);
          //std::cout << "  " << p.x << " " << p.y << std::endl;
        }
        std::sort(neighbours.begin(), neighbours.end(), point2dDistanceComp);
      }

      const nav_msgs::OccupancyGrid& map_;

  };

} /* topological_mapper */
