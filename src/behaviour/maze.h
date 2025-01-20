/******************************************************************************
 * Project: mazerunner-core                                                   *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#pragma once
/***
 * The Maze class holds the map of the maze as the state of all four
 * walls in each cell.
 *
 * When looking for exits, you can set a view mask. The mask is:
 *      MASK_OPEN => unseen walls are treated as exits
 *    MASK_CLOSED => unseen walls are treated as walls
 *
 * Set the mask to OPEN while searching and CLOSED when calculating a speed run
 *
 * A cell is considered to have been visited if all of its walls have been seen.
 *
 * There are a number of supporting structures and types. These are described below.
 *
 */

#include <stdint.h>
#include "common/queue.h"
#include "config.h"

// #include "vt100.h"

#define START Location(0, 0)

#define AHEAD 0
#define RIGHT 1
#define BEHIND 2
#define LEFT 3

//// Set up all the data for a 32x32 maze
//// but allow a 16x16 section to be used if needed
#define MAZE_WIDTH (16)
#define CLASSIC_MAZE_WIDTH (16)

#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_WIDTH)
// #define NUMBER_OF_HORIZ_WALLS (MAZE_WIDTH)
// #define NUMBER_OF_HORIZ_WALL_ROWS (MAZE_WIDTH + 1)
// #define NUMBER_OF_VERT_WALLS (MAZE_WIDTH + 1)
// #define NUMBER_OF_VERT_WALL_ROWS (MAZE_WIDTH)
// #define DELTA_NORTH (2 * NUMBER_OF_HORIZ_WALLS + 1)
#define NODE_COUNT ((NUMBER_OF_HORIZ_WALLS * NUMBER_OF_HORIZ_WALL_ROWS) + (NUMBER_OF_VERT_WALLS * NUMBER_OF_VERT_WALL_ROWS))

// New Constants
enum Direction {
  DIR_N = 0,
  DIR_NE = 1,
  DIR_E = 2,
  DIR_SE = 3,
  DIR_S = 4,
  DIR_SW = 5,
  DIR_W = 6,
  DIR_NW = 7,
  DIR_COUNT = 8,
  DIR_NONE = 8,  /// used for finding the node at the cell centre
  DIR_BLOCKED
};

const Direction ortho_directions[] = {DIR_N, DIR_E, DIR_S, DIR_W};
const char orthoDirChar[] = "NnEeSsWw?";
const char *moveNames[] = {
    "FWD01", "-----", "SS90R", "-----", "IP180", "-----", "SS90E", "-----",
};

/***
 * Walls exist in the map in one of four states and so get recorded using
 * two bits in the map. There are various schemes for this but here the
 * state is held in a single entity for each wall. Symbolic names for each
 * state are listed in the WallState enum.
 *
 */
enum WallState {
  EXIT = 0,     // a wall that has been seen and confirmed absent
  WALL = 1,     // a wall that has been seen and confirmed present
  UNKNOWN = 2,  // a wall that has not yet been seen
  VIRTUAL = 3,  // a wall that has not yet been seen
};

//***************************************************************************//
/***
 * The state of all four walls in a cell are stored as a structure in a single
 * variable.
 *
 * Note that GCC for AVR and STM32 (and probably most other) targets should
 * recognise this as representing a single byte in memory but that is not
 * guaranteed.
 */
struct CellWalls {
  WallState north : 2;
  WallState east : 2;
  WallState south : 2;
  WallState west : 2;
};

/***
 * Since maze wall state can have one of four values, the MazeMask
 * is used to let you use or ignore the fact that a wall has been seen.
 *
 * When the mask is set to MASK_OPEN, any unseen walls are treated as
 * being absent. Setting it to MASK_CLOSED treats unseen walls as present.
 *
 * Practically, this means that fast runs should be calculated after flooding
 * the maze with the MASK_CLOSE option so that the route never passes through
 * unseen walls or cels.
 *
 * Searching should be performed with the MASK_OPEN option so that the flood_manhattan
 * uses all the cels.
 */
enum MazeMask {
  MASK_OPEN = 0x01,    // open maze for search
  MASK_CLOSED = 0x03,  // closed maze for fast run
};

//***************************************************************************//

inline bool is_diagonal(Direction d) {
  return (d & 0x01) == 1;
}

inline Direction ahead_from(const Direction direction) {
  return direction;
}

inline Direction behind_from(const Direction direction) {
  return static_cast<Direction>((direction + DIR_COUNT / 2) % DIR_COUNT);
}
inline Direction left_from(const Direction direction) {
  return static_cast<Direction>((direction + DIR_COUNT - 2) % DIR_COUNT);
}
inline Direction right_from(const Direction direction) {
  return static_cast<Direction>((direction + DIR_COUNT + 2) % DIR_COUNT);
}

//***************************************************************************//

/***
 * Location stores a distance in the maze as an (x,y) coordinate pair. This is
 * probably a more intuitive representation than storing just a single index
 * into and array. The method takes a little more code to manage than historical
 * methods but they were devised for processors with very little program memory.
 *
 * This is a example of more processor power making is easier to represent data
 * in a more user-friendly manner.
 *
 * Locations have a number of supporting operations collected into the struct so
 * that you don't have to keep re-writing the same bits of code.
 *
 */
class Location {
 public:
  uint8_t x;
  uint8_t y;

  Location()
      : x(0),
        y(0) {};
  Location(uint8_t ix, uint8_t iy)
      : x(ix),
        y(iy) {};

  bool operator==(const Location &obj) const {
    return x == obj.x && y == obj.y;
  }

  bool operator!=(const Location &obj) const {
    return x != obj.x || y != obj.y;
  }

  // these operators prevent the user from exceeding the bounds of the maze
  // by wrapping to the opposite edge
  Location north() const {
    return Location(x, (y + 1) % MAZE_WIDTH);
  }

  Location east() const {
    return Location((x + 1) % MAZE_WIDTH, y);
  }

  Location south() const {
    return Location(x, (y + MAZE_WIDTH - 1) % MAZE_WIDTH);
  }

  Location west() const {
    return Location((x + MAZE_WIDTH - 1) % MAZE_WIDTH, y);
  }

  Location neighbour(const Direction heading) const {
    switch (heading) {
      case DIR_N:
        return north();
      case DIR_E:
        return east();
      case DIR_S:
        return south();
      case DIR_W:
        return west();
      default:
        return *this;  // this is actually an error and should be handled
    }
  }
};

//***************************************************************************//

/***
 * The Maze class is the heart of the micromouse data.
 *
 * In this basic version, the maze has a single goal location even though all
 * micromouse mazes can have a rectangular region that defines a goal area.
 *
 * The two main data blocks in the class store the wall state of every cell
 * and a cost associated with every cell after the maze is flooded.
 *
 * To keep code simple, you will note that each wall is stored twice - once
 * as seen from each side. The methods in the maze take care to ensure that
 * changes to a wall are recorded in both associated cells.
 *
 * Before flooding the maze, take care to set the maze mask as appropriate.
 * See the description above for details of the mask.
 *
 * When the robot searches and updates the map, you should call the method
 * update_wall_state() to record changes. The similar-looking method
 * set_wall_state() is private for a reason. It is unconditional and may
 * result in walls being changed after they were frst seen.
 *
 */

class Maze {
 public:
  enum PrintStyle { PRINT_AS_CDECL, PRINT_WITH_COSTS, PRINT_WALLS_ONLY };
  Maze()
      : m_goal(GOAL) {
    set_width(16);
  }

  Location goal() const {
    return m_goal;
  }

  void set_width(int width) {
    m_width = width;
  }
  /// @brief  changes the default goal. For example in a practice maze
  void set_goal(const Location goal) {
    m_goal = goal;
  }
  void set_goal(const int x, const int y) {
    m_goal = Location(x, y);
  }

  /// @brief  return the state of the walls in a cell
  CellWalls walls(const int x, const int y) const {
    return m_walls[x][y];
  }

  CellWalls walls(const Location cell) const {
    return walls(cell.x, cell.y);
  }

  /// @brief return true if ANY walls in a cell have NOT been seen
  bool has_unknown_walls(const int x, const int y) const {
    CellWalls walls_here = walls(x, y);
    if (walls_here.north == UNKNOWN || walls_here.east == UNKNOWN || walls_here.south == UNKNOWN || walls_here.west == UNKNOWN) {
      return true;
    } else {
      return false;
    }
  }

  bool has_unknown_walls(const Location cell) const {
    return has_unknown_walls(cell.x, cell.y);
  }

  WallState wall_state(const int x, const int y, const Direction heading) const {
    CellWalls walls_here = walls(x, y);
    switch (heading) {
      case DIR_N:
        return walls_here.north;
        break;
      case DIR_E:
        return walls_here.east;
        break;
      case DIR_S:
        return walls_here.south;
        break;
      case DIR_W:
        return walls_here.west;
        break;
      default:
        //// TODO: this is is an error we should probably capture
        return WallState::VIRTUAL;
        break;
    }
  }

  /// @brief  Use the current mask to test if a given wall is an exit
  /// if mask is MASK_CLOSED, we also get the UNSEEN bit
  bool is_exit(const int x, const int y, const Direction heading) const {
    CellWalls walls_here = walls(x, y);
    return isExit(walls_here, heading);
  }

  bool is_exit(const Location cell, const Direction heading) const {
    return is_exit(cell.x, cell.y, heading);
  }

  bool isExit(CellWalls walls, Direction dir) const {
    bool result = false;
    MazeMask mask;
    {
      mask = m_mask;
    }
    switch (dir) {
      case DIR_N:
        result = ((uint8_t)walls.north & (uint8_t)mask) == EXIT;
        break;
      case DIR_E:
        result = ((uint8_t)walls.east & (uint8_t)mask) == EXIT;
        break;
      case DIR_S:
        result = ((uint8_t)walls.south & (uint8_t)mask) == EXIT;
        break;
      case DIR_W:
        result = ((uint8_t)walls.west & (uint8_t)mask) == EXIT;
        break;
      default:
        result = false;
        break;
    }
    return result;
  }

  /**
   * Returns true if there is a real, mapped exit. The mask is not used/
   * @param walls - The wall data for the cell
   * @param dir  - one of the cardinal directions
   * @return
   */
  bool isMappedExit(CellWalls walls, Direction dir) const {
    bool result = false;
    switch (dir) {
      case DIR_N:
        result = walls.north == EXIT;
        break;
      case DIR_E:
        result = walls.east == EXIT;
        break;
      case DIR_S:
        result = walls.south == EXIT;
        break;
      case DIR_W:
        result = walls.west == EXIT;
        break;
      default:
        result = false;
        break;
    }
    return result;
  }

  bool is_unknown(Location loc, const Direction heading) const {
    return is_unknown(loc.x, loc.y, heading);
  }
  bool is_unknown(const int x, const int y, const Direction heading) const {
    bool result = false;
    CellWalls walls_here = walls(x, y);
    switch (heading) {
      case DIR_N:
        if ((walls_here.north & UNKNOWN) == UNKNOWN) {
          result = true;
        }
        break;
      case DIR_E:
        if ((walls_here.east & UNKNOWN) == UNKNOWN) {
          result = true;
        }
        break;
      case DIR_W:
        if ((walls_here.west & UNKNOWN) == UNKNOWN) {
          result = true;
        }
        break;
      case DIR_S:
        if ((walls_here.south & UNKNOWN) == UNKNOWN) {
          result = true;
        }
        break;
      default:

        break;
    }
    return result;
  }
  /// @brief only change a wall if it is unknown
  // This is what you use when exploring. Once seen, a wall should not be changed again.
  void update_wall_state(const int x, int y, const Direction heading, const WallState state) {
    if (is_unknown(x, y, heading)) {
      set_wall_state(Location(x, y), heading, state);
    }
  }

  void update_wall_state(const Location cell, const Direction heading, const WallState state) {
    update_wall_state(cell.x, cell.y, heading, state);
  }

  /// @brief set empty maze with border walls and the start cell, zero costs
  void initialise() {
    for (int x = 0; x < m_width; x++) {
      for (int y = 0; y < m_width; y++) {
        m_walls[x][y].north = UNKNOWN;
        m_walls[x][y].east = UNKNOWN;
        m_walls[x][y].south = UNKNOWN;
        m_walls[x][y].west = UNKNOWN;
      }
    }
    for (int x = 0; x < m_width; x++) {
      m_walls[x][0].south = WALL;
      m_walls[x][m_width - 1].north = WALL;
    }
    for (int y = 0; y < m_width; y++) {
      m_walls[0][y].west = WALL;
      m_walls[m_width - 1][y].east = WALL;
    }
    //    set_wall_state(START, DIR_N, EXIT);

    // the open maze treats unknowns as exits
    set_mask(MASK_OPEN);
    set_goal(GOAL);
    //    flood_manhattan(goal());
  }

  void set_mask(const MazeMask mask) {
    m_mask = mask;
  }

  MazeMask get_mask() const {
    return m_mask;
  }

  /// @brief return cost for neighbour cell in supplied heading
  uint16_t neighbour_cost(const Location cell, const Direction heading) const {
    if (not is_exit(cell, heading)) {
      return UINT16_MAX;
    }
    Location next_cell = cell.neighbour(heading);
    return cost(next_cell);
  }

  /// @brief  return the cost associated withthe supplied cell location
  uint16_t cost(const Location cell) const {
    return cost(cell.x, cell.y);
  }

  uint16_t cost(const int x, const int y) const {
    return m_cost[x][y];
  }

  void setCost(const int x, const int y, const uint16_t cost) {
    m_cost[x][y] = cost;
  }

  void setCost(Location cell, const uint16_t cost) {
    setCost(cell.x, cell.y, cost);
  }

  /***
   * @brief basic manhattan flood_manhattan of the maze
   *
   * Very simple cell counting flood_manhattan fills m_cost array with the
   * manhattan distance from every cell to the target.
   *
   * Although the queue looks complicated, this is a fast flood_manhattan that
   * examines each accessible cell exactly once. Consequently, it runs
   * in fairly constant time, taking 5.3ms when there are no interrupts.
   *
   * @param target - the cell from which all distances are calculated
   */

  int flood_manhattan(const Location target) {
    for (int x = 0; x < m_width; x++) {
      for (int y = 0; y < m_width; y++) {
        setCost(x, y, UINT16_MAX);
      }
    }
    /***
     * When the maze is being flooded, there is a queue of 'frontier'
     * cells. These are the cells that are waiting to be checked for
     * neighbours. I believe the maximum size that this queue can
     * possibly be for a classic maze is 64 (MAZE_CELL_COUNT/4) cells.
     * HOWEVER, this is unproven
     */
    const int QUEUE_LENGTH = MAZE_CELL_COUNT / 4;
    int max_length = 0;
    Queue<Location> queue(QUEUE_LENGTH);
    setCost(target, 0);
    queue.add(target);
    while (queue.size() > 0) {
      max_length = std::max(max_length, queue.size());
      Location here = queue.head();
      uint16_t newCost = cost(here) + 1;
      //      int exit_count = 0;
      for (auto &dir : ortho_directions) {
        if (is_exit(here, dir)) {
          Location nextCell = here.neighbour(dir);
          if (cost(nextCell) > newCost) {
            setCost(nextCell, newCost);
            queue.add(nextCell);
          }
          //          exit_count++;
        }
      }
    }
    return cost(0, 0);
  }

  /***
   * Algorithm looks around the current cell and records the smallest
   * neighbour and its direction. By starting with the supplied direction,
   * then looking right, then left, the result will preferentially be
   * ahead if there are multiple neighbours with the same m_cost.
   *
   * This could be extended to look ahead then towards the goal but it
   * probably is not worth the effort
   * @brief get the geating to the lowest cost neighbour
   * @param cell
   * @param start_heading
   * @return
   */
  Direction direction_to_smallest(const Location cell, const Direction start_heading) const {
    Direction next_heading = start_heading;
    Direction best_heading = DIR_BLOCKED;
    uint16_t best_cost = UINT16_MAX;  // cost(cell);
    uint16_t cost;
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost) {
      best_cost = cost;
      best_heading = next_heading;
    }
    next_heading = right_from(start_heading);
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost) {
      best_cost = cost;
      best_heading = next_heading;
    }
    next_heading = left_from(start_heading);
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost) {
      best_cost = cost;
      best_heading = next_heading;
    }
    next_heading = behind_from(start_heading);
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost) {
      best_cost = cost;
      best_heading = next_heading;
    }
    if (best_cost == UINT16_MAX) {
      best_heading = DIR_BLOCKED;
    }
    return best_heading;
  }

  int getWidth() {
    return m_width;
  }

  CellWalls *getMazeData() {
    return (CellWalls *)m_walls;
  }

  void print_walls() {
    int y, x;
    printf("\n");
    int width = getWidth();
    for (y = width - 1; y >= 0; y--) {
      for (x = 0; x < width; x++) {
        printf("o");
        switch (wall_state(x, y, DIR_N)) {
          case EXIT:
            printf("   ");
            break;
          case WALL:
            printf("---");
            break;
          case UNKNOWN:
            printf(" · ");
            break;
          case VIRTUAL:
            printf("···");
            break;
        }
      }
      printf("o\n");
      for (x = 0; x < width; x++) {
        switch (wall_state(x, y, DIR_W)) {
          case EXIT:
            printf("    ");
            break;
          case WALL:
            printf("|   ");
            break;
          case UNKNOWN:
            printf("·   ");
            break;
          case VIRTUAL:
            printf(":   ");
            break;
        }
      }
      printf("|\n");
    }
    // all the south border
    for (x = 0; x < width; x++) {
      printf("o---");
    }
    printf("o\n");
    printf("\n");
    printf("Goal: [%d,%d]\n", goal().x, goal().y);
  }

  void print(Maze::PrintStyle style) {
    int y, x;
    int width = getWidth();
    // flood (GOAL, unknownsAreClear);
    printf("\n");
    if (style == PRINT_AS_CDECL) {
      printf("const uint8_t _maze[] = {\n");
      for (x = 0; x < width; x++) {
        printf("   ");
        for (y = 0; y < width; y++) {
          printf("0x%02X, ", walls_as_uint8(x, y));
        }
        printf("\n");
      }
      printf("   };\n\n");
      return;
    }
    for (y = width - 1; y >= 0; y--) {
      for (x = 0; x < width; x++) {
        if (not is_exit(x, y, DIR_N)) {
          printf("o---");
        } else {
          printf("o   ");
        }
      }
      printf("o\n");
      for (x = 0; x < width; x++) {
        if (not is_exit(x, y, DIR_W)) {
          printf("|");
        } else {
          printf(" ");
        }
        // contents
        switch (style) {
          case PRINT_WITH_COSTS:
            if (cost(x, y) == UINT16_MAX) {
              printf(" x ");
            } else {
              printf("%3d", cost(x, y));
            }
            break;
          default:
            printf("   ");
            break;
        }
      }
      if (not is_exit(width - 1, y, DIR_E)) {
        printf("|\n");
      } else {
        printf(" \n\n");
      }
    }
    // all the south walls
    for (x = 0; x < width; x++) {
      if (is_exit(x, 0, DIR_S)) {
        printf("o   ");
      } else {
        printf("o---");
      }
    }
    printf("o\n");
  }

 private:
  void set_wall_state(const Location loc, const Direction heading, const WallState state) {
    switch (heading) {
      case DIR_N:
        m_walls[loc.x][loc.y].north = state;
        m_walls[loc.north().x][loc.north().y].south = state;
        break;
      case DIR_E:
        m_walls[loc.x][loc.y].east = state;
        m_walls[loc.east().x][loc.east().y].west = state;
        break;
      case DIR_W:
        m_walls[loc.x][loc.y].west = state;
        m_walls[loc.west().x][loc.west().y].east = state;
        break;
      case DIR_S:
        m_walls[loc.x][loc.y].south = state;
        m_walls[loc.south().x][loc.south().y].north = state;
        break;
      default:
        // ignore any other heading (blocked)
        break;
    }
  }

  uint8_t walls_as_uint8(const int x, const int y) const {
    uint8_t wall_data = 0x00;
    CellWalls cell_walls = walls(x, y);
    wall_data |= isExit(cell_walls, DIR_N) ? 0 : 0x01;
    wall_data |= isExit(cell_walls, DIR_E) ? 0 : 0x02;
    wall_data |= isExit(cell_walls, DIR_S) ? 0 : 0x04;
    wall_data |= isExit(cell_walls, DIR_W) ? 0 : 0x08;
    return wall_data;
  }

  MazeMask m_mask = MASK_OPEN;
  Location m_goal;
  int m_width = MAZE_WIDTH;
  uint16_t m_cost[MAZE_WIDTH][MAZE_WIDTH];
  CellWalls m_walls[MAZE_WIDTH][MAZE_WIDTH];
#ifdef ARES
  mutable std::mutex m_maze_mutex;  // used for thread safe access
#endif
};
