//
// Created by peter on 19/11/2024.
//

#pragma once

#include <SFML/Graphics.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include "behaviour/maze.h"
#include "common/core.h"
#include "configuration.h"
#include "drawing.h"
#include "world/mazedata.h"

/***
 * The types represent a bit mask where the LSB is present
 * and the other bits represent the class.
 *
 *    x x x
 *    | | `- 0 = absent, 1 = present
 *    | `--- 0 = observed, 1 = unseen
 *    `----- 1 => state taken from mouse map
 */

enum WallType : uint8_t {

  WT_MappedAbsent = 0x00,   // absent  - from mouse map
  WT_MappedPresent = 0x01,  // present - from mouse map
  WT_MappedUnknown = 0x02,  // unknown - from mouse map
  WT_MappedVirtual = 0x03,  // virtual - from mouse map

  WT_Absent = 0x04,   //  absent  - from world maze
  WT_Present = 0x05,  //  present - from world maze
  WT_Unknown = 0x06,  //  unknown - from world maze - should never happen
  WT_Virtual = 0x07,  //  virtual - from world maze - should never happen
};

inline const sf::Color WallColours[] = {
    {sf::Color(0, 0, 0, 255)},      //
    {sf::Color(255, 0, 0, 255)},    //
    {sf::Color(64, 32, 64, 128)},   //
    {sf::Color(255, 0, 255, 128)},  //

    {sf::Color(20, 20, 20, 64)},   //
    {sf::Color(128, 0, 0, 64)},    //
    {sf::Color(0, 128, 128, 64)},  //
    {sf::Color(0, 128, 128, 64)},  //

};

struct Wall {
  WallType state = WallType::WT_MappedUnknown;
  sf::RectangleShape shape;
};

/***
 * The MazeManager class looks after the maze for the application, not for the robot or its controller.
 * How they do that is their concern.
 *
 * However, the method used for storing each wall once is the basis of Harjit's diagonal solver.
 * Each Wall can be located uniquely by the method getWallIndex(x,y,Direction).
 *
 * Note that, because walls represent the boundary between two cells, there is no direct reverse
 * mapping of wall index to cells.
 * TODO: Create a README about the method and the arrangement of walls
 *
 * MazeManager will load, save, print and draw a maze to te screen. It stores the actual real-world maze
 * that the simulated robot explored. Each wall has one of the states shown above and a sf::RectangleShape
 * describing its position in the window. This assumes one pixel per mm. The view can be scaled to fit
 * the entire maze in a more manageable screen size.
 *
 * The rectangles and associated VertexArray entries for the walls and posts are pre-computed for
 * when the manager is instantiated. This speeds up the maze rendering and the generation of the
 * obstacle lists. The vertex arrays also take care of the fact that the widow coordinate system
 * starts top left while the world coordinate system starts bottom left.
 *
 * The reason for storing the posts is so that the sensor emulation and collision detection can
 * work on a more limited set obstacles around the current mouse location rather than test
 * against all 4000+ wall and post location in a 32x32 maze.
 *
 */

class MazeManager {
 public:
  MazeManager()
      : m_cells_vertex_array(sf::Quads),  //
        m_posts_vertex_array(sf::Quads),
        m_walls_vertex_array(sf::Quads)  //
  {
    resize(16);

    //    createWallGeometry();  //
    //    createPostGeometry();
    //    initialiseWallStates();
  };

  ~MazeManager() = default;

  /**
   * Change the dimensions of the maze. This is a destructive operation
   * and all wall and geometry data will be lost and recalculated
   *
   * @param new_width - mazes are always square
   */
  void resize(int new_width) {
    m_maze_width = new_width;
    m_wall_count = (new_width + 1) * new_width + (new_width + 1) * new_width;
    m_post_count = (new_width + 1) * (new_width + 1);
    const float ClassicWall = 12.0f;
    const float ClassicCell = 180.0f;

    float Scale = 16.0f / (float)new_width;
    m_wall_thickness = ClassicWall * Scale;
    m_wall_length = (ClassicCell - ClassicWall) * Scale;
    m_cell_size = ClassicCell * Scale;
    m_maze_base_size = (float)new_width * ClassicCell * Scale + m_wall_length;
    m_maze_base_rectangle.setSize({m_maze_base_size, m_maze_base_size});
    m_maze_base_rectangle.setPosition(0.0f, 0.0f);
    m_maze_base_rectangle.setFillColor(conf::MazeBaseColour);

    m_wall_states.resize(m_wall_count, WallType::WT_MappedAbsent);
    m_cell_rectangles.resize(m_maze_width * m_maze_width);
    m_wall_rectangles.resize(m_wall_count);
    m_post_rectangles.resize(m_post_count);
    m_cells_vertex_array.resize(m_maze_width * m_maze_width * 4);
    m_walls_vertex_array.resize(m_wall_count * 4);
    m_posts_vertex_array.resize(m_post_count * 4);

    createCellGeometry();
    createPostGeometry();
    createWallGeometry();  //
    initialiseWallStates();
  }

  /***
   * Mazes stored in files are assumed to be in the common text format like:
   *
   * o---o---o---o
   * |
   * o   o---o   o
   * |       |
   * o---o   o---o
   * |
   * o---o---o---o
   *
   * The entire file is read into one string and then parsed separately
   */
  bool loadMazeStringFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
      std::cerr << "Error: Could not open file " << filename << std::endl;
      return false;
    }

    std::ostringstream ss;
    ss << file.rdbuf();             // Read the file contents into the stringstream
    m_maze_data_string = ss.str();  // Store it in the mazeData string
    return true;
  }

  /***
   * Mazes held in memory are assumed to consist of an array of bytes, one per cell. The
   * wall information is held in the lower four bits of each byte. The other four bits
   * can be used to denote special features of the cell. For example, the Goal cell might
   * have the most significant bit set. The bits are arranged thus:
   *
   *     G---WESN
   *     00001001 -> a normal cell with walls to the North and West
   *     10001010 -> A cell in the goal area with walls to the South and West
   *
   * TODO: Implement the special features
   */
  bool loadFromMemory(const uint8_t* data, int mazeWidth) {
    resize(mazeWidth);
    initialiseWallStates();
    for (int y = 0; y < mazeWidth; y++) {
      for (int x = 0; x < mazeWidth; x++) {
        int index = x * mazeWidth + y;
        int walls = data[index];
        for (int d = 0; d < 4; d++) {
          // we need only do the North and East walls
          if (walls & BIT(0)) {
            setWallState(x, y, Direction::DIR_N, WallType::WT_Present);
          }
          if (walls & BIT(1)) {
            setWallState(x, y, Direction::DIR_E, WallType::WT_Present);
          }
        }
      }
    }
    if (data[0] & BIT(1)) {
      setWallState(0, 0, Direction::DIR_E, WallType::WT_Present);
    }
    return true;
  }

  /***
   * Similar to loadFromMemory except that  the wall states get updated from
   * the map data held by the mouse. To do this, we leave the world data
   * bit set
   *
   * TODO: This is pretty horrible. Sort it out.
   * @param data
   * @param mazeWidth
   * @return
   */
  bool updateFromMap(Maze& maze, int mazeWidth) {
    for (int y = 0; y < mazeWidth; y++) {
      for (int x = 0; x < mazeWidth; x++) {
        if (maze.wall_state(x, y, DIR_N) == EXIT) {
          setWallState(x, y, Direction::DIR_N, WT_MappedAbsent);
        }
        if (maze.wall_state(x, y, DIR_N) == WALL) {
          setWallState(x, y, Direction::DIR_N, WT_MappedPresent);
        }
        if (maze.wall_state(x, y, DIR_E) == EXIT) {
          setWallState(x, y, Direction::DIR_E, WT_MappedAbsent);
        }
        if (maze.wall_state(x, y, DIR_E) == WALL) {
          setWallState(x, y, Direction::DIR_E, WT_MappedPresent);
        }
        if (maze.wall_state(x, y, DIR_S) == EXIT) {
          setWallState(x, y, Direction::DIR_S, WT_MappedAbsent);
        }
        if (maze.wall_state(x, y, DIR_S) == WALL) {
          setWallState(x, y, Direction::DIR_S, WT_MappedPresent);
        }
        if (maze.wall_state(x, y, DIR_W) == EXIT) {
          setWallState(x, y, Direction::DIR_W, WT_MappedAbsent);
        }
        if (maze.wall_state(x, y, DIR_W) == WALL) {
          setWallState(x, y, Direction::DIR_W, WT_MappedPresent);
        }
      }
    }
    return true;
  }

  /***
   * A list (array) of previous contest mazes is held in the MazeDataSource struct.
   */
  bool loadFromList(MazeDataSource& mazeData, int width) {
    return loadFromMemory(mazeData.data, width);  //
  }

  /***
   * Convert the string format used in maze files to the internal format
   */
  void LoadMazeFromString(const std::string& mazeData) {
    (void)mazeData;
    //
  }

  bool isHorizontalWall(int wall_index) {
    return ((wall_index % (m_maze_width * 2 + 1)) < m_maze_width);  //
  }

  /***
   * Walls are stored in a list. Each has a unique ID. See the docs directory
   * for an explanation of the ID calculation. The method ensures no wall
   * information is duplicated and also makes it easier to create a solver
   * and path generator that creates diagonal paths. This is because all
   * the nodes will end up being wall centres.
   *
   * @param cellX
   * @param cellY
   * @param direction
   * @return
   */
  int getWallIndex(int cellX, int cellY, Direction direction) const {
    int index = cellX + (m_maze_width * 2 + 1) * cellY;
    switch (direction) {
      case Direction::DIR_N:
        index += (m_maze_width * 2 + 1);
        break;
      case Direction::DIR_E:
        index += (m_maze_width + 1);
        break;
      case Direction::DIR_S:
        index += 0;
        break;
      case Direction::DIR_W:
        index += m_maze_width;
        break;
      default:
        index = 0;  // even though this is wrong.
        break;
    }
    return index;
  }

  /***
   * When setting the state of a wall, we also need to change the
   * colour of the wall geometry.
   *
   * Changing a wall state is a relatively infrequent operation so
   * we update the vertex array here rather than during the draw method.
   */
  void setWallState(int index, WallType state) {
    m_wall_states[index] = state;  //
    sf::Color color = WallColours[state];
    m_walls_vertex_array[index * 4 + 0].color = color;
    m_walls_vertex_array[index * 4 + 1].color = color;
    m_walls_vertex_array[index * 4 + 2].color = color;
    m_walls_vertex_array[index * 4 + 3].color = color;
  }

  void setWallState(int x, int y, Direction direction, WallType state) {
    int index = getWallIndex(x, y, direction);
    setWallState(index, state);
  }

  WallType getWallState(int x, int y, Direction direction) {
    int index = getWallIndex(x, y, direction);
    return getWallState(index);
  }

  WallType getWallState(int index) {
    return m_wall_states[index];  //
  }

  /**
   * Working in world coordinates with the origin at the bottom left
   * corner of the bottom left post, calculate the bottom left corner
   * of a cell
   */
  sf::Vector2f getCellOrigin(int x, int y) {
    return {(float)x * m_cell_size, (float)y * m_cell_size};  //
  }

  float getCellSize() {
    return m_cell_size;  //
  }

  /***
   *The base has an array or squares, one for each cell, that can be
   * coloured according to status;
   */
  void createCellGeometry() {
    for (int y = 0; y < m_maze_width; y++) {
      for (int x = 0; x < m_maze_width; x++) {
        int index = x * (m_maze_width) + y;
        sf::Vector2f position = getCellOrigin(x, y);
        position += {m_wall_thickness, m_wall_thickness};
        float width = getCellSize() - m_wall_thickness;
        sf::Vector2f size = {width, width};
        m_cell_rectangles[index] = {position, size};
        sf::Color colour = conf::MazeBaseColour;
        m_cells_vertex_array[index * 4 + 0].position = position;
        m_cells_vertex_array[index * 4 + 1].position = sf::Vector2f(position.x + size.x, position.y);
        m_cells_vertex_array[index * 4 + 2].position = sf::Vector2f(position.x + size.x, position.y + size.y);
        m_cells_vertex_array[index * 4 + 3].position = sf::Vector2f(position.x, position.y + size.y);
        m_cells_vertex_array[index * 4 + 0].color = colour;
        m_cells_vertex_array[index * 4 + 1].color = colour;
        m_cells_vertex_array[index * 4 + 2].color = colour;
        m_cells_vertex_array[index * 4 + 3].color = colour;
      }
    }
  }

  void resetCellColours() {
    for (int y = 0; y < m_maze_width; y++) {
      for (int x = 0; x < m_maze_width; x++) {
        setCellColour(x, y, conf::MazeBaseColour);
      }
    }
  }

  void setCellColour(int x, int y, sf::Color colour) {
    int index = x * (m_maze_width) + y;
    m_cells_vertex_array[index * 4 + 0].color = colour;
    m_cells_vertex_array[index * 4 + 1].color = colour;
    m_cells_vertex_array[index * 4 + 2].color = colour;
    m_cells_vertex_array[index * 4 + 3].color = colour;
  }

  /**
   * The physical maze consist of two lists of rectangles - the posts
   * and the walls. The posts are all known and fixed in place at the
   * start of the maze. The walls vary from maze to maze.
   *
   * Note that the Japanese half-size event can have enclosed areas,
   *      including the goal area, that do not have posts. Contestants
   *      may chose to deal with this by adding virtual walls to the
   *      map since they know where the goal area is. The virtual
   *      walls will prevent the mouse from straying into dangerous
   *      territory.
   *
   * The posts are static so just make them once.
   */
  void createPostGeometry() {
    for (int y = 0; y <= m_maze_width; y++) {
      for (int x = 0; x <= m_maze_width; x++) {
        float ox = (float)x * m_cell_size;
        float oy = (float)y * m_cell_size;
        int index = x * (m_maze_width + 1) + y;
        m_post_rectangles[index] = {{ox, oy}, {m_wall_thickness, m_wall_thickness}};
      }
    }

    for (int i = 0; i < m_post_count; ++i) {
      const sf::FloatRect& rect = m_post_rectangles[i];
      const sf::Vector2f& position = rect.getPosition();
      const sf::Vector2f& size = rect.getSize();
      sf::Color colour = conf::PostColour;

      m_posts_vertex_array[i * 4 + 0].position = position;
      m_posts_vertex_array[i * 4 + 1].position = sf::Vector2f(position.x + size.x, position.y);
      m_posts_vertex_array[i * 4 + 2].position = sf::Vector2f(position.x + size.x, position.y + size.y);
      m_posts_vertex_array[i * 4 + 3].position = sf::Vector2f(position.x, position.y + size.y);
      m_posts_vertex_array[i * 4 + 0].color = colour;
      m_posts_vertex_array[i * 4 + 1].color = colour;
      m_posts_vertex_array[i * 4 + 2].color = colour;
      m_posts_vertex_array[i * 4 + 3].color = colour;
    }
  }

  /***
   * Like the posts, we can create all the wall rectangles and their
   * associated vertex array entries when the maze if first created.
   * These positions do not change so the cost of creation is saved
   * on every frame.
   *
   * Walls can be set or cleared just by changing the state and
   * then applying the relevant colour. In this function, we just
   * create the geometry and set each wall as WT_WorldAbsent.
   * The caller is then responsible for setting the state of each
   * wall according to the physical maze.
   *
   * The vertex array holding the shapes will (must) have been
   * presized when the maze object is created or resized
   */
  void createWallGeometry() {
    sf::FloatRect wall_shape;
    sf::FloatRect hWall({0, 0}, {m_wall_length, m_wall_thickness});
    sf::FloatRect vWall({0, 0}, {m_wall_thickness, m_wall_length});
    for (int y = 0; y < m_maze_width; y++) {
      for (int x = 0; x < m_maze_width; x++) {
        sf::Vector2f origin = getCellOrigin(x, y);
        int index;
        sf::Vector2f offset;
        /// North Wall
        offset.x = m_wall_thickness;
        offset.y = m_cell_size;
        wall_shape = hWall;
        wall_shape.left = origin.x + offset.x;
        wall_shape.top = origin.y + offset.y;
        index = getWallIndex(x, y, Direction::DIR_N);
        m_wall_rectangles[index] = wall_shape;
        setWallState(index, WallType::WT_MappedUnknown);
        /// West Wall
        offset.x = 0;
        offset.y = m_wall_thickness;
        wall_shape = vWall;
        wall_shape.left = origin.x + offset.x;
        wall_shape.top = origin.y + offset.y;
        index = getWallIndex(x, y, Direction::DIR_W);
        m_wall_rectangles[index] = wall_shape;
        setWallState(index, WallType::WT_MappedUnknown);
      }
    }
    // now the South and East border
    sf::Vector2f origin;
    for (int i = 0; i < m_maze_width; i++) {
      int index;
      origin = getCellOrigin(i, 0);
      sf::Vector2f offset;
      // South Wall
      offset.x = m_wall_thickness;
      offset.y = 0;
      wall_shape = hWall;
      wall_shape.left = origin.x + offset.x;
      wall_shape.top = origin.y + offset.y;
      index = getWallIndex(i, 0, Direction::DIR_S);
      m_wall_rectangles[index] = wall_shape;
      setWallState(index, WallType::WT_MappedUnknown);

      // East Wall
      origin = getCellOrigin(m_maze_width - 1, i);
      offset.x = m_cell_size;
      offset.y = m_wall_thickness;
      wall_shape = vWall;
      wall_shape.left = origin.x + offset.x;
      wall_shape.top = origin.y + offset.y;
      index = getWallIndex(m_maze_width - 1, i, Direction::DIR_E);
      m_wall_rectangles[index] = wall_shape;
      setWallState(index, WallType::WT_MappedUnknown);
    }
    // Now we have the geometry, we can create the vertex array positions
    for (int i = 0; i < m_wall_count; ++i) {
      sf::FloatRect& rect = m_wall_rectangles[i];
      sf::Vector2f position = rect.getPosition();
      sf::Vector2f size = rect.getSize();
      m_walls_vertex_array[i * 4 + 0].position = position;
      m_walls_vertex_array[i * 4 + 1].position = sf::Vector2f(position.x + size.x, position.y);
      m_walls_vertex_array[i * 4 + 2].position = sf::Vector2f(position.x + size.x, position.y + size.y);
      m_walls_vertex_array[i * 4 + 3].position = sf::Vector2f(position.x, position.y + size.y);
    }
  }

  void setWallColour(int index, sf::Color colour) {
    assert(index >= 0);
    m_walls_vertex_array[index * 4 + 0].color = colour;
    m_walls_vertex_array[index * 4 + 1].color = colour;
    m_walls_vertex_array[index * 4 + 2].color = colour;
    m_walls_vertex_array[index * 4 + 3].color = colour;
  }

  void resetWallColours() {
    for (int i = 0; i < m_wall_count; i++) {
      sf::Color color = WallColours[m_wall_states[i]];
      m_walls_vertex_array[i * 4 + 0].color = color;
      m_walls_vertex_array[i * 4 + 1].color = color;
      m_walls_vertex_array[i * 4 + 2].color = color;
      m_walls_vertex_array[i * 4 + 3].color = color;
    }
  }

  /**
   * Reset all the wall states for an empty maze. Fill in the borders and the start cell.
   */
  void initialiseWallStates() {
    for (int y = 0; y < m_maze_width; y++) {
      for (int x = 0; x < m_maze_width; x++) {
        setWallState(x, y, Direction::DIR_N, WallType::WT_Absent);
        setWallState(x, y, Direction::DIR_W, WallType::WT_Absent);
      }
    }
    for (int i = 0; i < m_maze_width; i++) {
      setWallState(i, 0, Direction::DIR_S, WallType::WT_Present);
      setWallState(i, m_maze_width - 1, Direction::DIR_N, WallType::WT_Present);
      setWallState(0, i, Direction::DIR_W, WallType::WT_Present);
      setWallState(m_maze_width - 1, i, Direction::DIR_E, WallType::WT_Present);
    }
    //    setWallState(0, 0, Direction::DIR_E, WallType::WT_Present);
    setWallState(0, 0, Direction::DIR_N, WallType::WT_Absent);
  }

  void render(sf::RenderWindow& window) {
    window.draw(m_maze_base_rectangle);
    //    setWallState(40, WallType::WT_Absent);
    //    setWallState(41, WallType::WT_Present);
    //    setWallState(42, WallType::WT_Unknown);
    //    setWallState(43, WallType::WT_Virtual);
    //    setWallState(44, WallType::WT_MappedAbsent);
    //    setWallState(45, WallType::WT_MappedPresent);
    //    setWallState(46, WallType::WT_MappedUnknown);
    //    setWallState(47, WallType::WT_MappedVirtual);
    window.draw(m_walls_vertex_array);
    window.draw(m_posts_vertex_array);
    window.draw(m_cells_vertex_array);
  }

  /**
   * The obstacles are all the walls and posts that may be in range of the mouse sensors.
   *
   * While this will clearly depend on the nature of the sensors, a simple approach is taken
   * here whereby the current cell and its immediate 8 neighbours have al their walls and
   * posts added to the list. This matches typical behaviour in practical robots though
   * half-size micromouse sensors may have a greater range. In that case, the brute-force
   * approach will need some modification.
   *
   * For testing, the selected items are highlighted
   *
   * The returned list is passed to the sensor simulation method and the obstacle list is
   * normally only used internally by the maze_manager.
   *
   * NOTE: there is a performance bottleneck here and in the sensor test run by the Application
   *       Currently we return a list of up to 40 rectangles that need to be checked. Most of
   *       the time there will be fewer than that number. If, instead, only the cell bounding
   *       rectangles were returned - ideally only those that are in view - there could be
   *       a significant performance increase. That might be only four or five rectangles
   *
   * @param robot_x the current cell x coordinate
   * @param robot_y  the current cel y coordinate
   * @return  a list of rectangles representing objects in view of the sensors.
   */
  const std::vector<sf::FloatRect>& GetObstacles(float robot_x, float robot_y) {
    int x = robot_x / m_cell_size;
    int y = robot_y / m_cell_size;

    m_walls_visible.clear();
    for (int xx = x - 1; xx <= x + 1; xx++) {
      for (int yy = y - 1; yy <= y + 1; yy++) {
        addWallToSet(m_walls_visible, xx, yy, Direction::DIR_N);
        addWallToSet(m_walls_visible, xx, yy, Direction::DIR_E);
        addWallToSet(m_walls_visible, xx, yy, Direction::DIR_S);
        addWallToSet(m_walls_visible, xx, yy, Direction::DIR_W);
      }
    };

    m_obstacles.clear();

    for (auto i : m_walls_visible) {
      m_obstacles.push_back(m_wall_rectangles[i]);
    }

    m_posts_visible.clear();
    /// we just do the bottom left post for all required cells
    for (int xx = x - 1; xx <= x + 2; xx++) {
      for (int yy = y - 1; yy <= y + 2; yy++) {
        addPostToList(m_posts_visible, xx, yy);
      }
    }
    for (auto& post_index : m_posts_visible) {
      m_obstacles.push_back(m_post_rectangles[post_index]);
    }
    return m_obstacles;  //
  }

  sf::Vector2f getCellCentre(int x, int y) {
    return {(float)x * m_cell_size + m_cell_size / 2.0f + m_wall_thickness / 2.0f, (float)y * m_cell_size + m_cell_size / 2.0f + m_wall_thickness / 2.0f};
  }

  int getCellFromPosition(float x, float y) {
    int cx = x / (float)m_cell_size;
    int cy = y / (float)m_cell_size;
    return cx * m_maze_width + cy;
  }

 private:
  /// Just add the bottom left post of all the relevant cells
  void addPostToList(std::vector<int>& list, int x, int y) {
    if (x < 0 || x > m_maze_width) {
      return;
    }
    if (y < 0 || y > m_maze_width) {
      return;
    }
    int index = x * (m_maze_width + 1) + y;
    list.push_back(index);
  }

  /***
   * we use a std::unordered_set here because it automatically handles duplicates.
   */
  void addWallToSet(std::unordered_set<int>& list, int x, int y, Direction d) {
    if (x < 0 && (d == Direction::DIR_W || d == Direction::DIR_N || d == Direction::DIR_S)) {
      return;
    }
    if (x >= m_maze_width && (d == Direction::DIR_E || d == Direction::DIR_N || d == Direction::DIR_S)) {
      return;
    }
    if (y < 0 && (d == Direction::DIR_E || d == Direction::DIR_S || d == Direction::DIR_W)) {
      return;
    }
    if (y >= m_maze_width && (d == Direction::DIR_W || d == Direction::DIR_N || d == Direction::DIR_E)) {
      return;
    }

    int index = getWallIndex(x, y, d);
    /// we only want walls that are present in the world
    WallType wall = m_wall_states[index];
    if (wall == WallType::WT_Present || wall == WallType::WT_MappedPresent) {
      list.insert(index);
    }
  }

  // stuff to track edits and changes
  std::string m_maze_data_string;

  // these hold the precalculated geometry of
  // the maze
  std::vector<WallType> m_wall_states;  // the logical state of the walls in the world maze
  sf::RectangleShape m_maze_base_rectangle;
  std::vector<sf::FloatRect> m_cell_rectangles;
  std::vector<sf::FloatRect> m_post_rectangles;
  std::vector<sf::FloatRect> m_wall_rectangles;
  sf::VertexArray m_cells_vertex_array;
  sf::VertexArray m_posts_vertex_array;
  sf::VertexArray m_walls_vertex_array;

  std::vector<sf::FloatRect> m_obstacles;  // the things the robot can see or hit
  /// objects visible to the mouse sensors
  std::unordered_set<int> m_walls_visible;
  std::vector<int> m_posts_visible;

  int m_maze_width;
  float m_cell_size;
  float m_maze_base_size;
  float m_wall_length;
  float m_wall_thickness;
  int m_wall_count;
  int m_post_count;

  // clang_format on
};
