//
// Created by peter on 19/11/2024.
//

#ifndef MAZE_H
#define MAZE_H

#include <SFML/Graphics.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include "common/core.h"
#include "configuration.h"
#include "drawing.h"
#include "world/mazedata.h"

enum class WallState {
  KnownAbsent,   //
  KnownPresent,  //
  Unknown,       //
  Virtual,       //
  Mystery
};

struct Wall {
  WallState state = WallState::Unknown;
  sf::RectangleShape shape;
};

enum class Direction { North, NorthEast, East, SouthEast, South, SouthWest, West, NorthWest };

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
      : m_posts_vertex_array(sf::Quads),  //
        m_walls_vertex_array(sf::Quads)   //
  {
    resize(32);

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

    m_maze_base_rectangle.setSize({conf::MazeSize, conf::MazeSize});
    m_maze_base_rectangle.setPosition(0.0f, 0.0f);
    m_maze_base_rectangle.setFillColor(conf::MazeBaseColour);

    m_wall_states.resize(m_wall_count, WallState::Unknown);
    m_wall_rectangles.resize(m_wall_count);
    m_post_rectangles.resize(m_post_count);
    m_walls_vertex_array.resize(m_wall_count * 4);
    m_posts_vertex_array.resize(m_post_count * 4);

    createWallGeometry();  //
    createPostGeometry();
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
    for (int y = 0; y < mazeWidth - 1; y++) {
      for (int x = 0; x < mazeWidth - 1; x++) {
        int index = x * mazeWidth + y;
        int walls = data[index];
        for (int d = 0; d < 4; d++) {
          // we need only do the North and East walls
          if (walls & BIT(0)) {
            setWallState(x, y, Direction::North, WallState::KnownPresent);
          }
          if (walls & BIT(1)) {
            setWallState(x, y, Direction::East, WallState::KnownPresent);
          }
        }
      }
    }
    setWallState(0, 0, Direction::East, WallState::KnownPresent);
    return true;
  }

  /***
   * A list (array) of previous contest mazes is held in the MazeDataSource struct.
   */
  bool loadFromList(MazeDataSource& mazeData, int width) {
    return loadFromMemory(mazeData.data, width);  //
  }

  /***
   * Convert the string format used in maze fles to the internal format
   */
  void LoadMazeFromString(const std::string& mazeData) {
    (void)mazeData;
    //
  }

  bool isHorizontalWall(int wall_index) {
    return ((wall_index % (m_maze_width * 2 + 1)) < m_maze_width);  //
  }

  int getWallIndex(int cellX, int cellY, Direction direction) const {
    int index = cellX + (m_maze_width * 2 + 1) * cellY;
    switch (direction) {
      case Direction::North:
        index += (m_maze_width * 2 + 1);
        break;
      case Direction::East:
        index += (m_maze_width + 1);
        break;
      case Direction::South:
        index += 0;
        break;
      case Direction::West:
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
   * Changing a wall state if a relatively infrequent operation so
   * we update the VertexArray here rather than during the draw method.
   */
  void setWallState(int index, WallState state) {
    m_wall_states[index] = state;  //
    m_walls_vertex_array[index * 4 + 0].color = conf::WallStateColors[(int)state];
    m_walls_vertex_array[index * 4 + 1].color = conf::WallStateColors[(int)state];
    m_walls_vertex_array[index * 4 + 2].color = conf::WallStateColors[(int)state];
    m_walls_vertex_array[index * 4 + 3].color = conf::WallStateColors[(int)state];
  }

  void setWallState(int x, int y, Direction direction, WallState state) {
    int index = getWallIndex(x, y, direction);
    setWallState(index, state);
  }

  WallState getWallState(int index) {
    return m_wall_states[index];  //
  }

  WallState getWallState(int x, int y, Direction direction) {
    int index = getWallIndex(x, y, direction);
    return getWallState(index);
  }

  /***
   * Working in world coordinates with the origin at the bottom left
   * corner of the bottom left post, calculate the bottom left corner
   * of a cell
   */
  sf::Vector2f getCellOrigin(int x, int y) {
    return {(float)x * m_cell_size, (float)y * m_cell_size};  //
  }

  float getCellSize() { return m_cell_size; }
  /**
   * The posts are static so just make them once.
   */
  void createPostGeometry() {
    //    sf::RectangleShape post({m_wall_thickness, m_wall_thickness});
    for (int y = 0; y <= m_maze_width; y++) {
      for (int x = 0; x <= m_maze_width; x++) {
        float ox = (float)x * m_cell_size;
        float oy = (float)y * m_cell_size;
        int index = x * (m_maze_width + 1) + y;
        m_post_rectangles[index] = {{ox, oy}, {m_wall_thickness, m_wall_thickness}};
      }
    }
    sf::Color colour = conf::PostColour;

    for (int i = 0; i < m_post_count; ++i) {
      const sf::FloatRect& rect = m_post_rectangles[i];
      const sf::Vector2f& position = rect.getPosition();
      const sf::Vector2f& size = rect.getSize();

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

  void setPostColour(int index, sf::Color colour) {
    m_posts_vertex_array[index * 4 + 0].color = colour;
    m_posts_vertex_array[index * 4 + 1].color = colour;
    m_posts_vertex_array[index * 4 + 2].color = colour;
    m_posts_vertex_array[index * 4 + 3].color = colour;
  }

  void resetPostColours() {
    for (int i = 0; i < m_post_count; ++i) {
      m_posts_vertex_array[i * 4 + 0].color = conf::PostColour;
      m_posts_vertex_array[i * 4 + 1].color = conf::PostColour;
      m_posts_vertex_array[i * 4 + 2].color = conf::PostColour;
      m_posts_vertex_array[i * 4 + 3].color = conf::PostColour;
    }
  }

  /***
   * Like the posts, we can create all the wall rectangles and their
   * associated vertex array entries when the maze if first created.
   * These positions do not change so the cost of creation is saved
   * on every frame.
   *
   * Walls can then be set or cleared just by changing the state and
   * then applying the relevant colour
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
        // North Wall
        offset.x = m_wall_thickness;
        offset.y = m_cell_size;
        ;
        wall_shape = hWall;
        wall_shape.left = origin.x + offset.x;
        wall_shape.top = origin.y + offset.y;
        index = getWallIndex(x, y, Direction::North);
        m_wall_rectangles[index] = wall_shape;
        setWallState(index, WallState::Unknown);
        // West Wall
        offset.x = 0;
        offset.y = m_wall_thickness;
        wall_shape = vWall;
        wall_shape.left = origin.x + offset.x;
        wall_shape.top = origin.y + offset.y;
        index = getWallIndex(x, y, Direction::West);
        m_wall_rectangles[index] = wall_shape;
        setWallState(index, WallState::Unknown);
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
      index = getWallIndex(i, 0, Direction::South);
      m_wall_rectangles[index] = wall_shape;
      setWallState(index, WallState::Unknown);

      // East Wall
      origin = getCellOrigin(m_maze_width - 1, i);
      offset.x = m_cell_size;
      offset.y = m_wall_thickness;
      wall_shape = vWall;
      wall_shape.left = origin.x + offset.x;
      wall_shape.top = origin.y + offset.y;
      index = getWallIndex(m_maze_width - 1, i, Direction::East);
      m_wall_rectangles[index] = wall_shape;
      setWallState(index, WallState::Unknown);
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
    m_walls_vertex_array[index * 4 + 0].color = colour;
    m_walls_vertex_array[index * 4 + 1].color = colour;
    m_walls_vertex_array[index * 4 + 2].color = colour;
    m_walls_vertex_array[index * 4 + 3].color = colour;
  }

  void resetWallColours() {
    for (int i = 0; i < m_wall_count; i++) {
      m_walls_vertex_array[i * 4 + 0].color = conf::WallStateColors[(int)m_wall_states[i]];
      m_walls_vertex_array[i * 4 + 1].color = conf::WallStateColors[(int)m_wall_states[i]];
      m_walls_vertex_array[i * 4 + 2].color = conf::WallStateColors[(int)m_wall_states[i]];
      m_walls_vertex_array[i * 4 + 3].color = conf::WallStateColors[(int)m_wall_states[i]];
    }
  }

  void initialiseWallStates() {
    for (int y = 0; y < m_maze_width; y++) {
      for (int x = 0; x < m_maze_width; x++) {
        setWallState(x, y, Direction::North, WallState::Unknown);
        setWallState(x, y, Direction::West, WallState::Unknown);
      }
    }
    for (int i = 0; i < m_maze_width; i++) {
      setWallState(i, 0, Direction::South, WallState::KnownPresent);
      setWallState(i, m_maze_width - 1, Direction::North, WallState::KnownPresent);
      setWallState(0, i, Direction::West, WallState::KnownPresent);
      setWallState(m_maze_width - 1, i, Direction::East, WallState::KnownPresent);
    }
    setWallState(0, 0, Direction::East, WallState::KnownPresent);
    setWallState(0, 0, Direction::North, WallState::KnownAbsent);
  }

  void draw(sf::RenderWindow& window) {
    window.draw(m_maze_base_rectangle);
    window.draw(m_walls_vertex_array);
    window.draw(m_posts_vertex_array);
  }

  void updateObstacles() {
    return;
    //    std::lock_guard<std::mutex> lock(m_mutex_obstacles);
    //    m_obstacles.clear();
    //    int c = 40;
    //    for (auto& wall : m_walls) {
    //      if (wall.state == WallState::KnownPresent) {
    //        m_obstacles.push_back(wall.shape);
    //      }
    //      if (--c <= 0) {
    //        return;
    //      }
    //    }
  }

  const std::vector<sf::FloatRect>& GetObstacles(float robot_x, float robot_y) {
    m_obstacles.clear();
    int cell_x = robot_x / m_cell_size;
    int cell_y = robot_y / m_cell_size;
    std::vector<int> walls_seen;
    int start_wall = getWallIndex(cell_x, cell_y, Direction::South);  // the 'base' wall for this cell
    for (int i : conf::SensorWallOffsets) {
      int wall_index = start_wall + i;
      if (wall_index >= 0 && wall_index < m_wall_count) {
        if (getWallState(wall_index) == WallState::KnownPresent) {
          m_obstacles.push_back(m_wall_rectangles[wall_index]);
          walls_seen.push_back(wall_index);
        }
      }
    }
    if (conf::DebugHighlightTestedWalls) {
      for (auto i : walls_seen) {
        setWallColour(i, conf::WallHighlightColour);
      }
    }

    // now the posts
    int start_post = cell_x * (m_maze_width + 1) + cell_y;
    for (int i : conf::SensorPostOffsets) {
      int post_index = start_post + i;
      if (post_index >= 0 && post_index < m_post_count) {
        m_obstacles.push_back(m_post_rectangles[post_index]);
        if (conf::DebugHighlightTestedWalls) {
          setPostColour(post_index, conf::WallHighlightColour);
        }
      }
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

  sf::FloatRect getWallRect(int index) { return m_wall_rectangles[index]; }

 private:
  // stuff to track edits and changes
  std::string m_maze_data_string;
  bool m_maze_changed = false;

  // these hold the precalculated geometry of
  // the maze
  std::vector<WallState> m_wall_states;  // the logical state of the walls in the world maze
  sf::RectangleShape m_maze_base_rectangle;
  std::vector<sf::FloatRect> m_wall_rectangles;
  std::vector<sf::FloatRect> m_post_rectangles;
  sf::VertexArray m_posts_vertex_array;
  sf::VertexArray m_walls_vertex_array;

  mutable std::mutex m_mutex_obstacles;    // Protects access when building collision list
  std::vector<sf::FloatRect> m_obstacles;  // the things the robot can see or hit

  int m_maze_width;
  float m_cell_size;
  float m_maze_base_size;
  float m_wall_length;
  float m_wall_thickness;
  int m_wall_count;
  int m_post_count;
};

#endif  // MAZE_H
