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

const int MAZE_WIDTH = (16);
const int NODES_PER_HORIZ_WALL_ROW = (MAZE_WIDTH);
const int NODES_PER_VERT_WALL_ROW = (MAZE_WIDTH - 1);
const int NUMBER_OF_HORIZ_WALLS = (NODES_PER_HORIZ_WALL_ROW);
const int NUMBER_OF_VERT_WALLS = (NODES_PER_VERT_WALL_ROW + 2);
const int NUMBER_OF_HORIZ_WALL_ROWS = (MAZE_WIDTH + 1);
const int NUMBER_OF_VERT_WALL_ROWS = (MAZE_WIDTH);
const int NUMBER_OF_WALLS = ((NUMBER_OF_HORIZ_WALLS * NUMBER_OF_HORIZ_WALL_ROWS) + (NUMBER_OF_VERT_WALLS * NUMBER_OF_VERT_WALL_ROWS));
const int NUMBER_OF_POSTS = (MAZE_WIDTH + 1) * (MAZE_WIDTH + 1);

const int LOCATION_DELTA_NORTH = (MAZE_WIDTH * 2 + 1);
const int LOCATION_DELTA_NORTH_EAST = (MAZE_WIDTH + 1);
const int LOCATION_DELTA_EAST = (1);
const int LOCATION_DELTA_SOUTH_EAST = (-MAZE_WIDTH);
const int LOCATION_DELTA_SOUTH = (-(MAZE_WIDTH * 2 + 1));
const int LOCATION_DELTA_SOUTH_WEST = (-(MAZE_WIDTH + 1));
const int LOCATION_DELTA_WEST = (-1);
const int LOCATION_DELTA_NORTH_WEST = (MAZE_WIDTH);

const float Scale = 16.0f / (float)MAZE_WIDTH;
const float WALL_THICKNESS = 12.0f * Scale;
const float WALL_LENGTH = 166.0f * Scale;
const float CELL_SIZE = 180.0 * Scale;
const float GAP = 2.0f;

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
 * TODO: Create a README about the method and the arrangement of walls
 *
 * MazeManager can load, save, print and draw a maze to te screen. It stores the actual real-world maze
 * that the simulated robot explored. Each wall has one of the states shown above and a sf::RectangleShape
 * describing its position in the window. This assumes one pixel per mm. The view can be scaled to fit
 * the entire maze in a more manageable screen size.
 *
 * A structure called Wall is used to store the state and shape of each wall.
 *
 * The rectangles are pre-computed for an entire 32x32 sized maze when the manager is instantiated.
 * All the walls states are set to unknown.
 *
 * Each Wall can be located uniquely by the method GetWallIndex(x,y,Direction).
 *
 * The MazeManager also creates and stores all the posts in the maze using the same Wall struct
 * except that these are all set to KnownPresent.
 *
 * The reason for storing the posts is so that the sensor emulation and collision detection can
 * work on a more limited set obstacles around the current mouse location rather than test
 * against all 4000+ wall and post location in a 32x32 maze.
 *
 */

class MazeManager {
 public:
  /// these constants are mostly needed for testing

  MazeManager()
      : m_vertexArrayPosts(sf::Quads, NUMBER_OF_POSTS * 4),  //
        m_vertexArrayWalls(sf::Quads, NUMBER_OF_WALLS * 4)   //
  {
    m_MazeBase.setSize({2892.0f, 2892.0f});
    m_MazeBase.setPosition(0.0f, 0.0f);
    m_MazeBase.setFillColor(conf::MazeBaseColour);

    createWalls();  //
    createPosts();
    InitMaze();
    loadFromMemory(japan2007ef, MAZE_WIDTH);
  };

  ~MazeManager() = default;

  bool LoadMazeStringFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
      std::cerr << "Error: Could not open file " << filename << std::endl;
      return false;
    }

    std::ostringstream ss;
    ss << file.rdbuf();         // Read the file contents into the stringstream
    mazeDataString = ss.str();  // Store it in the mazeData string

    return true;
  }

  bool loadFromMemory(const uint8_t* data, int mazeWidth) {
    InitMaze();
    for (int y = 0; y < mazeWidth - 1; y++) {
      for (int x = 0; x < mazeWidth - 1; x++) {
        int index = x * mazeWidth + y;
        int walls = data[index];
        for (int d = 0; d < 4; d++) {
          // we need only do the North and East walls
          if (walls & BIT(0)) {
            SetWallState(x, y, Direction::North, WallState::Mystery);
          }
          if (walls & BIT(1)) {
            SetWallState(x, y, Direction::East, WallState::Mystery);
          }
        }
      }
    }
    SetWallState(0, 0, Direction::East, WallState::KnownPresent);
    return true;
  }

  bool loadFromList(MazeDataSource& mazeData) { return loadFromMemory(mazeData.data, MAZE_WIDTH); }

  void LoadMazeFromString(const std::string& mazeData) {
    (void)mazeData;
    //
  }

  bool isHorizontalWall(int wall_index) {
    return ((wall_index % LOCATION_DELTA_NORTH) < NUMBER_OF_HORIZ_WALLS);  //
  }

  int GetWallIndex(int cellX, int cellY, Direction direction) const {
    int index = cellX + LOCATION_DELTA_NORTH * cellY;
    switch (direction) {
      case Direction::North:
        index += LOCATION_DELTA_NORTH;
        break;
      case Direction::East:
        index += LOCATION_DELTA_NORTH_EAST;
        break;
      case Direction::South:
        index += 0;
        break;
      case Direction::West:
        index += LOCATION_DELTA_NORTH_WEST;
        break;
      default:
        index = 0;  // even though this is wrong.
        break;
    }
    return index;
  }

  sf::Color GetWallColor(WallState state) {
    switch (state) {
      case WallState::KnownAbsent:
        return conf::KnownAbsentColour;
      case WallState::KnownPresent:
        return conf::KnownPresentColour;
      case WallState::Unknown:
        return conf::UnknownColour;
      case WallState::Virtual:
        return conf::VirtualColour;
      default:
        return conf::ErrorColour;
    }
  }

  void SetWallState(int index, WallState state) {
    wallState[index] = state;  //
    m_vertexArrayWalls[index * 4 + 0].color = conf::WallStateColors[(int)state];
    m_vertexArrayWalls[index * 4 + 1].color = conf::WallStateColors[(int)state];
    m_vertexArrayWalls[index * 4 + 2].color = conf::WallStateColors[(int)state];
    m_vertexArrayWalls[index * 4 + 3].color = conf::WallStateColors[(int)state];
  }

  void SetWallState(int x, int y, Direction direction, WallState state) {
    int index = GetWallIndex(x, y, direction);
    SetWallState(index, state);
  }

  sf::Vector2f getCellOrigin(int x, int y) {
    return {(float)x * CELL_SIZE, (float)y * CELL_SIZE};  //
  }

  void InitWall(int x, int y, Direction direction, WallState state) {
    (void)x;
    (void)y;
    (void)direction;
    (void)state;
  }

  /**
   * The posts are static so just make them once
   */
  void createPosts() {
    //    sf::RectangleShape post({WALL_THICKNESS, WALL_THICKNESS});
    for (int y = 0; y <= MAZE_WIDTH; y++) {
      for (int x = 0; x <= MAZE_WIDTH; x++) {
        float ox = (float)x * CELL_SIZE;
        float oy = (float)y * CELL_SIZE + WALL_THICKNESS;
        int index = x * (MAZE_WIDTH + 1) + y;
        m_postRects[index] = {{ox, oy}, {WALL_THICKNESS, WALL_THICKNESS}};
      }
    }
    sf::Color color = conf::KnownPresentColour;
    for (std::size_t i = 0; i < NUMBER_OF_POSTS; ++i) {
      const sf::FloatRect& rect = m_postRects[i];
      const sf::Vector2f& position = Drawing::toWindowCoords(rect.getPosition(), conf::MazeSize);
      const sf::Vector2f& size = rect.getSize();

      m_vertexArrayPosts[i * 4 + 0].position = position;
      m_vertexArrayPosts[i * 4 + 1].position = sf::Vector2f(position.x + size.x, position.y);
      m_vertexArrayPosts[i * 4 + 2].position = sf::Vector2f(position.x + size.x, position.y + size.y);
      m_vertexArrayPosts[i * 4 + 3].position = sf::Vector2f(position.x, position.y + size.y);
      m_vertexArrayPosts[i * 4 + 0].color = color;
      m_vertexArrayPosts[i * 4 + 1].color = color;
      m_vertexArrayPosts[i * 4 + 2].color = color;
      m_vertexArrayPosts[i * 4 + 3].color = color;
    }
  }

  /***
   * Like the posts, we can create all the wall rectangles and their
   * associated vertex array entries when the maze if first created.
   * These positions do not change so the cost of creation is saved
   * on every frame.
   *
   * Walls can be set or cleared just by changing the state and then
   * applying the relevant colour
   */
  void createWalls() {
    sf::FloatRect wall_shape;
    sf::FloatRect hWall({0, 0}, {WALL_LENGTH, WALL_THICKNESS});
    sf::FloatRect vWall({0, 0}, {WALL_THICKNESS, WALL_LENGTH});
    for (int y = 0; y < MAZE_WIDTH; y++) {
      for (int x = 0; x < MAZE_WIDTH; x++) {
        sf::Vector2f origin = getCellOrigin(x, y);
        int index;
        sf::Vector2f offset;
        // North Wall
        offset.x = WALL_THICKNESS;
        offset.y = CELL_SIZE + WALL_THICKNESS;
        wall_shape = hWall;
        wall_shape.left = origin.x + offset.x;
        wall_shape.top = origin.y + offset.y;
        index = GetWallIndex(x, y, Direction::North);
        m_wallRects[index] = wall_shape;
        SetWallState(index, WallState::Unknown);
        // West Wall
        offset.x = 0;
        offset.y = CELL_SIZE;
        wall_shape = vWall;
        wall_shape.left = origin.x + offset.x;
        wall_shape.top = origin.y + offset.y;
        index = GetWallIndex(x, y, Direction::West);
        m_wallRects[index] = wall_shape;
        SetWallState(index, WallState::Unknown);
      }
    }
    // now the South and East border
    sf::Vector2f origin;
    for (int i = 0; i < MAZE_WIDTH; i++) {
      int index;
      origin = getCellOrigin(i, 0);
      sf::Vector2f offset;
      // South Wall
      offset.x = WALL_THICKNESS;
      offset.y = WALL_THICKNESS;
      wall_shape = hWall;
      wall_shape.left = origin.x + offset.x;
      wall_shape.top = origin.y + offset.y;
      index = GetWallIndex(i, 0, Direction::South);
      m_wallRects[index] = wall_shape;
      SetWallState(index, WallState::Unknown);

      // East Wall
      origin = getCellOrigin(MAZE_WIDTH - 1, i);
      offset.x = CELL_SIZE;
      offset.y = CELL_SIZE;
      wall_shape = vWall;
      wall_shape.left = origin.x + offset.x;
      wall_shape.top = origin.y + offset.y;
      index = GetWallIndex(MAZE_WIDTH - 1, i, Direction::East);
      std::cout << i << " > " << index << std::endl;
      m_wallRects[index] = wall_shape;
      SetWallState(index, WallState::Unknown);
    }
    sf::Color colour = conf::UnknownColour;
    for (std::size_t i = 0; i < NUMBER_OF_WALLS; ++i) {
      sf::FloatRect& rect = m_wallRects[i];
      sf::Vector2f position = rect.getPosition();
      position = Drawing::toWindowCoords(position, conf::MazeSize);
      sf::Vector2f size = rect.getSize();
      m_vertexArrayWalls[i * 4 + 0].position = position;
      m_vertexArrayWalls[i * 4 + 1].position = sf::Vector2f(position.x + size.x, position.y);
      m_vertexArrayWalls[i * 4 + 2].position = sf::Vector2f(position.x + size.x, position.y + size.y);
      m_vertexArrayWalls[i * 4 + 3].position = sf::Vector2f(position.x, position.y + size.y);
      m_vertexArrayWalls[i * 4 + 0].color = colour;
      m_vertexArrayWalls[i * 4 + 1].color = colour;
      m_vertexArrayWalls[i * 4 + 2].color = colour;
      m_vertexArrayWalls[i * 4 + 3].color = colour;
    }
  }

  void InitMaze() {
    for (int y = 0; y < MAZE_WIDTH; y++) {
      for (int x = 0; x < MAZE_WIDTH; x++) {
        SetWallState(x, y, Direction::North, WallState::Unknown);
        SetWallState(x, y, Direction::West, WallState::Unknown);
      }
    }
    for (int i = 0; i < MAZE_WIDTH; i++) {
      SetWallState(i, 0, Direction::South, WallState::KnownPresent);
      SetWallState(i, MAZE_WIDTH - 1, Direction::North, WallState::KnownPresent);
      SetWallState(0, i, Direction::West, WallState::KnownPresent);
      SetWallState(MAZE_WIDTH - 1, i, Direction::East, WallState::KnownPresent);
    }
    SetWallState(0, 0, Direction::East, WallState::KnownPresent);
    SetWallState(0, 0, Direction::North, WallState::KnownAbsent);
  }

  void Render(sf::RenderWindow& window) {
    window.draw(m_MazeBase);
    window.draw(m_vertexArrayWalls);
    window.draw(m_vertexArrayPosts);
  }

  void UpdateObstacles() {
    return;
    //    std::lock_guard<std::mutex> lock(m_ObstacleMutex);
    //    m_Obstacles.clear();
    //    int c = 40;
    //    for (auto& wall : m_walls) {
    //      if (wall.state == WallState::KnownPresent) {
    //        m_Obstacles.push_back(wall.shape);
    //      }
    //      if (--c <= 0) {
    //        return;
    //      }
    //    }
  }

  const std::vector<sf::RectangleShape>& GetObstacles() {
    UpdateObstacles();
    return m_Obstacles;  //
  }

 private:
  sf::VertexArray m_vertexArrayPosts;
  sf::VertexArray m_vertexArrayWalls;
  //  Wall m_walls[NUMBER_OF_WALLS];  // Array of wall states
  WallState wallState[NUMBER_OF_WALLS];
  sf::FloatRect m_wallRects[NUMBER_OF_WALLS];
  sf::FloatRect m_postRects[NUMBER_OF_POSTS];

  std::string mazeDataString;
  sf::RectangleShape m_MazeBase;
  bool m_MazeChanged = false;
  mutable std::mutex m_ObstacleMutex;  // Protects access to m_pose and m_orientation

  std::vector<sf::RectangleShape> m_Obstacles;
};

#endif  // MAZE_H
