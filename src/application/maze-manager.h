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
  Virtual        //
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

  MazeManager() {
    m_MazeBase.setSize({2892.0f, 2892.0f});
    m_MazeBase.setPosition(0.0f, 0.0f);
    m_MazeBase.setFillColor(conf::MazeBaseColour);

    InitMaze();  //
    InitPosts();
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
    for (int y = 0; y < mazeWidth; y++) {
      for (int x = 0; x < mazeWidth; x++) {
        int index = x * mazeWidth + y;
        int walls = data[index];
        for (int d = 0; d < 4; d++) {
          // we need only do the North and East walls
          if (walls & BIT(0)) {
            SetWallState(x, y, Direction::North, WallState::KnownPresent);
          }
          if (walls & BIT(1)) {
            SetWallState(x, y, Direction::East, WallState::KnownPresent);
          }
        }
      }
    }
    m_vertexArrayWalls = MakeVertexArrayFromWalls(m_walls);
    return true;
  }

  bool loadFromList(MazeDataSource& mazeData) { return loadFromMemory(mazeData.data, MAZE_WIDTH); }

  void LoadMazeFromString(const std::string& mazeData) {
    (void)mazeData;
    //
  }

  bool IsHorizontal(int wall_index) {
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
        return conf::knownAbsentColour;
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
    m_walls[index].state = state;
    m_walls[index].shape.setFillColor(GetWallColor(state));
  }

  void SetWallState(int x, int y, Direction direction, WallState state) {
    int index = GetWallIndex(x, y, direction);
    SetWallState(index, state);
  }

  void InitWall(int x, int y, Direction direction, WallState state) {
    sf::RectangleShape wall_shape;
    /// get the top left
    float cx = float(x) * CELL_SIZE;
    float cy = (CELL_SIZE * MAZE_WIDTH) - (float)(y + 1) * CELL_SIZE;
    switch (direction) {
      case Direction::North:
        cx += WALL_THICKNESS + 1;
        wall_shape.setSize({WALL_LENGTH, WALL_THICKNESS});
        break;
      case Direction::East:
        cx += CELL_SIZE;
        cy += WALL_THICKNESS + 1;
        wall_shape.setSize({WALL_THICKNESS, WALL_LENGTH});
        break;
      case Direction::South:
        cx += WALL_THICKNESS + 1;
        cy += CELL_SIZE;
        wall_shape.setSize({WALL_LENGTH, WALL_THICKNESS});
        break;
      case Direction::West:
        cy += WALL_THICKNESS + 1;
        wall_shape.setSize({WALL_THICKNESS, WALL_LENGTH});
        break;
      default:
        break;  // this is actually an error.
    }
    int index = GetWallIndex(x, y, direction);
    wall_shape.setPosition(cx, cy);
    m_walls[index].shape = wall_shape;
    SetWallState(index, state);
  }

  void InitPosts() {
    // the posts are static so just make them once

    for (int y = 0; y <= MAZE_WIDTH; y++) {
      for (int x = 0; x <= MAZE_WIDTH; x++) {
        float left = (float)x * CELL_SIZE;
        float top = (CELL_SIZE * MAZE_WIDTH) - (float)(y)*CELL_SIZE;

        sf::FloatRect post_rect(left + GAP / 2.0f, top + GAP / 2.0f, WALL_THICKNESS - GAP, WALL_THICKNESS - GAP);
        int index = x * (MAZE_WIDTH + 1) + y;
        m_posts[index] = post_rect;
      }
    }
    m_vertexArrayPosts = MakeVertexArrayFromPosts(m_posts, conf::KnownPresentColour);
  }

  void InitMaze() {
    for (int y = 0; y < MAZE_WIDTH; y++) {
      for (int x = 0; x < MAZE_WIDTH; x++) {
        InitWall(x, y, Direction::North, WallState::Unknown);
        InitWall(x, y, Direction::West, WallState::Unknown);
      }
    }
    for (int i = 0; i < MAZE_WIDTH; i++) {
      InitWall(i, 0, Direction::South, WallState::KnownPresent);
      InitWall(i, MAZE_WIDTH - 1, Direction::North, WallState::KnownPresent);
      InitWall(0, i, Direction::West, WallState::KnownPresent);
      InitWall(MAZE_WIDTH - 1, i, Direction::East, WallState::KnownPresent);
    }
    InitWall(0, 0, Direction::East, WallState::KnownPresent);
    InitWall(0, 0, Direction::North, WallState::KnownAbsent);

    std::cout << "Maze Width " << MAZE_WIDTH << std::endl;
    std::cout << "Wall Length " << WALL_LENGTH << std::endl;
    std::cout << "Wall Width " << WALL_THICKNESS << std::endl;
    m_vertexArrayWalls = MakeVertexArrayFromWalls(m_walls);
  }

  void Render(sf::RenderWindow& window) {
    window.draw(m_MazeBase);

    window.draw(m_vertexArrayWalls);
    window.draw(m_vertexArrayPosts);
  }

  void UpdateObstacles() {
    std::lock_guard<std::mutex> lock(m_ObstacleMutex);
    m_Obstacles.clear();
    int c = 40;
    for (auto& wall : m_walls) {
      if (wall.state == WallState::KnownPresent) {
        m_Obstacles.push_back(wall.shape);
      }
      if (--c <= 0) {
        return;
      }
    }
  }

  std::vector<sf::RectangleShape> GetObstacles() {
    return m_Obstacles;  //
  }

 private:
  /***
   * This utility will take a list of float rectangles (should be ints?) and create a vertex array
   * for faster rendering.
   *
   * I think all the walls need to go in one list and all the posts in another list. The posts are
   * static and never change so it can be done once..
   * The walls are dynamic and need to be updated whenever they change.
   *
   * @param rectangles
   * @param color
   * @return
   */
  //    sf::VertexArray MakeVertexArray(const std::vector<sf::FloatRect>& rects, const sf::Color& color = sf::Color::Red) {
  sf::VertexArray MakeVertexArrayFromPosts(const sf::FloatRect* rectangles, const sf::Color& color = conf::KnownPresentColour) {
    sf::VertexArray vertexArray(sf::Quads, NUMBER_OF_POSTS * 4);

    for (std::size_t i = 0; i < NUMBER_OF_POSTS; ++i) {
      const sf::FloatRect& rect = rectangles[i];
      float left = rect.left;
      float top = rect.top;
      vertexArray[i * 4 + 0].position = sf::Vector2f(left, top);
      vertexArray[i * 4 + 1].position = sf::Vector2f(left + rect.width, top);
      vertexArray[i * 4 + 2].position = sf::Vector2f(left + rect.width, top + rect.height);
      vertexArray[i * 4 + 3].position = sf::Vector2f(left, top + rect.height);

      vertexArray[i * 4 + 0].color = color;
      vertexArray[i * 4 + 1].color = color;
      vertexArray[i * 4 + 2].color = color;
      vertexArray[i * 4 + 3].color = color;
    }

    return vertexArray;
  }

  /***
   * While waiting to convert the walls/posts to plain rectangles, we need to make
   * a vertex array from the vector of walls that we have now.
   *
   * This method does that.
   */
  //  sf::VertexArray MakeVertexArrayFromShapes(const std::vector<sf::RectangleShape>& shapes) {
  sf::VertexArray MakeVertexArrayFromWalls(const Wall* walls) {
    sf::VertexArray vertexArray(sf::Quads, NUMBER_OF_WALLS * 4);
    for (std::size_t i = 0; i < NUMBER_OF_WALLS; ++i) {
      const sf::RectangleShape& shape = walls[i].shape;
      const sf::Vector2f& position = shape.getPosition();
      const sf::Vector2f& size = shape.getSize();
      const sf::Color& color = shape.getFillColor();

      vertexArray[i * 4 + 0].position = position;
      vertexArray[i * 4 + 1].position = sf::Vector2f(position.x + size.x, position.y);
      vertexArray[i * 4 + 2].position = sf::Vector2f(position.x + size.x, position.y + size.y);
      vertexArray[i * 4 + 3].position = sf::Vector2f(position.x, position.y + size.y);

      vertexArray[i * 4 + 0].color = color;
      vertexArray[i * 4 + 1].color = color;
      vertexArray[i * 4 + 2].color = color;
      vertexArray[i * 4 + 3].color = color;
    }
    return vertexArray;
  }

  sf::VertexArray m_vertexArrayWalls;
  sf::VertexArray m_vertexArrayPosts;
  Wall m_walls[NUMBER_OF_WALLS];  // Array of wall states
  /// we need to retain the rectangles for sensors and collisions
  sf::FloatRect m_posts[NUMBER_OF_POSTS];  // Array of post shapes

  std::string mazeDataString;
  sf::RectangleShape m_MazeBase;
  bool m_MazeChanged = false;
  mutable std::mutex m_ObstacleMutex;  // Protects access to m_pose and m_orientation

  std::vector<sf::RectangleShape> m_Obstacles;
};

#endif  // MAZE_H
