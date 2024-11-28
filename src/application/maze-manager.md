## Responsibilities of the MazeManager Class

1. Loading:

- Load a maze layout from a file or predefined data structure.
- Store the maze in an internal representation (e.g., a grid of cells).

2. Rendering:

- Render the maze onto the application window, showing walls, pathways, or any special markers (e.g., the goal).

3. MazeManager Queries:

- Provide utility functions for collision checking or sensor line-of-sight calculations.

## Proposed Attributes and Methods

### Attributes

- m_grid: A 2D vector (or array) to store the maze layout. Each cell can represent the presence of walls or open paths.
- m_cellSize: The size of each maze cell (in pixels), used for rendering.
- m_mazeSize: Dimensions of the maze in rows and columns.
- m_wallColor and m_pathColor: Colors used for rendering walls and paths.

### Methods

- Constructor: Initialize the maze with default values or load from a file.
- LoadMaze(const std::string& filename): Load a maze layout from a file.
- Render(sf::RenderWindow& window): Draw the maze onto the SFML window.
- GetCellType(int x, int y): Return the type of a specific cell (e.g., wall or path).
- SetCellType(int x, int y, int type): Update a specific cell's type.