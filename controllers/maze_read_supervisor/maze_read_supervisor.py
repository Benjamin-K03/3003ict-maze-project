from controller import Supervisor
import os

MAZE_FILE_PATH = "../../maze_layout.txt" # Import from project root
WALL_SIZE = 0.1
WALL_HEIGHT = 0.12

def load_maze_from_file(filepath):
    """Reads the text file and converts it into a 2D array of integers."""
    grid = []
    if not os.path.exists(filepath):
        print(f"ERROR: Cannot find maze file at {filepath}")
        return None

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if line:
                # Convert the string "1101" into a list of ints [1, 1, 0, 1]
                grid.append([int(char) for char in line])
    return grid

def spawn_maze():
    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())
    root_node = supervisor.getRoot()
    children_field = root_node.getField("children")

    grid = load_maze_from_file(MAZE_FILE_PATH)
    if not grid:
        return

    grid_height_cells = len(grid)
    grid_width_cells = len(grid[0])

    physical_width = grid_width_cells * WALL_SIZE
    physical_length = grid_height_cells * WALL_SIZE

    offset_x = -1 * WALL_SIZE
    offset_y = -1 * WALL_SIZE

    print(
        f"Generating a custom {grid_width_cells}x{grid_height_cells} maze..."
    )

    maze_string_parts = ["""
        Group {
          children [
        """]

    # Build foor string
    floor_str = f"""
            Solid {{
              translation {(-2 * WALL_SIZE + physical_width / 2.0) + (WALL_SIZE / 2.0)} {(-2 * WALL_SIZE + physical_length / 2.0) + (WALL_SIZE / 2.0)} -0.01
              children [
                Shape {{
                  appearance PBRAppearance {{
                    baseColor 0.3 0.3 0.3
                    roughness 0.9
                  }}
                  geometry Box {{
                    size {physical_width} {physical_length} 0.02
                  }}
                }}
              ]
              name "maze_floor"
              boundingObject Box {{
                size {physical_width} {physical_length} 0.02
              }}
            }}
    """
    maze_string_parts.append(floor_str)

    # Build walls string
    for row_idx, row in enumerate(grid):
        for col_idx, cell in enumerate(row):
            if cell == 1:
                flipped_row_idx = (grid_height_cells - 1) - row_idx # Flip row indexes to prevent maze being mirrored in webot 3d space

                pos_x = (col_idx * WALL_SIZE) + offset_x
                pos_y = (flipped_row_idx * WALL_SIZE) + offset_y
                pos_z = WALL_HEIGHT / 2.0

                wall_node_str = f"""
                Solid {{
                  translation {pos_x} {pos_y} {pos_z}
                  children [
                    Shape {{
                      appearance PBRAppearance {{
                        baseColor 0.2 0.5 0.8
                        roughness 0.8
                        metalness 0.1
                      }}
                      geometry Box {{
                        size {WALL_SIZE} {WALL_SIZE} {WALL_HEIGHT}
                      }}
                    }}
                  ]
                  name "maze_wall_{col_idx}_{row_idx}"
                  boundingObject Box {{
                    size {WALL_SIZE} {WALL_SIZE} {WALL_HEIGHT}
                  }}
                }}
                """
                maze_string_parts.append(wall_node_str)

    maze_string_parts.append("  ]\n}")
    final_maze_string = "".join(maze_string_parts)
    children_field.importMFNodeFromString(1, final_maze_string)

    supervisor.step(time_step)
    print("Custom maze generated successfully!")

if __name__ == "__main__":
    spawn_maze()