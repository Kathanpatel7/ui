import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Define global variables
global_width = 0
global_length = 0
global_height = 0
global_pallet_height = 0
global_pallet_width = 0
global_clearance = 0

def store_data(data):
    global global_width, global_length, global_height, global_pallet_height, global_pallet_width, global_clearance
    # Store the six integers in the global variables
    global_width, global_length, global_height,_, global_pallet_height, global_pallet_width, global_clearance = data

if __name__ == "__main__":
    # Read the integers from the command-line arguments
    data = list(map(int, sys.argv[1:]))

    # Store the data in global variables
    store_data(data)

    # Server Connection Code Ends

    def can_place_box(x, y, box_width, box_height, pallet_size, clearance, placed_items, overhang):
        """Check if a box can be placed at the given position."""
        if overhang:
            if (x + box_width + clearance > pallet_size[0] + overhang[0] or 
                y + box_height + clearance > pallet_size[1] + overhang[1]):
                return False
        else:
            if (x + box_width + clearance > pallet_size[0] or 
                y + box_height + clearance > pallet_size[1]):
                return False
        return not any(
            (x - clearance < px + pw and x + box_width + clearance > px and 
             y - clearance < py + ph and y + box_height + clearance > py)
            for px, py, pw, ph, _ in placed_items  # Adjust unpacking for orientation info
        )

    def find_next_position(pallet_size, box_dim, clearance, placed_items, overhang):
        for y in range(pallet_size[1] - box_dim[1] + 1):
            for x in range(pallet_size[0] - box_dim[0] + 1):
                if can_place_box(x, y, box_dim[0], box_dim[1], pallet_size, clearance, placed_items, overhang):
                    return (x, y)
        return None

    def optimize_box_placement(box_dim, pallet_size, clearance, overhang):
        placed_items = []
        while True:
            position = find_next_position(pallet_size, box_dim, clearance, placed_items, overhang)
            if position:
                placed_items.append((position[0], position[1], box_dim[0], box_dim[1], 0))  # Added orientation info
            else:
                # Try with rotated box
                rotated_box_dim = (box_dim[1], box_dim[0])
                position = find_next_position(pallet_size, rotated_box_dim, clearance, placed_items, overhang)
                if position:
                    placed_items.append((position[0], position[1], rotated_box_dim[0], rotated_box_dim[1], 1))  # Orientation changed
                else:
                    break  # No more space on the pallet

        return placed_items

    def get_center_coordinates(placed_items):
        center_coordinates = []
        for item in placed_items:
            center_x = item[0] + item[2] / 2
            center_y = item[1] + item[3] / 2
            center_coordinates.append((center_x, center_y, item[4]))  # Include orientation info in the center coordinates
        return center_coordinates

    def plot_pallet(pallet_size, items, clearance):
        fig, ax = plt.subplots()
        pallet = patches.Rectangle((0, 0), pallet_size[0], pallet_size[1], edgecolor='black', facecolor='none')
        ax.add_patch(pallet)

        for item in items:
            rect = patches.Rectangle((item[0], item[1]), item[2], item[3], edgecolor='blue', facecolor='lightblue')
            ax.add_patch(rect)

        # Plot clearance rectangles
        for item in items:
            clearance_rect = patches.Rectangle((item[0] - clearance, item[1] - clearance), 
                                                item[2] + 2 * clearance, item[3] + 2 * clearance, 
                                                edgecolor='red', facecolor='none')
            ax.add_patch(clearance_rect)

        center_coordinates = get_center_coordinates(items)
        for center in center_coordinates:
            plt.scatter(center[0], center[1], color='green', marker='x')  # Changed color to green

        plt.xlim(0, pallet_size[0])
        plt.ylim(0, pallet_size[1])
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

    # User input for box dimensions
    box_width = global_width
    box_height = global_height

    # User input for pallet size (width, height)
    pallet_width = global_pallet_width
    pallet_height = global_pallet_height

    # User input for clearance
    clearance = global_clearance

    # User input for overhang
    overhang_option = 'n'
    if overhang_option.lower() == 'y':
        overhang_length = int(input("Enter the overhang in MM: "))
        overhang_width = overhang_length
        overhang = (overhang_length, overhang_width)
    else:
        overhang = None

    # Adjust pallet size with overhang
    if overhang:
        pallet_width += overhang[0]
        pallet_height += overhang[1]

    # Optimize box placement
    placed_items = optimize_box_placement((box_width, box_height), (pallet_width, pallet_height), clearance, overhang)

    # Get the center coordinates
    center_coordinates = get_center_coordinates(placed_items)

    # Output the center coordinates
    output = "Center coordinates of each box:\n"
    for center in center_coordinates:
        output += f"({center[0]}, {center[1]}, {center[2]})\n"

    # Plot the pallet with items and center coordinates
    plot_pallet((pallet_width, pallet_height), placed_items, clearance)

    # Output the number of boxes that could be placed
    output += f"Number of boxes that could be placed: {len(placed_items)}\n"

    print(output)
