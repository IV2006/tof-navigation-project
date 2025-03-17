import matplotlib.pyplot as plt
import re

dig_str = r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
# dig_str = r"-?\d+(\.\d+)?"

def get_points(out_file, delim, num_digits):
    points = []
    if num_digits == 2:
        search_str = fr"{dig_str}\s*{dig_str}\s*"
    else:
        search_str = fr"{dig_str}\s*{dig_str}\s*{dig_str}\s*"
    
    with open(out_file, 'r') as file:
        recording = False  # flag to indicate whether we're between the delimiters
        for line in file:
            line = line.strip()
            
            if delim in line and not recording:
                recording = True
                continue
            elif delim in line and recording:
                return points

            if recording:
                match = re.search(search_str, line)
                if match:
                    digits = match.groups()
                    x, y = float(digits[0]), float(digits[num_digits - 1])
                    if -2 <= x <= 2 and -2 <= y <= 2:  # ignore outlires
                        points.append((x, y))
                else:
                    print(f"Line did not match: {line}")

def get_all_points(out_file):
    delim = "--------------------------------------------------ALL-POINTS--------------------------------------------------"
    all_points = get_points(out_file, delim, 3)
    print(f"{all_points=}")
    return all_points

def get_edge_points(out_file):
    delim = "--------------------------------------------------EDGE-POINTS--------------------------------------------------"
    edge_points = get_points(out_file, delim, 2)
    print(f"{edge_points=}")
    return edge_points

def get_exit_points(out_file):
    delim = "--------------------------------------------------EXIT-POINTS--------------------------------------------------"
    exit_points = get_points(out_file, delim, 2)
    print(f"{exit_points=}")
    return exit_points


def show_points(all_points, edge_points, exit_points):
    p_x_vals = [point[0] for point in all_points]
    p_y_vals = [point[1] for point in all_points]

    e_x_vals = [point[0] for point in edge_points]
    e_y_vals = [point[1] for point in edge_points]

    i_x_vals = [point[0] for point in exit_points]
    i_y_vals = [point[1] for point in exit_points]

    # create the plot
    plt.figure(figsize=(8, 6))
    plt.scatter(p_x_vals, p_y_vals, color='red', s=4)
    plt.scatter(e_x_vals, e_y_vals, color='blue', s=16)
    plt.scatter(i_x_vals, i_y_vals, color='green', s=64)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.axis('equal')  # to keep the aspect ratio
    plt.show()

if __name__ == "__main__":
    out_file = "/media/sf_AI_extras/output.txt"
    all_points = get_all_points(out_file)
    edge_points = get_edge_points(out_file)
    exit_points = get_exit_points(out_file)
    show_points(all_points, edge_points, exit_points)
