import serial
import math
import open3d as o3d

def wait_for_sensor_data(connection):
    while True:
        line = connection.readline()
        decoded_line = line.decode().strip()
        if decoded_line == "DATA TX STARTED":
            return True
        elif decoded_line == "DATA TX STOPPED":
            return False
        else:
            try:
                return float(decoded_line)
            except ValueError:
                continue

def convert_polar_to_xyz(dist, angle_deg, offset_x):
    x = offset_x
    y = dist * math.cos(math.radians(angle_deg))
    z = dist * math.sin(math.radians(angle_deg))
    return x, y, z

def build_wireframe_geometry(coordinates):
    wireframe = o3d.geometry.LineSet()
    wireframe.points = o3d.utility.Vector3dVector(coordinates)
    edges = []

    total_layers = len(coordinates) // 32
    for layer in range(total_layers):
        for point in range(32):
            current_index = layer * 32 + point
            next_index = current_index + 1 if point < 31 else layer * 32
            edges.append([current_index, next_index])
            if layer < total_layers - 1:
                edges.append([current_index, current_index + 32])
        edges.append([layer * 32, layer * 32 + 31])

    wireframe.lines = o3d.utility.Vector2iVector(edges)
    return wireframe

def generate_point_cloud(coordinates):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(coordinates)
    return cloud

def display_scene(cloud, wireframe):
    o3d.visualization.draw_geometries([cloud, wireframe])

def main():
    port = serial.Serial('COM4', 115200, timeout=10)
    port.reset_output_buffer()
    port.reset_input_buffer()

    angle_step = 360 / 32
    distance_data = []
    scan_index = 0
    current_x = 0
    points_per_layer = 32

    input("Press Enter to begin scan...")
    port.write('s'.encode())

    while True:
        reading = wait_for_sensor_data(port)
        if reading is True:
            continue
        elif reading is False:
            break
        else:
            angle = scan_index * angle_step
            x, y, z = convert_polar_to_xyz(reading, angle, current_x)
            print(f"x: {x}, y: {y}, z: {z}")
            distance_data.append((x, y, z))

            scan_index += 1
            if scan_index % points_per_layer == 0:
                current_x += 500

    port.close()

    cloud = generate_point_cloud(distance_data)
    frame = build_wireframe_geometry(distance_data)
    display_scene(cloud, frame)

if __name__ == "__main__":
    main()