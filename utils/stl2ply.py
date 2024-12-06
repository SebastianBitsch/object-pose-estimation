# From https://gist.github.com/SebastianBitsch/bf1d4df30ae6ee3ca3d2d274e57e7e57
import argparse

import open3d as o3d

# Convert .stl models to .ply point clouds
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--file', type=str, help='Path of the file to convert, output will be in the same dir named *.ply')
    parser.add_argument('--n_points', type=int, default=10000, help='How many points the ply file should contain')
    parser.add_argument('--scale', type=float, default=1.0, help="How much to scale the object")
    parser.add_argument('--draw_output', type=bool, default=False, help='Should the resulting file be shown in plot')
    parser.add_argument('--write_ascii', type=bool, default=True, help='Whether to write the file as plain text file or in binary')
    args = parser.parse_args()

    assert args.file.endswith(".stl"), f"Error: Unsupported file, file should be a `.stl` file, got {args.file}"

    print("Parsing file ... ")

    mesh = o3d.io.read_triangle_mesh(args.file)

    # Scale and translate
    mesh.scale(args.scale, center=mesh.get_center())
    mesh.translate(-mesh.get_center())

    pointcloud = mesh.sample_points_poisson_disk(args.n_points)

    if args.draw_output:
        o3d.visualization.draw_geometries([mesh])       # Draw input
        o3d.visualization.draw_geometries([pointcloud]) # Draw output

    file_out = args.file.replace(".stl", ".ply")
    o3d.io.write_point_cloud(
        file_out, 
        pointcloud, 
        format = "auto", 
        write_ascii = args.write_ascii, 
        compressed = False, 
        print_progress = True
    )