import os
import argparse

import numpy as np
import open3d as o3d

# Just a basic dummy material
material_contentes = """
    newmtl material_0
    Ka 0.40000000 0.40000000 0.40000000
    Kd 0.40000000 0.40000000 0.40000000
    Ks 0.40000000 0.40000000 0.40000000
    Ns 1.00000000
    map_Kd material_0.png
"""

# Convert .stl models to simple .obj 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--file', type=str, help='Path of the file to convert, output will be in the same dir named *.ply')
    parser.add_argument('--draw_output', type=bool, default=False, help='Should the resulting file be shown in plot')
    parser.add_argument('--scale', type=float, default=1.0, help="Should the object be scaled, if so by how much")
    parser.add_argument('--write_ascii', type=bool, default=True, help='Whether to write the file as plain text file or in binary')
    args = parser.parse_args()

    assert args.file.endswith(".stl"), f"Error: Unsupported file, file should be a `.stl` file, got {args.file}"

    print("Parsing file ... ")

    base_path = os.path.dirname(args.file)

    mesh = o3d.io.read_triangle_mesh(args.file)

    mesh.scale(args.scale, center=mesh.get_center())
    mesh.compute_vertex_normals()

    # TEXTURE
    texture = np.ones((16,16,3), dtype=np.uint8) * 100 # grey color
    o3d.io.write_image(os.path.join(base_path, "material_0.png"), o3d.geometry.Image(texture))


    # MATERIAL
    with open(os.path.join(base_path, "material.mtl"), "w") as mat_file:
        mat_file.write(material_contentes)


    # OBJECT
    file_out = args.file.replace(".stl", ".obj")
    
    with open(file_out, "r+") as f:

        # Add reference to the material and texture
        f.write(f"mtllib material.mtl\n")
        f.write(f"usemtl material_0\n")
        
        # Add vertices
        for v, n in zip(mesh.vertices, mesh.vertex_normals):
            f.write(f"v {v[0]} {v[1]} {v[2]}\n")
            f.write(f"vn {n[0]} {n[1]} {n[2]}\n")
            f.write("vt 0.5 0.5\n")

        # Add triangles
        for t in mesh.triangles:
            v1, v2, v3 = t + 1
            f.write(f"f {v1}/{v1} {v2}/{v2} {v3}/{v3}\n")

    print(f"Done. Saving to '{file_out}'")

    # Print the size of object for debugging
    bb = mesh.get_axis_aligned_bounding_box()
    size = bb.max_bound - bb.min_bound
    print("Size of object:")
    print(f"Width  (x): {size[0]:.2f}")
    print(f"Height (y): {size[1]:.2f}")
    print(f"Depth  (z): {size[2]:.2f}")

    # Draw output
    if args.draw_output:
        o3d.visualization.draw_geometries([mesh])
