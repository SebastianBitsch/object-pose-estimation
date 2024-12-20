import os
import argparse

import numpy as np
import pyzed.sl as sl

from tqdm import trange
from PIL import Image

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--output_dir',          type=str,                      help='Path of where to output the files into a rgb/ depth/ and cloud/ dir')
    parser.add_argument('--im_w',                type=int,   default=1920,      help='Width of captured frame')
    parser.add_argument('--im_h',                type=int,   default=1080,      help='Width of captured frame')
    parser.add_argument('--n_frames',            type=int,   default=100,       help='How many frames to capture')
    parser.add_argument('--fps',                 type=int,   default=30,        help='How many frames to capture per second')
    parser.add_argument('--resolution',          type=str,   default="vga",     help="What resolution to use, select from: [HD4K, QHDPLUS, HD2K, HD1080, HD1200, HD720, SVGA, VGA, AUTO, LAST]")
    parser.add_argument('--unit',                type=str,   default="meter",   help="What unit should the measurements in, select one from: [MILLIMETER, CENTIMETER, METER, INCH, FOOT, LAST]")
    parser.add_argument('--depth_mode',          type=str,   default="quality", help="What quality should depth map be captured in, select one from: [NONE, PERFORMANCE, QUALITY, ULTRA, NEURAL, NEURAL_PLUS, LAST]")
    parser.add_argument('--min_depth',           type=float, default=0.2,       help="What is the min depth of the scene")
    parser.add_argument('--max_depth',           type=float, default=5.0,       help="What is the max depth of the scene")
    parser.add_argument('--depth_stabilization', type=float, default=0.0,       help="How much depth stabilization to use")

    args = parser.parse_args()

    assert args.resolution.upper() in set(sl.RESOLUTION), f"Error: Expected resolution to one of {set(sl.RESOLUTION)}, got {args.resolution}"
    assert args.unit.upper()       in set(sl.UNIT),       f"Error: Expected unit to one of {set(sl.UNIT)}, got {args.unit}"
    assert args.depth_mode.upper() in set(sl.DEPTH_MODE), f"Error: Expected depth mode to one of {set(sl.DEPTH_MODE)}, got {args.depth_mode}"

    rgb_dir   = os.path.join(args.output_dir, "rgb")    
    depth_dir = os.path.join(args.output_dir, "depth")
    cloud_dir = os.path.join(args.output_dir, "cloud")

    os.makedirs(rgb_dir,   exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(cloud_dir, exist_ok=True)

    depth = sl.Mat(args.im_w, args.im_h, sl.MAT_TYPE.U8_C4)
    image = sl.Mat(args.im_w, args.im_h, sl.MAT_TYPE.U8_C4)
    cloud = sl.Mat()

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION[args.resolution.upper()]
    init_params.camera_fps = args.fps

    init_params.coordinate_units = sl.UNIT[args.unit.upper()]
    init_params.depth_mode = sl.DEPTH_MODE[args.quality.upper()]

    init_params.depth_minimum_distance = args.min_depth
    init_params.depth_maximum_distance = args.max_depth
    init_params.depth_stabilization    = args.depth_stabilization


    error = zed.open(init_params)
    if error != sl.ERROR_CODE.SUCCESS:
        print(f"Error: Failed to open camera with error: '{error}', exit program")
        exit(1)

    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.enable_fill_mode = True

    for i in trange(args.n_frames):

        if zed.grab(runtime_parameters) != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to capture frame {i+1}")
            continue

        zed.retrieve_image(image, sl.VIEW.LEFT)                     # Get the left image, thats our origin
        im = (image.get_data()[:,:,:3])[:, :, ::-1]                 # kinda shit but BGRA -> RGB
        im = np.nan_to_num(im, nan=0.0, posinf=0.0, neginf = 0.0)   # where depth couldnt be found will be inf or nan, just put zeros
        rgb = Image.fromarray(im)

        zed.retrieve_image(depth, sl.VIEW.DEPTH)                    # Get depth matrix. Depth is aligned on the left RGB image
        d = Image.fromarray(depth.get_data()[:,:,0])                # Has multiple channels for some reason, we just grap first

        zed.retrieve_measure(cloud, sl.MEASURE.XYZRGBA)

        # Save
        rgb.save(os.path.join(rgb_dir, f"im_{i:0>7d}.png"))
        d.save(os.path.join(depth_dir, f"im_{i:0>7d}.png"))
        cloud.write(os.path.join(cloud_dir, f"im_{i:0>7d}.ply"))

    zed.close()

