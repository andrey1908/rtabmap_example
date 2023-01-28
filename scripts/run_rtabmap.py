import argparse
import time
import os
import os.path as osp
from .rtabmap import Rtabmap, RtabmapMounts


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-lm', '--local-mapping', action='store_true')
    parser.add_argument('--load-map', type=str)
    parser.add_argument('--save-map', type=str)

    parser.add_argument('--build', action='store_true')
    return parser


def run_rtabmap():
    parser = build_parser()
    args = parser.parse_args()

    rtabmap = Rtabmap('rtabmap:latest', 'rtabmap')
    rtabmap.create_containter()

    # build and exit
    if args.build:
        rtabmap.build_rtabmap()
        exit(0)

    catkin_ws_folder = osp.abspath(osp.join(osp.dirname(__file__), "../../.."))
    docker_catkin_ws_folder = "/home/docker_rtabmap/catkin_ws"
    if not args.local_mapping:
        config_path = osp.join(docker_catkin_ws_folder,
            "src/rtabmap_example/config/husky.yaml")
        node_name = "occupancy_grid_map"
    else:
        config_path = osp.join(docker_catkin_ws_folder,
            "src/rtabmap_example/config/husky_local_mapping.yaml")
        node_name = "occupancy_grid_local_map"

    if args.load_map:
        load_map_filename = osp.abspath(osp.expanduser(args.load_map))
        if not load_map_filename.startswith(catkin_ws_folder + '/'):
            raise RuntimeError(f"Load map file {args.load_map} should be in "
                f"{catkin_ws_folder} folder")
        load_map_path = osp.join(docker_catkin_ws_folder,
            osp.relpath(load_map_filename, catkin_ws_folder))
    else:
        load_map_path = None

    if args.save_map:
        save_map_filename = osp.abspath(osp.expanduser(args.save_map))
        if not save_map_filename.startswith(catkin_ws_folder + '/'):
            raise RuntimeError(f"Save map file {args.save_map} should be in "
                f"{catkin_ws_folder} folder")
        save_map_path = osp.join(docker_catkin_ws_folder,
            osp.relpath(save_map_filename, catkin_ws_folder))
    else:
        save_map_path = None

    time_str = time.strftime("%Y-%m-%d_%H.%M.%S")
    results = rtabmap.run_rtabmap(config_path,
        load_map_path=load_map_path, save_map_path=save_map_path,
        node_name=node_name)

    logs_folder = osp.abspath(osp.expanduser(osp.join(catkin_ws_folder, "rtabmap_logs")))
    os.makedirs(logs_folder, exist_ok=True)
    with open(osp.join(logs_folder, f"{time_str}.txt"), 'w') as f:
        f.write(results.stdout)
