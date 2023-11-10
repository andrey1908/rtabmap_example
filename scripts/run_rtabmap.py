import argparse
import time
import os
import os.path as osp
import glob
import shutil
if __name__ == '__main__':
    from rtabmap import Rtabmap, RtabmapMounts
else:
    from .rtabmap import Rtabmap, RtabmapMounts


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-lm', '--local-mapping', action='store_true')
    parser.add_argument('--load-map', type=str)
    parser.add_argument('--save-map', type=str)
    parser.add_argument('--save-tracking-results', type=str)

    parser.add_argument('-sem', '--use-semantic', action='store_true')

    parser.add_argument('--log-rosbag', action='store_true')
    parser.add_argument('--move-rosbags-to', type=str)

    parser.add_argument('--build', action='store_true')
    return parser


def getPathInsideDocker(path: str,
        catkin_ws_folder: str, docker_catkin_ws_folder: str):
    full_path = osp.abspath(osp.expanduser(path))
    if not full_path.startswith(catkin_ws_folder + '/'):
        raise RuntimeError(f"Load map file {path} should be in "
            f"{catkin_ws_folder} folder")
    docker_path = osp.join(docker_catkin_ws_folder,
        osp.relpath(full_path, catkin_ws_folder))
    return docker_path


def removeOldRosbagLogFiles(folder, max_total_size):  # in MB
    files = sorted(glob.glob(folder + "/*.bag") + glob.glob(folder + "/*.bag.active"))
    sizes = list(map(osp.getsize, files))
    sizes = [size / 1024 / 1024 for size in sizes]

    accum_size = 0
    i = len(files)
    while accum_size <= max_total_size:
        i -= 1
        if i < 0:
            break
        accum_size += sizes[i]

    i += 1
    for j in range(i):
        os.remove(files[j])


def moveRosbagsTo(from_folder, to_folder):
    files = sorted(glob.glob(from_folder + "/*.bag"))
    for file in files:
        shutil.move(file, to_folder)


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
    if not osp.isdir(osp.join(catkin_ws_folder, 'src')):
        raise RuntimeError("Looks like rtabmap_example package is not in 'src' folder")
    docker_catkin_ws_folder = "/home/docker_rtabmap/catkin_ws"

    config_paths = [osp.join(docker_catkin_ws_folder,
        "src/rtabmap_example/config/husky.yaml")]
    if not args.local_mapping:
        node_name = "occupancy_grid_map"
    else:
        config_paths.append(osp.join(docker_catkin_ws_folder,
            "src/rtabmap_example/config/husky_enable_local_mapping.yaml"))
        node_name = "occupancy_grid_local_map"

    if args.load_map:
        load_map_path = getPathInsideDocker(args.load_map,
            catkin_ws_folder, docker_catkin_ws_folder)
    else:
        load_map_path = None

    if args.save_map:
        save_map_path = getPathInsideDocker(args.save_map,
            catkin_ws_folder, docker_catkin_ws_folder)
    else:
        save_map_path = None

    if args.save_tracking_results:
        save_tracking_results_path = getPathInsideDocker(args.save_tracking_results,
            catkin_ws_folder, docker_catkin_ws_folder)
    else:
        save_tracking_results_path = None

    time_str = time.strftime("%Y-%m-%d_%H.%M.%S")
    logs_folder = osp.abspath(osp.expanduser(osp.join(catkin_ws_folder, "rtabmap_logs")))
    docker_logs_folder = osp.join(docker_catkin_ws_folder, "rtabmap_logs")
    os.makedirs(logs_folder, exist_ok=True)

    if args.log_rosbag:
        removeOldRosbagLogFiles(logs_folder, max_total_size=1024)
        topics_to_record = ['/tf', '/tf_static',
            '/cartographer/tracked_local_odometry', '/cartographer/tracked_global_odometry',
            '/cartographer/trajectory_node_list', '/cartographer/constraint_list']
        docker_out_rosbag_log_file = osp.join(docker_logs_folder, f"{time_str}.bag")
        rtabmap.rosrun_async("rosbag", "record",
            arguments=f"{' '.join(topics_to_record)} -O {docker_out_rosbag_log_file}", session='rtabmap_rosbag_log')

    results = rtabmap.run_rtabmap(config_paths,
        load_map_path=load_map_path, save_map_path=save_map_path,
        save_tracking_results_path=save_tracking_results_path,
        node_name=node_name, use_semantic=args.use_semantic)

    if args.log_rosbag:
        rtabmap.stop_session('rtabmap_rosbag_log')
        if args.move_rosbags_to:
            os.makedirs(args.move_rosbags_to, exist_ok=True)
            print("Moving rosbag logs to HDD. Please wait...")
            moveRosbagsTo(logs_folder, args.move_rosbags_to)
            print("Done.")

    with open(osp.join(logs_folder, f"{time_str}.txt"), 'w') as f:
        f.write(results.stdout)


if __name__ == '__main__':
    run_rtabmap()
