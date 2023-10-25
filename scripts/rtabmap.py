import os.path as osp
from docker_helper import DockerMounts, RosDockerContainer


class RtabmapMounts(DockerMounts):
    def __init__(self, files=tuple(), folders=tuple()):
        super().__init__(files=files, folders=folders)
        self.pass_rtabmap_to_docker()

    def pass_rtabmap_to_docker(self):
        rtabmap_ws_folder = (osp.join(osp.dirname(__file__), '../../..'))
        if not osp.isdir(osp.join(rtabmap_ws_folder, 'src')):
            raise RuntimeError("Looks like rtabmap_example package is not in 'src' folder")
        self.docker_rtabmap_ws_folder, volume = \
            DockerMounts.pass_folders_to_docker(rtabmap_ws_folder, '/home/docker_rtabmap/catkin_ws')
        self.volume_args = self.volume_args + f'-v {volume} '


class Rtabmap(RosDockerContainer):
    def __init__(self, image_name, container_name, user_name=None):
        super().__init__(image_name, container_name, user_name=user_name)
        self.source_files = ['/home/docker_rtabmap/catkin_ws/devel_isolated/setup.bash']

    def create_containter(self, mounts: RtabmapMounts=None, net='host'):
        if mounts is None:
            mounts = RtabmapMounts()
        super().create_containter(mounts=mounts, net=net)

    def build_rtabmap(self):
        result = self.run("cd ~/catkin_ws && catkin_make_isolated -DCMAKE_BUILD_TYPE=Release")
        return result

    def run_rtabmap(self, config_paths,
            load_map_path=None, save_map_path=None,
            save_tracking_results_path=None,
            optimization_results_topic=None, node_name=None,
            use_semantic=False):
        result = self.roslaunch("rtabmap_example", "occupancy_grid_map.launch",
            arguments=
                (f"config_paths:={','.join(config_paths)} ") +
                (f"load_map_path:={load_map_path} " if load_map_path else "") +
                (f"save_map_path:={save_map_path} " if save_map_path else "") +
                (f"save_tracking_results_path:={save_tracking_results_path} " if save_tracking_results_path else "") +
                (f"optimization_results:={optimization_results_topic} " if optimization_results_topic else "") +
                (f"node_name:={node_name} " if node_name else "") +
                (f"accum/subscribe_rgb:={use_semantic}"),
            source_files=self.source_files)
        return result
