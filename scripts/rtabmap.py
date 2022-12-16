import os.path as osp
from docker_helper import DockerMounts, RosDockerContainer


class RtabmapMounts(DockerMounts):
    def __init__(self, files=tuple(), folders=tuple()):
        super().__init__(files=files, folders=folders)
        self.pass_rtabmap_to_docker()

    def pass_rtabmap_to_docker(self):
        rtabmap_ws_folder = (osp.join(osp.dirname(__file__), '../../..'))
        self.docker_rtabmap_ws_folder, volume = \
            DockerMounts.pass_folders_to_docker(rtabmap_ws_folder, '/home/docker_rtabmap/catkin_ws')
        self.volume_args = self.volume_args + f'-v {volume} '


class Rtabmap(RosDockerContainer):
    def __init__(self, image_name, container_name, user_name=None):
        super().__init__(image_name, container_name, user_name=user_name)
        self.source_files = ['/home/docker_rtabmap/catkin_ws/devel_isolated/setup.bash']

    def create_containter(self, docker_mounts: RtabmapMounts=None, net='host'):
        if docker_mounts is None:
            docker_mounts = RtabmapMounts()
        super().create_containter(docker_mounts=docker_mounts, net=net)

    def run_rtabmap(self, load_map_path=None, save_map_path=None,
            accumulative_mapping=True, temporary_mapping=False,
            cell_size=0.1):
        if load_map_path is None:
            load_map_path = ""
        if save_map_path is None:
            save_map_path = ""
        result = self.roslaunch("rtabmap_example", "occupancy_grid_map.launch",
            arguments=
                f"load_map_path:={load_map_path} "
                f"save_map_path:={save_map_path} "
                f"accumulative_mapping:={str(accumulative_mapping).lower()} "
                f"temporary_mapping:={str(temporary_mapping).lower()} "
                f"cell_size:={cell_size} ",
            source_files=self.source_files)
        return result
