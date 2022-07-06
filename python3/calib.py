import open3d as o3d
import numpy as np
import os
import yaml
import datetime
import time
import subprocess
from tqdm import tqdm
from utils import *

EXTENSIONS_PCD = ['.pcd']
VERSION = "2.0.0"
PROJECT = "Test"
EXCUTABLE_BIN = "exec bin/calibration "
PCDS_ADDR = "data/pcds"


def is_pcd(filename):
    return any(filename.endswith(ext) for ext in EXTENSIONS_PCD)


class CalibApp:
    def __init__(self, folder):
        self.lla = dict()
        self.selected = dict()
        self.feature = dict()
        # I just dont want to check if the last charactor is '/' in C++
        self.address = os.path.join(folder, '')
        self.select_yaml_file = os.path.join(self.address, "selected.yaml")
        self.feature_yaml_file = os.path.join(self.address, "feature.yaml")
        self.lla_yaml_file = os.path.join(self.address, "gps.yaml")
        self.result_yaml_file = os.path.join(self.address, "result.yaml")
        self.analyse_txt_file = os.path.join(self.address, "analyse.txt")

    def compute(self):
        if (os.path.exists(self.feature_yaml_file) & os.path.exists(self.lla_yaml_file)):
            self.load()
            self.solve()
        else:
            if not os.path.exists(self.select_yaml_file):
                self.select()
            if not os.path.exists(self.feature_yaml_file):
                self.extract()
            if (os.path.exists(self.feature_yaml_file) & os.path.exists(self.lla_yaml_file)):
                self.load()
                self.solve()
            else:
                print("Lack of feature yaml or gps yaml")

    def select(self):
        print("1) Please pick one mark point using [shift + left click]")
        print("   Press [shift + right click] to undo point picking")
        print("2) After picking points, press 'Q' to pass to next frame")
        all_file = os.listdir(self.address)
        pcd_file = [f for f in all_file if is_pcd(f)]
        pcd_file.sort()
        print("Find ", len(pcd_file), " pcds.")
        for id in tqdm(range(len(pcd_file))):
            pc = get_geometry(os.path.join(self.address, pcd_file[id]))
            vis = o3d.visualization.VisualizerWithEditing()
            vis.create_window(pcd_file[id], 1024, 768)
            #############################################################################
            render_option = vis.get_render_option()
            render_option.background_color = np.array([0, 0, 0])
            render_option.point_size = 1.0
            #############################################################################
            vis.add_geometry(pc)
            vis.run()
            vis.destroy_window()
            select_queue = vis.get_picked_points()
            #############################################################################
            # (TODO: This a bug of Open3D : https://github.com/isl-org/Open3D/issues/4960)
            del render_option
            del vis
            #############################################################################
            assert len(
                select_queue) <= 1, "[SELECT] At most one point could be selected in single frame"
            if len(select_queue) == 1:
                current_point = np.asarray(pc.points, dtype=np.float64)[
                    select_queue][0]
                print(current_point)
                self.selected.update({pcd_file[id]: {"x": float(current_point[0]),
                                                     "y": float(current_point[1]),
                                                     "z": float(current_point[2])}})
            else:
                print("Passed")
        print("Collected ", len(self.selected),
              " points, generating yaml file")
        with open(self.select_yaml_file, 'w') as file:
            file.write(yaml.dump(self.selected, sort_keys=False))

    def extract(self):
        # import c++ programe use selected.yaml to generate feature.yaml
        trace(EXCUTABLE_BIN + self.address)
        process = subprocess.Popen(
            EXCUTABLE_BIN + self.address, shell=True)
        while (process.poll() is None):
            time.sleep(1)
        os.system("clear")
        trace("Finished choose feature point, return stats: ", process.poll())
        process.terminate()

    def load(self):
        if yaml.__version__ >= '5.1':
            dict_feature = yaml.load(
                open(self.feature_yaml_file), Loader=yaml.FullLoader)
            dict_gps = yaml.load(open(self.lla_yaml_file),
                                 Loader=yaml.FullLoader)
        else:
            dict_feature = yaml.load(
                open(self.feature_yaml_file))
            dict_gps = yaml.load(open(self.lla_yaml_file))
        for key, value in dict_feature.items():
            self.feature.update({key: [value['x'], value['y'], value['z']]})
        for key, value in dict_gps.items():
            self.lla.update(
                {key: [value['longtitude'], value['latitude'], value['altitude']]})
        debug("Feature", self.feature)
        debug("GPS", self.lla)

    def solve(self):
        # load feature and gps yaml
        coor_lla = []
        coor_cen = []
        for key in self.feature:
            if key in self.lla:
                coor_cen.append(self.feature[key])
                coor_lla.append(self.lla.pop(key))
        coor_cen = np.array(coor_cen, dtype=np.float64)
        coor_lla = np.array(coor_lla, dtype=np.float64)
        if (coor_cen.shape[0] >= 4):
            assert coor_cen.shape == coor_lla.shape, "[SOLVE] Shape dont match: feature and lla"
            coor_ecef = lla_to_ecef(coor_lla)
            assert coor_cen.shape == coor_ecef.shape, "[SOLVE] Shape dont match: feature and ecef"
            best_candidate = best_RT(coor_ecef, coor_cen)
            best_candidate.print()
            self.save(best_candidate)
        else:
            trace("Input point less than 4")

    def save(self, result):
        rtm3x4 = np.concatenate((result.r, np.transpose(result.t)), axis=1)
        rtm4x4 = np.concatenate(
            (rtm3x4, np.array([0, 0, 0, 1]).reshape(1, 4)), axis=0)
        output = dict({
            "version": VERSION,
            "project": PROJECT,
            "data": datetime.date.today(),
            "ToGps": dict({"transform": str_matrix(rtm4x4)})
        })
        with open(self.result_yaml_file, 'w') as file:
            file.write(yaml.dump(output, sort_keys=False))
        with open(self.analyse_txt_file, 'w') as file:
            file.write(result.output())


if __name__ == "__main__":
    base_addr = os.listdir(PCDS_ADDR)
    for folder in base_addr:
        temp_full_addr = os.path.join(PCDS_ADDR, folder)
        if os.path.isdir(temp_full_addr):
            opera = CalibApp(temp_full_addr)
            opera.compute()
