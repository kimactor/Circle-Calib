import open3d as o3d
import numpy as np
import math
import time
import itertools

DEBUG = True
DOWNSAMPLEVOXEL = 0.1
ERROR_THRESHOLD = 0.08

def trace(*txt):
    print("\033[0;37;41m\t" + str(txt) + "\033[0m")


def debug(header, msg, tail=None):
    if DEBUG:
        print("\033[0;37;41m\t" + header + "\033[0m")
        print(msg)
        if tail:
            print("\033[0;37;41m\t" + tail + "\033[0m")


intensity_to_RGB = np.array([[0,   8, 255],
                             [0,  17, 255],
                             [0,  25, 255],
                             [0,  34, 255],
                             [0,  42, 255],
                             [0,  51, 255],
                             [0,  59, 255],
                             [0,  68, 255],
                             [0,  76, 255],
                             [0,  85, 255],
                             [0,  93, 255],
                             [0, 102, 255],
                             [0, 110, 255],
                             [0, 119, 255],
                             [0, 127, 255],
                             [0, 136, 255],
                             [0, 144, 255],
                             [0, 153, 255],
                             [0, 161, 255],
                             [0, 170, 255],
                             [0, 178, 255],
                             [0, 187, 255],
                             [0, 195, 255],
                             [0, 204, 255],
                             [0, 212, 255],
                             [0, 221, 255],
                             [0, 229, 255],
                             [0, 238, 255],
                             [0, 246, 255],
                             [0, 255, 255],
                             [0, 255, 247],
                             [0, 255, 238],
                             [0, 255, 230],
                             [0, 255, 221],
                             [0, 255, 213],
                             [0, 255, 204],
                             [0, 255, 196],
                             [0, 255, 187],
                             [0, 255, 179],
                             [0, 255, 170],
                             [0, 255, 162],
                             [0, 255, 153],
                             [0, 255, 145],
                             [0, 255, 136],
                             [0, 255, 128],
                             [0, 255, 119],
                             [0, 255, 111],
                             [0, 255, 102],
                             [0, 255,  94],
                             [0, 255,  85],
                             [0, 255,  77],
                             [0, 255,  68],
                             [0, 255,  60],
                             [0, 255,  51],
                             [0, 255,  43],
                             [0, 255,  34],
                             [0, 255,  26],
                             [0, 255,  17],
                             [0, 255,   9],
                             [0, 255,   0],
                             [8, 255,   0],
                             [17, 255,   0],
                             [25, 255,   0],
                             [34, 255,   0],
                             [42, 255,   0],
                             [51, 255,   0],
                             [59, 255,   0],
                             [68, 255,   0],
                             [76, 255,   0],
                             [85, 255,   0],
                             [93, 255,   0],
                             [102, 255,   0],
                             [110, 255,   0],
                             [119, 255,   0],
                             [127, 255,   0],
                             [136, 255,   0],
                             [144, 255,   0],
                             [153, 255,   0],
                             [161, 255,   0],
                             [170, 255,   0],
                             [178, 255,   0],
                             [187, 255,   0],
                             [196, 255,   0],
                             [204, 255,   0],
                             [212, 255,   0],
                             [221, 255,   0],
                             [230, 255,   0],
                             [238, 255,   0],
                             [246, 255,   0],
                             [255, 255,   0],
                             [255, 247,   0],
                             [255, 238,   0],
                             [255, 230,   0],
                             [255, 221,   0],
                             [255, 213,   0],
                             [255, 204,   0],
                             [255, 196,   0],
                             [255, 187,   0],
                             [255, 179,   0],
                             [255, 170,   0],
                             [255, 162,   0],
                             [255, 153,   0],
                             [255, 145,   0],
                             [255, 136,   0],
                             [255, 128,   0],
                             [255, 119,   0],
                             [255, 111,   0],
                             [255, 102,   0],
                             [255,  94,   0],
                             [255,  85,   0],
                             [255,  76,   0],
                             [255,  68,   0],
                             [255,  60,   0],
                             [255,  51,   0],
                             [255,  43,   0],
                             [255,  34,   0],
                             [255,  26,   0],
                             [255,  17,   0],
                             [255,   9,   0],  # [255,   9,   0]
                             [255,   0,   0]])


def read_pcd(pcd_path):
    lines = []
    num_points = None
    with open(pcd_path, 'r') as f:
        for line in f:
            lines.append(line.strip())
            if line.startswith('POINTS'):
                num_points = int(line.split()[-1])
    assert num_points is not None, "[Load PCD] Failed to load point number"
    points = []
    for line in lines[-num_points:]:
        x, y, z, i = list(map(float, line.split()))
        if (i > 250 or i < 100 and math.sqrt(x*x + y*y + z*z) < 100):
            points.append((np.array([x, y, z, i])))
    return points


def get_geometry(pcd_path):
    pcd = o3d.geometry.PointCloud()
    # t1 = time.time_ns()
    PCXYZI = read_pcd(pcd_path)
    # t2 = time.time_ns()
    XYZI = np.asarray(PCXYZI)
    # t3 = time.time_ns()

    # trace("Read pcd used : ", t2 - t1)
    # trace("Take as array used : ", t3 - t2)
    point_size = XYZI.shape[0]
    # t1 = time.time_ns()
    pcd.points = o3d.utility.Vector3dVector(XYZI[:, :3])
    # t2 = time.time_ns()
    # trace("o3d.utility.Vector3dVector used : ", t2 - t1)
    RGB = np.zeros([point_size, 3])
    pcd.colors.clear()
    # t1 = time.time_ns()
    for i in range(point_size):
        index = int(XYZI[i][3]/255*120)-1
        pcd.colors.append(intensity_to_RGB[index])
    # t2 = time.time_ns()
    print(len(pcd.points))
    # t3 = time.time_ns()
    downpcd = pcd.voxel_down_sample(voxel_size=DOWNSAMPLEVOXEL)
    # t4 = time.time_ns()
    print(len(downpcd.points))
    # trace("RGB used : ", t2-t1)
    # trace("Down sampling used : ", t4 - t3)
    return downpcd


def solve_svg(f_1, f_2):
    # R@f_2 + T = f_1
    assert f_1.shape == f_2.shape, "[SVG] Shape dont match: topose and frompose"
    assert f_1.shape[0] >= 4, "[SVG] Doesn't have enough mark points"
    assert f_1.shape[1] == 3, "[SVG] Wrong input point size"
    center_1 = f_1 - np.mean(f_1, axis=0)
    center_2 = f_2 - np.mean(f_2, axis=0)

    H = np.transpose(center_1)@center_2
    U, W, VT = np.linalg.svd(H)
    R = U@VT
    if np.linalg.det(R) < 0:
        VT[:] = [[1, 0, 0],
                 [0, 1, 0],
                 [0, 0, -1]]@VT
        R[:] = U@VT
    T = np.mean(f_1 - np.dot(f_2, np.transpose(R)), axis=0)
    return R, T

# LLA(lon,lat,alt) to ECEF (Earth-Centred Earth Fixed)
def lla_to_ecef(lla):
    ecef = []
    # f = 1 / 298.257223563                  # WGS84椭球扁率
    r = 6378137                              # 长半轴
    # b = r * (1 - f)                        # 椭球扁率
    e = 8.1819190842622e-2                   # 椭球第一偏心率
    esq = math.pow(e, 2)
    # LLA : longtitude, latitude, altitude
    # ECEF: X(), Y(), Z(North Pole)
    for item in lla:
        lon = math.radians(item[0])
        lat = math.radians(item[1])
        alt = item[2]

        sin_lat = math.sin(lat)
        cos_lat = math.cos(lat)
        sin_lon = math.sin(lon)
        cos_lon = math.cos(lon)
        N = r / math.sqrt(1 - esq * math.pow(sin_lat, 2))

        ecef.append([(N+alt)*cos_lat*cos_lon,
                    (N+alt)*cos_lat*sin_lon,
                    (N*(1-esq) + alt)*sin_lat])
    ecef = np.array(ecef, dtype=np.float64)
    assert ecef.shape == lla.shape, "[LLA2ECEF] Shape dont match: ECEF and LLA"
    return ecef


def str_matrix(m):
    assert type(m) is np.ndarray, "Input should be numpy.array"
    # M1
    # ring=""
    # for i in m.flat:
    #     ring += str(i)+" "
    # M2
    ring = str(m)
    ring = ring.replace("[", '')
    ring = ring.replace("]", '')
    ring = ring.replace("\n", '')
    return ring


class candidate:
    def __init__(self, r=None, t=None, topose=None, frompose=None):
        self.r = None
        error_threshold = ERROR_THRESHOLD
        if r is not None:
            self.r = np.array(r).reshape(3, 3)
            self.t = np.array(t).reshape(1, 3)
            self.e_coor = topose - (frompose@np.transpose(r) + t)
            self.e_dis = [math.sqrt(math.pow(e[0], 2) + math.pow(e[1],
                                                                 2) + math.pow(e[2], 2)) for e in self.e_coor]
            self.e_avg = np.mean(self.e_dis, axis=0)
            self.e_valid = np.count_nonzero(
                [(e < error_threshold) for e in self.e_dis])
            self.e_std = math.sqrt(
                np.sum(np.power(self.e_dis-self.e_avg, 2))/len(self.e_dis))
        else:
            self.e_avg = 1e5
            self.e_valid = 0
            self.e_std = 1e5

    def better(self, other):
        if other.r is None:
            return self
        if other.e_valid > self.e_valid:
            return other
        if other.e_avg < self.e_avg:
            return other
        if other.e_std < self.e_std:
            return other
        return other

    def print(self):
        trace("Best solution")
        debug("Rotation ", self.r)
        debug("Translation ", self.t)
        debug("Axis error ", self.e_coor)
        debug("Distance error ", self.e_dis)
        debug("Average error ", self.e_avg)
        debug("Inline points ", self.e_valid)
        debug("Std error ", self.e_std)

    def output(self):
        res = "Best solution\n" \
        + "Rotation :\n"      + str(self.r)      + "\n" \
        + "Translation: \n"   + str(self.t)      + "\n" \
        + "Axis error: \n"    + str(self.e_coor) + "\n" \
        + "Distance error: \n"+ str(self.e_dis)  + "\n" \
        + "Average error: \n" + str(self.e_avg)  + "\n" \
        + "Inline points: \n" + str(self.e_valid)+ "\n" \
        + "Std error: \n"     + str(self.e_std)
        return res


# (TODO: need to eliminate outliner points)
def best_RT(topose, frompose):
    assert topose.shape == frompose.shape, "Shape dont match: topose and frompose"
    seed = 4
    res = candidate()
    count = 0
    for picked in list(itertools.combinations(range(topose.shape[0]), seed)):
        to_temp = topose[picked, :]
        from_temp = frompose[picked, :]
        r, t = solve_svg(to_temp, from_temp)
        temp = candidate(r, t, topose, frompose)
        res = res.better(temp)
        count += 1
    trace("Combination times" + str(count))
    return res
