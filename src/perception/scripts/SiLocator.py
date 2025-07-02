import math
from typing import List, Tuple

# 定义常量
SICK_NUMS = 4
PI = math.pi
SICK_DISTAN_TO_CENTER_METER = 0.0
COURT_WIDE = 8.0
COURT_LENGTH = 15.0


# 定义数据结构
class Vec3:
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"Vec3({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"

    def __add__(self, other: 'Vec3') -> 'Vec3':
        """向量加法"""
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'Vec3') -> 'Vec3':
        """向量减法"""
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> 'Vec3':
        """标量乘法 (向量 * 标量)"""
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> 'Vec3':
        """反向标量乘法 (标量 * 向量)"""
        return self.__mul__(scalar)

    def dot(self, other: 'Vec3') -> float:
        """向量点积"""
        return self.x * other.x + self.y * other.y + self.z * other.z

    def length(self) -> float:
        """向量长度"""
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def as_tuple(self) -> Tuple[float, float, float]:
        """转换为元组形式"""
        return (self.x, self.y, self.z)


class SickData:
    __slots__ = ('id', 'distan', 'theta', 'error')

    def __init__(self, sick_id=0, distan=0.0, theta=0.0, error=0.0):
        self.id = sick_id
        self.distan = distan
        self.theta = theta
        self.error = error


class SiLocator:
    def __init__(self):
        self.sickdatas_list: List[SickData] = [SickData(i) for i in range(SICK_NUMS)]
        self.learning_rate = 0.00001
        self.derivation_rate = 0.0001
        self.total_cost = 0.0
        self.feedback_cnt = 0
        self.feedback_intv = 3

    @staticmethod
    def normalize(v: Vec3) -> Vec3:
        """向量归一化"""
        magnitude = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        if magnitude == 0:
            return Vec3(0, 0, 0)
        return Vec3(v.x / magnitude, v.y / magnitude, v.z / magnitude)

    @staticmethod
    def normalize_xy(v: Vec3) -> Vec3:
        """仅归一化向量的x和y分量，z分量保持不变"""
        magnitude_xy = math.sqrt(v.x ** 2 + v.y ** 2)
        if magnitude_xy == 0:
            return Vec3(0, 0, v.z)  # x和y为0时返回(0,0,z)
        return Vec3(v.x / magnitude_xy, v.y / magnitude_xy, v.z)

    @staticmethod
    def get_radius(x: float, y: float, theta: float, length: float, width: float) -> float:
        """计算在特定位置和角度的理论距离"""
        theta = theta % (2 * PI)

        # 计算四个关键角度
        theta_0 = math.pi / 2 if (length - x == 0) else math.atan(y / (length - x))
        theta_1 = math.pi / 2 if y == 0 else math.atan(x / y) + math.pi / 2
        theta_2 = math.pi if x == 0 else math.atan((width - y) / x) + math.pi
        theta_3 = 3 * math.pi / 2 if (width - y == 0) else math.atan((length - x) / (width - y)) + 3 * math.pi / 2

        # 根据角度区间返回不同边的距离
        if theta < theta_0:
            return (length - x) / math.cos(theta)
        if theta < theta_1:
            return y / math.cos(math.pi / 2 - theta)
        if theta < theta_2:
            return x / math.cos(math.pi - theta)
        if theta < theta_3:
            return (width - y) / math.cos(3 * math.pi / 2 - theta)
        return (length - x) / math.cos(theta)

    def cost_func(self, prd_x: float, prd_y: float, prd_fai: float,
                  court_len: float, court_wid: float) -> float:
        """计算代价函数"""
        self.total_cost = 0.0

        for sick in self.sickdatas_list:
            # 过滤无效数据
            if sick.distan > 12 or sick.distan <= 0:
                continue

            # 计算理论距离和误差
            theoryR = self.get_radius(
                prd_x, prd_y,
                sick.theta + prd_fai,
                court_len, court_wid
            )

            test_error = abs(theoryR - sick.distan)
            sick.error = test_error

            # 累加总误差
            self.total_cost += test_error

        return self.total_cost

    def grad_decent(self, chassis_tf: Vec3, silocator_tf: Vec3) -> None:
        """执行梯度下降优化"""
        cost_base = self.cost_func(
            chassis_tf.x, chassis_tf.y, chassis_tf.z,
            COURT_LENGTH, COURT_WIDE
        )

        # 计算X方向偏导
        cx = self.cost_func(
            chassis_tf.x + self.derivation_rate,
            chassis_tf.y,
            chassis_tf.z,
            COURT_LENGTH, COURT_WIDE)

        dx = (cost_base - cx) / self.derivation_rate

        # 计算Y方向偏导
        cy = self.cost_func(
            chassis_tf.x,
            chassis_tf.y + self.derivation_rate,
            chassis_tf.z,
            COURT_LENGTH, COURT_WIDE)

        dy = (cost_base - cy) / self.derivation_rate


        # 计算角度偏导
        cf = self.cost_func(
            chassis_tf.x,
            chassis_tf.y,
            chassis_tf.z + (self.derivation_rate),
            COURT_LENGTH, COURT_WIDE)

        df = (cost_base - cf) / (self.derivation_rate)

        origin_vec = Vec3(dx, dy, df)
        # 归一化梯度向量
        delta_vec = self.normalize_xy(Vec3(dx, dy, df))

        # 更新位姿估计
        silocator_tf.x += origin_vec.x * self.learning_rate
        silocator_tf.y += origin_vec.y * self.learning_rate
        silocator_tf.z += origin_vec.z * self.learning_rate

        return delta_vec, origin_vec, cost_base

    def update_sick_data(self, sick_id: int, distan: float, theta_deg: float) -> None:
        """更新激光传感器数据"""
        if 0 <= sick_id < SICK_NUMS:
            self.sickdatas_list[sick_id].distan = distan + SICK_DISTAN_TO_CENTER_METER
            self.sickdatas_list[sick_id].theta = math.radians(theta_deg)

    def test_getradius(self):
        for i in range(36):
            print("角度：", 360 / 36 * i, "长度：", self.get_radius(3, 3, math.radians(360 / 36 * i), COURT_LENGTH, COURT_WIDE))