import numpy as np

class MyTf:
    def __init__(self, x, y, yaw):
        """
        x, y: 平移
        yaw: 旋转角，单位 rad
        """
        self.x = x
        self.y = y
        self.yaw = yaw

        # 预计算旋转矩阵
        c = np.cos(yaw)
        s = np.sin(yaw)
        self.R = np.array([
            [c, -s],
            [s,  c]
        ])

        # 平移向量
        self.t = np.array([x, y])

    def transform(self, points):
        """
        将点从局部坐标系 -> 经过该刚体变换后的坐标系

        points: (2,) or (N,2)
        """
        points = np.atleast_2d(points)
        return (self.R @ points.T).T + self.t

    def inverse(self):
        """
        返回逆变换对象
        """
        R_inv = self.R.T   # 旋转矩阵逆就是转置
        t_inv = -R_inv @ self.t

        # 计算逆的 yaw
        yaw_inv = -self.yaw

        inv_tf = MyTf(0, 0, 0)
        inv_tf.R = R_inv
        inv_tf.t = t_inv
        inv_tf.x = t_inv[0]
        inv_tf.y = t_inv[1]
        inv_tf.yaw = yaw_inv
        return inv_tf

    def inverse_transform(self, points):
        """
        等价于先求逆再transform
        """
        inv = self.inverse()
        return inv.transform(points)

# ======= 用法示例 =======
if __name__ == "__main__":
    tf = MyTf(1.0, 2.0, np.deg2rad(90))

    pt = np.array([1.0, 0.0])
    p_global = tf.transform(pt)
    print("Transformed:", p_global)

    p_local = tf.inverse_transform(p_global)
    print("Inverse transformed:", p_local)