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

    def transform(self, x, y):
        """
        将单个点从局部坐标系 -> 经过该刚体变换后的坐标系

        输入: x, y (标量)
        输出: x_new, y_new (标量)
        """
        p = np.array([x, y])
        p_new = self.R @ p + self.t
        return p_new[0], p_new[1]

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

    def inverse_transform(self, x, y):
        """
        等价于先求逆再 transform
        """
        inv = self.inverse()
        return inv.transform(x, y)

# ======= 用法示例 =======
if __name__ == "__main__":
    tf = MyTf(1.0, 2.0, np.deg2rad(90))

    x_in, y_in = 1.0, 0.0
    x_global, y_global = tf.transform(x_in, y_in)
    print("Transformed:", x_global, y_global)

    x_local, y_local = tf.inverse_transform(x_global, y_global)
    print("Inverse transformed:", x_local, y_local)
