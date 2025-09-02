import numpy as np

class FlexibleKalmanFilter:
    """
    灵活的卡尔曼滤波器实现，支持不同维度的测量更新
    """
    def __init__(self, dim_x, initial_x=None, initial_P=None, Q=None, F=None, B=None):
        """
        初始化卡尔曼滤波器
        
        参数:
            dim_x: 状态向量维度
            initial_x: 初始状态估计 (dim_x, 1)
            initial_P: 初始协方差矩阵 (dim_x, dim_x)
            Q: 过程噪声协方差矩阵 (dim_x, dim_x)
            F: 状态转移矩阵 (dim_x, dim_x)
            B: 控制输入矩阵 (dim_x, dim_u)
        """
        self.dim_x = dim_x
        
        # 初始化状态估计
        self.x = np.zeros((dim_x, 1)) if initial_x is None else initial_x.reshape(dim_x, 1)
        
        # 初始化协方差矩阵
        self.P = np.eye(dim_x) if initial_P is None else initial_P
        
        # 初始化过程噪声
        self.Q = np.eye(dim_x) if Q is None else Q
        
        # 初始化状态转移矩阵
        self.F = np.eye(dim_x) if F is None else F
        
        # 初始化控制输入矩阵
        self.B = np.zeros((dim_x, 1)) if B is None else B
        
        # 存储最近一次的计算结果
        self.K = None  # 卡尔曼增益
        self.y = None  # 残差
        self.S = None  # 系统不确定性
        self.likelihood = None  # 似然度
        self.log_likelihood = None  # 对数似然度
        self.H = None  # 测量矩阵
        self.R=None
    def predict(self, u=None):
        """
        预测步骤: 基于上一状态估计下一状态
        
        参数:
            u: 控制输入向量 (dim_u, 1)
        """
        # 状态预测: x = Fx + Bu
        if u is not None:
            self.x = self.F @ self.x + self.B @ u
        else:
            self.x = self.F @ self.x
            
        # 协方差预测: P = FPF' + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x.copy()
    
    def update(self, z, H=None, R=None, check_valid=True):
        """
        更新步骤: 融合预测状态和测量值
        
        参数:
            z: 测量向量 (dim_z, 1)
            H: 测量矩阵 (dim_z, dim_x)
            R: 测量噪声协方差矩阵 (dim_z, dim_z)
            check_valid: 是否检查输入维度有效性
        """
        if H is None:
            H=self.H

        if R is None:
            if self.R is None:
                self.R = np.eye(z.shape[0])  # 默认单位矩阵
            R= self.R
        # 检查输入有效性
        if check_valid:
            dim_z = z.shape[0]
            assert H.shape == (dim_z, self.dim_x), f"H 矩阵形状错误，应为 ({dim_z}, {self.dim_x})"
            assert R.shape == (dim_z, dim_z), f"R 矩阵形状错误，应为 ({dim_z}, {dim_z})"
     
        # 计算残差 (测量与预测的差值)
        y = z - H @ self.x
        self.y = y
        
        # 计算卡尔曼增益
        S = H @ self.P @ H.T + R
        self.S = S
        K = self.P @ H.T @ np.linalg.inv(S)
        self.K = K
        
        # 更新状态估计
        self.x = self.x + K @ y
        
        # 更新协方差矩阵
        I = np.eye(self.dim_x)
        self.P = (I - K @ H) @ self.P
        
        # 计算似然度
        self.likelihood = self._calculate_likelihood(y, S)
        self.log_likelihood = np.log(self.likelihood) if self.likelihood > 0 else -np.inf
        
        return self.x.copy()
    
    def _calculate_likelihood(self, y, S):
        """计算测量的似然度"""
        # 多元高斯分布的似然度
        n = y.shape[0]
        det_S = np.linalg.det(S)
        if det_S <= 0:
            # 处理奇异矩阵的情况
            return 0.0
        
        exponent = -0.5 * (y.T @ np.linalg.inv(S) @ y)
        return (1.0 / np.sqrt((2 * np.pi) ** n * det_S)) * np.exp(exponent[0, 0])
    

class ExponentialMovingAverageFilter:
    def __init__(self, alpha=0.3):
        """
        指数加权移动平均滤波器
        :param alpha: 平滑因子(0 < alpha < 1)，越小越平滑
        """
        self.alpha = alpha
        self.filtered_value = None
    
    def update(self, new_value):
        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

class MovingAverageFilter:
    def __init__(self, window_size=5):
        """
        移动平均滤波器
        :param window_size: 窗口大小，决定平滑程度
        """
        self.window_size = window_size
        self.buffer = []

    def update(self, new_value):
        """
        更新滤波值
        :param new_value: 新输入值
        :return: 滤波后的值
        """
        self.buffer.append(new_value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)

    def reset(self):
        """重置滤波器状态"""
        self.buffer = []
