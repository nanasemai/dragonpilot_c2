#!/usr/bin/env python3
import os
import time
import numpy as np
from casadi import SX, vertcat, sin, cos
# from common.realtime import sec_since_boot
# from selfdrive.controls.lib.drive_helpers import LAT_MPC_N as N
from openpilot.selfdrive.legacy_modeld.constants import T_IDXS
if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.legacy_lateral_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython  # pylint: disable=no-name-in-module, import-error
# 添加导入
from openpilot.selfdrive.controls.lib.legacy_lateral_mpc_lib.lat_mpc_params import LateralParams
LAT_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LAT_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LAT_MPC_DIR, "acados_ocp_lat.json")
X_DIM = 4
P_DIM = 2
MODEL_NAME = 'lat'
ACADOS_SOLVER_TYPE = 'SQP_RTI'
N = 16
def gen_lat_model():
    """生成横向控制模型
    创建一个包含车辆状态、控制输入和动力学方程的模型
    状态变量包括:
    - x_ego: 车辆x坐标
    - y_ego: 车辆y坐标
    - psi_ego: 车辆航向角
    - curv_ego: 当前曲率
    """
    model = AcadosModel()
    model.name = MODEL_NAME

    # 定义状态变量
    x_ego = SX.sym('x_ego')      # 车辆x坐标
    y_ego = SX.sym('y_ego')      # 车辆y坐标
    psi_ego = SX.sym('psi_ego')  # 车辆航向角
    curv_ego = SX.sym('curv_ego')# 当前曲率
    model.x = vertcat(x_ego, y_ego, psi_ego, curv_ego)

    # 定义模型参数
    v_ego = SX.sym('v_ego')              # 车速
    rotation_radius = SX.sym('rotation_radius')  # 转向半径
    model.p = vertcat(v_ego, rotation_radius)

    # 定义控制输入
    curv_rate = SX.sym('curv_rate')      # 曲率变化率
    model.u = vertcat(curv_rate)

    # 定义状态导数
    x_ego_dot = SX.sym('x_ego_dot')      # x坐标变化率
    y_ego_dot = SX.sym('y_ego_dot')      # y坐标变化率
    psi_ego_dot = SX.sym('psi_ego_dot')  # 航向角变化率
    curv_ego_dot = SX.sym('curv_ego_dot')# 曲率变化率
    model.xdot = vertcat(x_ego_dot, y_ego_dot, psi_ego_dot, curv_ego_dot)

    # 定义系统动力学方程
    f_expl = vertcat(
        v_ego * cos(psi_ego) - rotation_radius * sin(psi_ego) * (v_ego * curv_ego),  # x方向运动
        v_ego * sin(psi_ego) + rotation_radius * cos(psi_ego) * (v_ego * curv_ego),  # y方向运动
        v_ego * curv_ego,                                                             # 航向角变化
        curv_rate                                                                     # 曲率变化
    )
    model.f_impl_expr = model.xdot - f_expl  # 隐式表达式
    model.f_expl_expr = f_expl               # 显式表达式
    return model

def gen_lat_ocp():
    """生成最优控制问题
    设置求解器参数、约束条件和成本函数
    """
    ocp = AcadosOcp()
    ocp.model = gen_lat_model()

    # 设置预测时域
    Tf = np.array(T_IDXS)[N]
    ocp.dims.N = N

    # 设置成本函数类型
    ocp.cost.cost_type = 'NONLINEAR_LS'    # 非线性最小二乘
    ocp.cost.cost_type_e = 'NONLINEAR_LS'  # 终端成本

    # 设置权重矩阵
    Q = np.diag([0.0, 0.0])                # 终端状态权重
    QR = np.diag([0.0, 0.0, 0.0])         # 状态和控制权重
    ocp.cost.W = QR
    ocp.cost.W_e = Q

    # 提取模型变量
    y_ego, psi_ego = ocp.model.x[1], ocp.model.x[2]
    curv_rate = ocp.model.u[0]
    v_ego = ocp.model.p[0]

    ocp.parameter_values = np.zeros((P_DIM,))

    ocp.cost.yref = np.zeros((3,))
    ocp.cost.yref_e = np.zeros((2,))

    # 设置成本函数表达式
    ocp.model.cost_y_expr = vertcat(
        y_ego,                          # 横向偏差成本
        ((v_ego +5.0) * psi_ego),      # 航向角成本
        ((v_ego +5.0) * 4 * curv_rate) # 转向率成本
    )
    ocp.model.cost_y_expr_e = vertcat(
        y_ego,                         # 终端横向偏差成本
        ((v_ego +5.0) * psi_ego)      # 终端航向角成本
    )

    # 设置约束条件
    ocp.constraints.constr_type = 'BGH'
    ocp.constraints.idxbx = np.array([2,3])  # 约束航向角和曲率
    ocp.constraints.ubx = np.array([np.radians(90), np.radians(50)])  # 上限
    ocp.constraints.lbx = np.array([-np.radians(90), -np.radians(50)])# 下限
    x0 = np.zeros((X_DIM,))
    ocp.constraints.x0 = x0

    # 设置求解器选项
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
    ocp.solver_options.qp_solver_iter_max = 1
    ocp.solver_options.qp_solver_cond_N = 1

    # set prediction horizon
    ocp.solver_options.tf = Tf
    ocp.solver_options.shooting_nodes = np.array(T_IDXS)[:N + 1]

    ocp.code_export_directory = EXPORT_DIR

    return ocp

class LateralMpc():
    def __init__(self, x0=np.zeros(X_DIM)):
        """初始化MPC控制器"""
        self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
        self.params = LateralParams()  # 加载控制参数
        self.reset(x0)

    def update_personality(self, personality):
        """更新驾驶风格参数
        根据选择的驾驶风格(从容/标准/激进)更新控制参数
        """
        self.params = LateralParams(personality)
        self.update_constraints()

    def reset(self, x0=np.zeros(X_DIM)):
      self.x_sol = np.zeros((N + 1, X_DIM))
      self.u_sol = np.zeros((N, 1))
      self.yref = np.zeros((N + 1, 3))
      for i in range(N):
        self.solver.cost_set(i, "yref", self.yref[i])
      self.solver.cost_set(N, "yref", self.yref[N][:2])

      # Somehow needed for stable init
      for i in range(N + 1):
        self.solver.set(i, 'x', np.zeros(X_DIM))
        self.solver.set(i, 'p', np.zeros(P_DIM))
      self.solver.constraints_set(0, "lbx", x0)
      self.solver.constraints_set(0, "ubx", x0)
      self.solver.solve()
      self.solution_status = 0
      self.solve_time = 0.0
      self.cost = 0

    def set_weights(self, path_weight, heading_weight, steer_rate_weight):
      W = np.asfortranarray(np.diag([path_weight, heading_weight, steer_rate_weight]))
      for i in range(N):
        self.solver.cost_set(i, 'W', W)
      # TODO hacky weights to keep behavior the same
      self.solver.cost_set(N, 'W', (3 / 20.) * W[:2, :2])

    def run(self, x0, p, y_pts, heading_pts):
        """运行MPC求解器
        参数:
            x0: 当前状态
            p: 模型参数(车速等)
            y_pts: 期望横向位置序列
            heading_pts: 期望航向角序列
        """
        x0_cp = np.copy(x0)
        p_cp = np.copy(p)
        v_ego = p_cp[0]

        # 计算当前曲率
        current_curvature = abs(x0_cp[3]) if len(x0_cp) > 3 else 0.0

        # 动态调整权重
        path_w = self.params.current['PATH_WEIGHT'] * (1 + current_curvature * self.params.current['CURVE_FACTOR'])
        heading_w = self.params.current['HEADING_WEIGHT'] * (1 + v_ego * 0.02)
        steer_rate_w = self.params.current['STEER_RATE_WEIGHT']

        # 更新权重
        self.set_weights(path_w, heading_w, steer_rate_w)

        self.solver.constraints_set(0, "lbx", x0_cp)
        self.solver.constraints_set(0, "ubx", x0_cp)
        self.yref[:,0] = y_pts
        v_ego = p_cp[0]
        # rotation_radius = p_cp[1]
        self.yref[:,1] = heading_pts*(v_ego+5.0)
        for i in range(N):
          self.solver.cost_set(i, "yref", self.yref[i])
          self.solver.set(i, "p", p_cp)
        self.solver.set(N, "p", p_cp)
        self.solver.cost_set(N, "yref", self.yref[N][:2])

        t = time.monotonic()
        self.solution_status = self.solver.solve()
        self.solve_time = time.monotonic() - t

        for i in range(N+1):
          self.x_sol[i] = self.solver.get(i, 'x')
        for i in range(N):
          self.u_sol[i] = self.solver.get(i, 'u')
        self.cost = self.solver.get_cost()


if __name__ == "__main__":
  ocp = gen_lat_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
