#!/usr/bin/env python3
import os
import time
import numpy as np
from cereal import log

from openpilot.common.numpy_fast import clip, interp
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.legacy_modeld.constants import index_function
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.legacy_longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython  # pylint: disable=no-name-in-module, import-error

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise']

X_DIM = 3      # 状态维度：[位置, 速度, 加速度]
U_DIM = 1      # 控制维度：[加加速度]
PARAM_DIM = 5  # 参数维度：[最小加速度, 最大加速度, 障碍物位置, 上一次加速度, 跟车时距]
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

# 成本函数权重
X_EGO_OBSTACLE_COST = 3.   # 与障碍物距离偏差成本
X_EGO_COST = 0.           # 位置偏差成本
V_EGO_COST = 0.           # 速度偏差成本
A_EGO_COST = 0.           # 加速度偏差成本
J_EGO_COST = 5.0          # 加加速度成本
A_CHANGE_COST = 200.      # 加速度变化成本
DANGER_ZONE_COST = 100.   # 危险区域成本
CRASH_DISTANCE = .5
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'
N = 12
MAX_T = 10.0
MIN_ACCEL = -3.5
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0

T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]
T_IDXS = np.array(T_IDXS_LST)
T_DIFFS = np.diff(T_IDXS, prepend=[0.])


def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.standard:
    return 1.0
  elif personality==log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  """获取基础跟车时距
  该函数根据不同的驾驶风格返回对应的基础跟车时距值。
  这个时距将被用于计算理想跟车距离，是一个重要的安全参数。
  参数:
    personality: 驾驶风格，包括从容(relaxed)、标准(standard)和激进(aggressive)三种模式
  返回值:
    float: 基础跟车时距(单位:秒)
    - 从容模式: 1.75秒，适合保守驾驶，注重舒适性
    - 标准模式: 1.45秒，平衡安全性和通行效率
    - 激进模式: 1.25秒，注重通行效率，适合经验丰富的驾驶员

  使用场景:
    1. 作为静态跟车模式的直接参考值
    2. 作为动态跟车模式的基准参考值
    3. 用于计算安全跟车距离
  """
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.75  # 从容模式：较大跟车时距，注重安全性和舒适性
  elif personality==log.LongitudinalPersonality.standard:
    return 1.45  # 标准模式：平衡的跟车时距，适合日常驾驶
  elif personality==log.LongitudinalPersonality.aggressive:
    return 1.25  # 激进模式：较小跟车时距，适合熟练驾驶员
  else:
    raise NotImplementedError("Longitudinal personality not supported")

def get_dynamic_follow(v_ego, personality=log.LongitudinalPersonality.standard, curvature=0.0, rel_speed=0.0):
  # 优化的动态跟车时距计算，保留原始核心逻辑但适当简化
  v_ego = max(0.0, min(v_ego, 40.0))  # 限制v_ego在0-40 m/s之间
  # 根据驾驶风格设置基础时距（保留原始的关键速度点）
  if personality==log.LongitudinalPersonality.relaxed:
    # 调整速度区间使过渡更平滑
    x_vel =  [0.0,  3.0,  8.0,  13.90,  20,    25,    40]  # m/s
    y_dist = [1.0,  1.05, 1.15,  1.25,   1.35,  1.55,  1.7] # 秒
  elif personality==log.LongitudinalPersonality.standard:
    # 调整速度区间使过渡更平滑，增加基础跟车时距
    x_vel =  [0.0,  3.0,  8.0,  13.90,  20,    25,    40]  # m/s
    y_dist = [0.95, 1.00, 1.05,  1.15,   1.25,  1.35,  1.4] # 秒  # 增加了基础跟车时距
  elif personality==log.LongitudinalPersonality.aggressive:
    # 调整速度区间使过渡更平滑
    x_vel =  [0.0,  4.00, 8.0,  13.89,  20,    25,    40]  # m/s
    y_dist = [0.65, 0.70, 0.75,  0.80,   0.85,  0.95,  1.0] # 秒
  else:
    raise NotImplementedError("Dynamic Follow personality not supported")
  base_t_follow = np.interp(v_ego, x_vel, y_dist)

  # 保留重要的曲率和相对速度影响
  curve_factor = np.clip(1.0 + abs(curvature) * 30.0, 1.0, 1.3)
  rel_speed_factor = np.clip(1.0 - rel_speed * 0.12, 0.88, 1.15)
  final_t_follow = base_t_follow * curve_factor * rel_speed_factor
  return np.clip(final_t_follow, 0.5, 2.5)

def get_stopped_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * COMFORT_BRAKE)

# 获取安全障碍物距离
def get_safe_obstacle_distance(v_ego, t_follow):
  return (v_ego**2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE

# 期望跟车距离
def desired_follow_distance(v_ego, v_lead, t_follow=get_T_FOLLOW()):
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead)

def get_stopped_equivalence_factor_krkeegen(v_lead, v_ego):
    """优化的停车等效距离计算
    考虑相对速度和当前速度的动态影响，保留核心安全逻辑
    """
    v_diff = v_lead - v_ego
    v_diff_offset = 0
    if np.all(v_diff > 0):
        # 动态速度因子：高速时更保守，低速时更积极
        speed_factor = np.clip(1.0 - (v_ego / 20.0), 0.2, 1.0)
        # 相对速度影响：速度差越大，反应越积极
        rel_speed_factor = np.clip(v_diff / 5.0, 0.0, 1.0)
        # 计算偏移量
        v_diff_offset = v_diff * speed_factor * rel_speed_factor
        # 根据当前速度动态调整最大偏移量
        max_offset = STOP_DISTANCE * (0.7 - v_ego * 0.02)  # 速度越高，允许的偏移量越小
        v_diff_offset = np.clip(v_diff_offset, 0, max_offset)
    # 基础制动距离
    base_distance = (v_lead**2) / (2 * COMFORT_BRAKE)
    # 高速安全裕度
    safety_margin = np.clip(v_ego * 0.1, 0.0, 2.0)
    return base_distance + v_diff_offset + safety_margin

def gen_long_model():
  """生成纵向控制模型
    创建包含车辆状态、控制输入和动力学方程的模型

    状态变量:
    - x_ego: 车辆纵向位置
    - v_ego: 车辆纵向速度
    - a_ego: 车辆纵向加速度

    控制变量:
    - j_ego: 加加速度(jerk)
    """
  model = AcadosModel()
  model.name = MODEL_NAME

  # set up states & controls
  x_ego = SX.sym('x_ego')  # 纵向位置
  v_ego = SX.sym('v_ego')  # 纵向速度
  a_ego = SX.sym('a_ego')  # 纵向加速度
  model.x = vertcat(x_ego, v_ego, a_ego)

  # 定义控制输入
  j_ego = SX.sym('j_ego')  # 加加速度
  model.u = vertcat(j_ego)

  # 定义状态导数
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow)

  # dynamics model 定义系统动力学方程
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - (3/4) * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, get_T_FOLLOW()])

  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, e2e=False):
    """初始化纵向MPC控制器

        参数:
        e2e: 是否使用端到端控制模式
    """
    self.e2e = e2e
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2] # 默认使用巡航控制
    self.target_obstacle_distance = float('nan')

  def reset(self):
    # self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.solver.reset()
    # self.solver.options_set('print_level', 2)
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N,1))
    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):
    if self.e2e:
      self.set_weights_for_xva_policy()
      self.params[:,0] = -10.
      self.params[:,1] = 10.
      self.params[:,2] = 1e5
      self.params[:,4] = get_T_FOLLOW()
    else:
      self.set_weights_for_lead_policy(prev_accel_constraint, personality)

  def set_weights_for_lead_policy(self, prev_accel_constraint, personality):
    jerk_factor = get_jerk_factor(personality)
    a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0
    W = np.asfortranarray(np.diag([X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, jerk_factor * a_change_cost, jerk_factor * J_EGO_COST]))
    for i in range(N):
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = a_change_cost * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array([LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST])
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights_for_xva_policy(self):
    W = np.asfortranarray(np.diag([0., 10., 1., 10., 0.0, 1.]))
    for i in range(N):
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array([LIMIT_COST, LIMIT_COST, LIMIT_COST, 0.0])
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.: # probably only helps if v < v_prev
      for i in range(0, N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-MIN_ACCEL * 2)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def set_accel_limits(self, min_a, max_a):
    self.cruise_min_a = min_a
    self.cruise_max_a = max_a

  def update(self, carstate, radarstate, v_cruise, personality=log.LongitudinalPersonality.standard, use_df_tune=False, use_krkeegen_tune=False):
    """更新MPC控制器

        参数:
        - carstate: 车辆状态信息
        - radarstate: 雷达探测信息
        - v_cruise: 巡航目标速度
        - personality: 驾驶风格
        - use_df_tune: 是否使用动态跟车调整
        - use_krkeegen_tune: 是否使用优化的停车距离计算
    """
    # 添加对carstate和radarstate的null检查
    if not hasattr(carstate, 'steeringAngleDeg') or not hasattr(radarstate, 'leadOne') or not hasattr(radarstate, 'leadTwo'):
        # 传感器数据丢失时重置控制器，避免使用过时的控制指令
        self.reset()
        return
    v_ego = self.x0[1]
    # 计算相对速度和道路曲率
    rel_speed = radarstate.leadOne.vRel if hasattr(radarstate.leadOne, 'status') and radarstate.leadOne.status else 0.0
    curvature = abs(carstate.steeringAngleDeg * 0.017453292519943295) / (max(v_ego, 1.0) * 2.5)
    # 获取动态跟车时距
    t_follow = get_T_FOLLOW(personality) if not use_df_tune else get_dynamic_follow(v_ego, personality, curvature, rel_speed)
    # 添加对leadOne和leadTwo的status属性检查
    self.status = (hasattr(radarstate.leadOne, 'status') and radarstate.leadOne.status) or (hasattr(radarstate.leadTwo, 'status') and radarstate.leadTwo.status)

    # 处理前车信息
    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    # set accel limits in params 设置约束条件
    self.params[:,0] = interp(float(self.status), [0.0, 1.0], [self.cruise_min_a, MIN_ACCEL])
    self.params[:,1] = self.cruise_max_a

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    # 计算安全距离
    if use_krkeegen_tune:
      lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor_krkeegen(lead_xv_0[:,1], v_ego)
      lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor_krkeegen(lead_xv_1[:,1], v_ego)
    else:
      lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])
      lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])

    # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
    # when the leads are no factor.
    v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
    v_upper = v_ego + (T_IDXS * self.cruise_max_a * 1.05)
    v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                               v_lower,
                               v_upper)
    cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)

    x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
    self.source = SOURCES[np.argmin(x_obstacles[0])]
    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = t_follow
    # 更新期望跟车距离
    min_obstacle_distance = np.min(x_obstacles, axis=1)
    # 修改为只获取当前时刻的跟车距离（第一个元素）
    self.target_obstacle_distance = min_obstacle_distance[0] if len(min_obstacle_distance) > 0 else float('nan')

    self.run()
    if (np.any(lead_xv_0[:,0] - self.x_sol[:,0] < CRASH_DISTANCE) and
        radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

  def update_with_xva(self, x, v, a):
    # v, and a are in local frame, but x is wrt the x[0] position
    # In >90degree turns, x goes to 0 (and may even be -ve)
    # So, we use integral(v) + x[0] to obtain the forward-distance
    xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
    x = np.cumsum(np.insert(xforward, 0, x[0]))
    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.params[:,3] = np.copy(self.prev_a)
    self.run()

  def run(self):
    # t0 = sec_since_boot()
    # reset = 0
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    # qp_iter = self.solver.get_stats('statistics')[-1][-1] # SQP_RTI specific
    # print(f"long_mpc timings: tot {self.solve_time:.2e}, qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e}, integrator {self.time_integrator:.2e}, qp_iter {qp_iter}")
    # res = self.solver.get_residuals()
    # print(f"long_mpc residuals: {res[0]:.2e}, {res[1]:.2e}, {res[2]:.2e}, {res[3]:.2e}")
    # self.solver.print_statistics()

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.prev_a = np.interp(T_IDXS + 0.05, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()
      # reset = 1
    # print(f"long_mpc timings: total internal {self.solve_time:.2e}, external: {(sec_since_boot() - t0):.2e} qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e} qp_iter {qp_iter}, reset {reset}")


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
