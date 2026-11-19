# hybrid_a_star_planner

## 详细功能与参数文档

- [FUNCTIONAL_GUIDE.md](FUNCTIONAL_GUIDE.md)：节点启动、功能/函数流程图、话题与包间关系、逐函数职责、当前实现边界。
- [PARAMETERS.md](PARAMETERS.md)：配置值、参数作用及增大/减小的影响。
- [航点交接与搜索提速](WAYPOINT_HANDOFF_SEARCH_FIX.md)：立即交接、顺序速度预算、短距离正交目标及真实河图复测方法。
- [终点容差与特殊转向](MANEUVER_TOLERANCE_UPDATE.md)：30°容差、安全借道、紧凑低速机动及参数操作。
- [工作空间总索引](../../PACKAGE_DOCUMENTATION_INDEX.md)：整条运行链、操作与验证。



## 最大航速优先与低速提速恢复

2026-10-10更新：统一10档降序速度采样，主航道优先，稳定出弯/持续低速后事件提速；参数和实时预算边界见 [PARAMETERS.md](PARAMETERS.md)。前后海图、开阔航道及真实ROS节点模拟船位测试见 [测试报告](../../artifacts/hybrid_speed_update/REPORT.md)。


对向分道后方目标的起转侧软偏好、10/20m短机动、真实低速尝试日志与阶段预算修正，见[修正与测试报告](OPPOSITE_LANE_MANEUVER_FIX.md)。报告包含78项单元回归、航道/规划ROS节点场景与真实河图短机动结果。
