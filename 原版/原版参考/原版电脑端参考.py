# -*- coding: utf-8 -*-
"""
PID 内核（PC 侧）—— 结构与屏幕固件**逐行同构**，找到的参数可以直接填回屏幕
=====================================================================
为什么要"同构"：参数 Kp/Ki/Kd 是没有量纲自洽性的，同一个数放在不同结构的
PID 里效果完全不同。所以 PC 这一份必须和要移植过去的 C51 版一模一样：

    输出功率 % = Kp·误差 + 积分项 + Kd·(误差变化率)
    积分项    += Ki·误差·dt          （dt 固定 0.3 秒，与屏幕 pid_ctrl.c 一致）
    输出限幅  0 ~ 100（板子 D 只有整数，还要按 1% 量化）

量纲（填屏幕 SetData 时必须按这个理解）：
    Kp  [% / ℃]        误差 1 ℃ 对应多少功率百分比
    Ki  [% / (℃·s)]    单位时间累积
    Kd  [%·s / ℃]      误差变化速度的阻尼
    Ti = Kp/Ki [s]     显示屏习惯看的积分时间；  Td = Kd/Kp [s] 微分时间

修掉了屏幕老算法（pid_ctrl.c）导致温度跳变的三个毛病：
    1) 老算法 `h` 兜底成 50 会让 Ku 算大、Kp 偏大 → 本版功率摆幅不足时**拒绝出结果**，
       而不是硬算一个大参数；
    2) 老算法微分直接吃温度噪声 → 本版微分作用于**测量值**，并加一阶滤波（lv_bo）；
    3) 老算法一个采样点噪声就能让功率跳好几档 → 本版加**输出变化率限制**
       （mei_miao_zui_da_bian_hua）和**死区**（si_qu）。
=====================================================================
"""

import collections
import math

# ---- 整定用的参数容器：一个候选人 = 一组 Kp/Ki/Kd ----
CanShuZu = collections.namedtuple("CanShuZu", ["Kp", "Ki", "Kd", "gui_ze_ming"])


# 屏幕 SetData 的整数刻度 —— 分参数（必须与屏幕 temp_business.c 的 PID_DB_SCALE_KP/KI/KD 一致）
#   屏上参数页三格都是整数录入，实际值 = 格子里填的数 / 刻度：
#     KP 格填 100 -> 100/10  = Kp 10.00
#     KI 格填   5 -> 5/100   = Ki  0.05   (×10 只能到 0.1，存不下定稿值)
#     KD 格填 200 -> 200/10  = Kd 20.00
SETDATA_FANG_DA = {"KP": 10, "KI": 100, "KD": 10}


class WenDuPID(object):
    """温度 PID（位置式）。ji_suan() 每个控制周期调一次，返回功率百分比。

    参数：
        Kp, Ki, Kd          三个参数（量纲见文件头）
        kong_zhi_zhou_qi_s  控制周期，必须和屏幕一致：0.3 秒
        shu_chu_shang_xian  输出上限（功率 %），一般 100
        shu_chu_xia_xian    输出下限（功率 %），一般 0
        si_qu               死区（℃）：误差小于它就不动，防止在设定点附近抖动
        lv_bo_xi_shu        微分项一阶滤波系数 0~1，越小越平滑（默认 0.3）
        mei_miao_zui_da_bian_hua  输出变化率限制（%/秒），None = 不限制
        fen_bian_lv         输出量化步进（%），板子只认整数 → 1.0
    """

    def __init__(self, Kp, Ki, Kd, kong_zhi_zhou_qi_s=0.3,
                 shu_chu_shang_xian=100.0, shu_chu_xia_xian=0.0,
                 si_qu=0.0, lv_bo_xi_shu=0.3,
                 d_si_qu=1.0,          # ★单位=【每拍台阶数】(一格 0.1℃): 1.0 = 只挡不到一个台阶的变化。
                  #   原值 0.2 的单位是 ℃/s, 在 dt=0.5 时 0.1/0.5=0.2 恰好等于门槛,
                  #   而判据是"严格小于才清零" ⇒ dt 一漂过 0.5s(多板/丢帧补拍) 一个台阶就被整段挡掉,
                  #   D 项等于被关掉。改成台阶数口径后 dt=0.5 时门槛仍是 0.2(行为不变), 其它 dt 自动归一。
                  #   与屏幕 pid_ctrl.h 的 PID_D_SI_QU 必须同时改。
                 mei_miao_zui_da_bian_hua=None, fen_bian_lv=1.0):
        self.Kp, self.Ki, self.Kd = float(Kp), float(Ki), float(Kd)
        self.dt = float(kong_zhi_zhou_qi_s)
        self.shang = float(shu_chu_shang_xian)
        self.xia = float(shu_chu_xia_xian)
        self.d_si_qu = float(d_si_qu)    # ★D 项门槛(单位=每拍台阶数, 一格 0.1℃), 与屏幕 PID_D_SI_QU 一致
        self.si_qu = float(si_qu)
        self.lv_bo = float(lv_bo_xi_shu)
        self.bian_hua_lv = mei_miao_zui_da_bian_hua
        self.fen_bian_lv = float(fen_bian_lv)
        self.fu_wei()

    # ---------------- 复位 ----------------
    def fu_wei(self, shang_ci_shu_chu=0.0, shang_ci_wen_du=None):
        """把积分项/历史值清零。手动切自动时传上次的实际功率，可做到无扰动切换。"""
        self.ji_fen = 0.0
        self.shang_ci_wen_du = shang_ci_wen_du
        self.shang_ci_shu_chu = float(shang_ci_shu_chu)
        self.wei_fen_lv_bo = 0.0

    # ---------------- 一次计算 ----------------
    def ji_suan(self, dang_qian_wen_du, she_zhi_wen_du, dt=None):
        """算一次输出。返回**已经量化**的功率百分比（板子直接能用的整数）。"""
        dt = self.dt if dt is None else float(dt)
        wu_cha_raw = she_zhi_wen_du - dang_qian_wen_du   # ★真实误差(积分用, 对应屏幕 err_i)
        wu_cha = wu_cha_raw

        # 死区: ★只作用于 P 项(与屏幕补丁18一致)。
        #   不能把误差整个清零 —— 否则一进死区 P 和积分同时停手, 输出冻住、
        #   温度会一路掉到死区带的下沿(实测: 设定100/死区1.0 -> 贴在 99.0℃ 即 AD290)。
        if self.si_qu > 0 and abs(wu_cha) < self.si_qu:
            wu_cha = 0.0

        # 微分项：作用于**测量值**（老算法作用于误差，设定点一动就来一次微分冲击）
        if self.shang_ci_wen_du is None:
            wei_fen = 0.0
        else:
            wei_fen = -(dang_qian_wen_du - self.shang_ci_wen_du) / dt
        # ★D 项死区(与屏幕 pid_ctrl.c 的 PID_D_SI_QU 一致): 单格量化台阶(一拍)
#   ★2026-10-15: 底板精度提升后一格 = 0.0853℃(原 0.341), 所以"一格/一拍"从 0.68 降到 0.17;
#   但屏幕 PID_D_SI_QU 目前仍是 0.7 —— 两边要改一起改, 别只改这边(会两边口径不一致)
        #   不该被当成"温度在变" —— 不加这道门, 它每拍踢一下输出(实测 ±7~14%)
        d_men = self.d_si_qu * 0.1 / dt   # 门槛 = 台阶数 × 一格(0.1℃) ÷ dt ⇒ 与 dt 解耦
        if d_men > 0 and abs(wei_fen) < d_men:
            wei_fen = 0.0
        self.wei_fen_lv_bo += self.lv_bo * (wei_fen - self.wei_fen_lv_bo)

        # 积分项：条件积分抗饱和 —— 输出已经打满且误差还在往同方向推时，不再积分
        da_man = (self.shang_ci_shu_chu >= self.shang and wu_cha_raw > 0) or \
                 (self.shang_ci_shu_chu <= self.xia and wu_cha_raw < 0)   # ★抗饱和判据用真实误差
        if not da_man:
            self.ji_fen += self.Ki * wu_cha_raw * dt   # ★积分用真实误差(不吃死区, 与屏幕补丁18一致)
            self.ji_fen = max(min(self.ji_fen, self.shang), self.xia)

        shu_chu = self.Kp * wu_cha + self.ji_fen + self.Kd * self.wei_fen_lv_bo

        # 输出限幅
        shu_chu = max(min(shu_chu, self.shang), self.xia)

        # 输出变化率限制：每秒钟最多变多少（防"一个噪声点跳好几档功率"）
        if self.bian_hua_lv is not None:
            bu = self.bian_hua_lv * dt
            shu_chu = max(min(shu_chu, self.shang_ci_shu_chu + bu),
                          self.shang_ci_shu_chu - bu)

        # ★2026-10-15 与屏幕 pid_ctrl.c 完全同口径: 变化率限幅的"基准"要用【未量化】的浮点值。
        #   屏幕的 pid_calc 不做量化(量化在调用方 temp_heat_output), 所以它的 last_out 是浮点;
        #   PC 原来在核心里量化后回存 shang_ci_shu_chu, 下一拍的限速基准就差 ±0.5 个点
        #   —— 逐拍比对实测最大差 0.66 个点 / 618 拍不一致。改成同口径后为 0。
        self.shang_ci_wen_du = dang_qian_wen_du
        self.shang_ci_shu_chu = shu_chu            # 未量化(与屏幕 last_out 同口径)
        if self.fen_bian_lv > 0:                   # 给板子的仍然是整数 D
            return round(shu_chu / self.fen_bian_lv) * self.fen_bian_lv
        return shu_chu

    # ---------------- 给人看的分解 ----------------
    def fen_jie(self, dang_qian_wen_du, she_zhi_wen_du):
        """返回 (P 项, I 项, D 项, 总输出) —— 报告里画分解图用（不改变内部状态）"""
        wu_cha = she_zhi_wen_du - dang_qian_wen_du
        p = self.Kp * wu_cha
        return p, self.ji_fen, self.Kd * self.wei_fen_lv_bo, p + self.ji_fen + self.Kd * self.wei_fen_lv_bo

    # ---------------- 参数换算（给人看/填屏幕用） ----------------
    def huan_suan_shi_jian_chang_shu(self):
        """→ (Kp, Ti 秒, Td 秒)。Ti/Td 是仪表行业习惯的写法。"""
        Ti = (self.Kp / self.Ki) if abs(self.Ki) > 1e-12 else float("inf")
        Td = (self.Kd / self.Kp) if abs(self.Kp) > 1e-12 else 0.0
        return self.Kp, Ti, Td

    def huan_suan_setdata(self):
        """屏幕 SetData 存的整数刻度 = 屏幕侧 PID_DB_SCALE。

        ★2026-10-17 起改为 **放大 100 倍**（屏幕 PID_DB_SCALE 10.0f -> 100.0f）：
          ★分参数刻度（2026-10-17）：Kp 存 ×10、Ki 存 ×100、Kd 存 ×10
          ⇒ 屏上三格都是整数：KP 填 100、KI 填 5、KD 填 200（组态不用改小数位）。
          原因：Ki 定稿 0.05，若统一 ×10 只能到 0.1（0.05*10=0.5 取整成 1）。
        """
        return dict(KP_SD=int(round(self.Kp * SETDATA_FANG_DA["KP"])),
                    KI_SD=int(round(self.Ki * SETDATA_FANG_DA["KI"])),
                    KD_SD=int(round(self.Kd * SETDATA_FANG_DA["KD"])))

    # 旧名字保留为别名（历史调用方不至于断）
    huan_suan_fang_da_shi_bei = huan_suan_setdata


# =====================================================================
