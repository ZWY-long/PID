/* ★2026-10-17 单文件化改造(用户要求"工程里只留 main.c 一个 .c")
 *   本文件 = 原 pid_ctrl.h + pid_ctrl.c 合并而成, 只被 main.c 的编译单元包含一次
 *   (main.c -> acc_w2_temp_business.h -> acc_w1_pid.h)。
 *   ★不许再拆回两个 .c: PidStateTypeDef pid_state[7] 只有这一份定义, 一旦拆开,
 *     main.c 与业务层会各自拿到一份独立状态 => 隐蔽的 PID 状态错乱。
 *   优化等级由 main.c 顶部的 #pragma OPTIMIZE (9,SIZE) 统一给出, 本文件不再重复。 */
#ifndef __ACC_W1_PID_H__
#define __ACC_W1_PID_H__

/***************************************************************************
 * 文件名: pid_ctrl.h
 * 功  能: PID算法内核头文件(纯算法层)
 *         仅负责: PID状态结构体定义、位置式PID计算、状态复位、自整定状态机
 * 约  束: 本组文件(pid_ctrl.h / pid_ctrl.c)不操作任何IO、不读写Flash、
 *         不访问业务数据库; 所有参数校验、Flash保存逻辑均隔离在业务层
 *         temp_business.h / temp_business.c
 * 版  本: V1.0  20260828
***************************************************************************/

/*----------------------- 基础类型定义 -----------------------*/
/* 裸机标准C工程(不带stdint.h), 此处自行定义固定宽度类型,
 * 与 define.h 中已有的 u8/u16 类型不冲突, 保护宏防止重复定义   */
#ifndef __PID_CTRL_U8T_DEFINED__
#define __PID_CTRL_U8T_DEFINED__
typedef unsigned char   uint8_t;        /* 8位无符号 */
typedef signed char     int8_t;         /* 8位有符号 */
typedef unsigned int    uint16_t;       /* 16位无符号(C51中int为16位) */
#endif

/*----------------------- 算法级宏定义 -----------------------*/
#define PID_CH_NUM          7           /* PID通道总数(通道号0~6)                */
#define PID_DT_S             0.5f      /* 【★第3批】只作兜底: 调用方没给有效dt时才用它。
                                        *   取 0.5 = 当前 PID 节拍(Scan_500msFlag), 也是多板一圈的时长 */
#define PID_TUNE_TICK_S      0.5f      /* 【★第3批】同上(与当前 500ms 节拍同口径)。
                                        *   ★自整定支路已停用(见 temp_business.c);
                                        *   启用前必须把真实 dt 传进 pid_auto_tune, 否则 Z-N 周期是错的 */

/*--------------- 【新增】输出抑制三项(与PC端整定工具同口径) ----------------*/
/* 说明: 三者的数值必须与PC端整定那一轮的设置一致, 否则温度表现会和整定时不一样。
 *       整定工具里对应: 死区 0.3℃ / 微分滤波 0.2 / 输出变化率上限 10 %每秒    */
#define PID_SI_QU_C              0.3f    /* 死区(℃) ★补丁16: 0.3->1.0 —— 加热管周波调功带来的纹波是 +-0.85℃,
                                         *   死区 1.0 直接把它滤掉(代价: 设定点可能偏 +-1℃ = 1%) */
#define PID_D_SI_QU              1.0f    /* ★单位 = 每拍台阶数(1 格 = 0.1℃), 门槛随 dt 自动归一。
                                          *   原值 0.2 的单位是 ℃/s: dt=0.5 时 0.1/0.5 = 0.2 恰好等于门槛,
                                          *   而判据是"严格小于才清零" -> dt 一漂过 0.5s(多板轮询/丢帧补拍),
                                          *   1 个 0.1℃ 台阶就被整段挡掉, D 项等于被关掉。
                                          *   改成台阶数口径后: dt=0.5 时门槛仍是 0.2(台面行为不变), 其它 dt 自动归一。 */
#define PID_LV_BO_XI_SHU        0.2f    /* 微分一阶滤波系数 0~1, 越小越平滑
                                          *   ★2026-10-15 精度提升: 0.1 -> 0.2。0.1 是为了滤掉 一格0.341℃
                                          *   的量化噪声; 现在一格只有 0.0853℃, 噪声小 4 倍, 用 0.2 就够,
                                          *   少的那点滞后让 D 项更跟得上。 */
#define PID_ZUI_DA_BIAN_HUA     5.0f   /* 输出变化率上限(%每秒); 上升/下降都限(第3批修正) —— 每拍最多变化
                                         *   10×dt 个百分点(实测采样间隔 dt 约 0.5s -> 约 5%/拍),
                                         *   下降不限速(断线/超温/目标下调要能立刻压到 0) */

/*----------------------- PID运行状态结构体 -----------------------*/
/* 说明: 仅保存运行时动态状态, 不包含 Kp/Ki/Kd;
 *       三个PID系数由业务层从数据库读取后作为参数传入 pid_calc */
typedef struct {
    float target_temp;                  /* 目标温度(℃)           */
    float actual_temp;                  /* 实测温度(℃)           */
    float output;                       /* 输出功率百分比 0.0~100.0 */
    float integral;                     /* 积分累加项            */
    float last_err;                     /* 上次偏差(★补丁7起微分改用 last_temp, 本字段保留不再参与运算) */
    float d_filt;                       /* 【新增】滤波后的微分(一阶低通), 抑制采样台阶放大   */
    float last_out;                     /* 【新增】上一拍输出(%), 供输出变化率上限使用       */
    float raw_out;                      /* ★补丁9: 限速前的原始PID输出(仅调试显示用, 不参与控制) */
    float last_temp;                    /* ★补丁7: 上次实测温度, 微分改到测量值上算(与 PC 端同口径) */
    uint8_t have_temp;                  /* ★补丁7: 是否已有"上次温度"(0=头一拍, 微分当 0)      */
    float max_out;                      /* 输出上限(百分比)      */
    float min_out;                      /* 输出下限(百分比)      */
} PidStateTypeDef;

/* 7路PID运行状态全局数组(定义于 pid_ctrl.c, 其余文件仅extern引用, 勿重复定义) */
extern PidStateTypeDef pid_state[PID_CH_NUM];

/*----------------------- 自整定返回码 -----------------------*/
#define PID_TUNE_RUNNING    0           /* 正在运行                */
#define PID_TUNE_DONE       1           /* 整定完成(曲线趋于平稳) */
#define PID_TUNE_FAIL       2           /* 整定失败异常            */
#define PID_TUNE_UPDATE     3           /* 本轮参数已更新(迭代进行中) */

/*----------------------- 函数声明 -----------------------*/

/* 位置式PID计算: ch=通道0~6, 内部读写全局 pid_state[ch];
 * 纯浮点运算, 带积分限幅抗积分饱和, 输出功率百分比0.0~100.0 */
void pid_calc(uint8_t ch, float Kp, float Ki, float Kd, float dt);  /* 【★改】dt=本拍真实采样间隔(秒) */

/* PID状态复位: 清零对应通道的 integral、last_err */
void pid_init(uint8_t ch);

/* 自整定: 非阻塞状态机, 迭代式整定(多次修正PID参数让温度曲线趋于平稳):
 * 每次调度采集温度与PID输出, 每完成一组峰→峰振荡周期按Z-N计算一次新参数;
 * 返回: 0=正在运行 1=整定完成 2=整定失败异常 3=本轮参数已更新;
 * 整定模式由业务层调用pid_calc进行PID调温, 本函数只做采集与参数迭代;
 * 注意: 函数内部绝对不读写Flash */
/* 【★第3批】启用本函数前必须加 dt 入参: 函数内部用 PID_TUNE_TICK_S 折算振荡周期,
 *   现在节拍是 500ms、多板时每块板的采样间隔又不一致, 写死会把 Z-N 算出的 Ti/Td 算错。 */
uint8_t pid_auto_tune(uint8_t ch, float *pKp, float *pKi, float *pKd);

/* 自整定复位: 复位状态机, 允许指定通道重新开始一次新的自整定 */
void pid_tune_reset(uint8_t ch);

/* ==================== 以下为原 pid_ctrl.c 的实现 ==================== */

/***************************************************************************
 * 文件名: pid_ctrl.c
 * 功  能: PID算法内核(纯算法层)
 *         - 位置式PID计算(积分限幅抗饱和, 输出0~100%)
 *         - PID状态复位
 *         - 自整定非阻塞状态机(波峰波谷采集 + Ziegler-Nichols整定)
 * 约  束: 本文件完全不触碰数据库、Flash、硬件IO;
 *         所有参数校验、Flash保存逻辑全部隔离在业务层 temp_business
 * 说  明: 本文件与 pid_ctrl.h 配套, 7路PID运行状态全局数组 pid_state[7]
 *         在此定义(全工程仅此一处定义, 其余文件用extern引用)
 * 版  本: V1.0  20260828
***************************************************************************/


/*================ 7路PID运行状态(全局RAM, 仅运行时动态状态) ================*/
PidStateTypeDef pid_state[PID_CH_NUM];

/*============================== pid_init ==============================*/
/* 函数功能: 复位指定通道的PID运行状态
 * 输入参数: ch - 通道号0~6
 * 返回参数: 无
 * 处理逻辑: 清零积分累加项与上次偏差, 防止历史控制状态影响新过程
========================================================================*/
void pid_init(uint8_t ch)
{
    if(ch >= PID_CH_NUM) return;                /* 通道越界保护 */

    pid_state[ch].integral = 0.0f;              /* 清零积分累加项 */
    pid_state[ch].last_err = 0.0f;              /* 清零上次偏差   */
    pid_state[ch].d_filt   = 0.0f;              /* 【新增】清零微分滤波 */
    pid_state[ch].last_out = 0.0f;              /* 【新增】清零上一拍输出 */
    pid_state[ch].last_temp = 0.0f;             /* 补丁7: 清"上次温度" */
    pid_state[ch].have_temp = 0;                /* 补丁7: 标记头一拍(微分当 0, 免得吃假台阶) */
}

/*============================== pid_calc ==============================*/
/* 函数功能: 位置式PID计算(纯浮点, 只做算法运算)
 * 输入参数: ch - 通道号0~6, 内部读写全局 pid_state[ch]
 *           Kp/Ki/Kd - PID系数(由业务层从数据库换算为float后传入)
 * 返回参数: 无(结果写入 pid_state[ch].output)
 * 处理逻辑: 1) err = target - actual
 *           2) integral += err * dt
 *           3) 积分限幅到输出量程, 抗积分饱和
*           4) out = Kp*err + Ki*integral + Kd*d_filt (微分补丁7起作用在测量值上)
 *           5) 输出限幅后写入 pid_state[ch].output
 * 约  束  : 不操作IO、不读写Flash、不与业务数据库耦合
========================================================================*/
/* ★改动: dt = 本拍真实采样间隔(秒), 由业务层按"该板温度新数据到达"时刻算出来传入;
 *   多板时各板间隔不同(约0.4~0.5s), 不能再写死 0.3s —— 写死会把微分项按比例放大。 */
void pid_calc(uint8_t ch, float Kp, float Ki, float Kd, float dt)
{
    float err;                                  /* 本次偏差       */
    float err_i;                                /* ★补丁18: 未加死区的真实误差(给积分用) */
    float out;                                  /* 本次计算输出   */
    float lo, hi;                               /* 输出限幅范围   */
    float d_raw;                                /* 【新增】本拍原始微分 */
    float bu;                                   /* 【新增】本拍输出允许的最大变化量 */

    if(ch >= PID_CH_NUM) return;                /* 通道越界保护 */
    if(dt <= 0.0f) dt = PID_DT_S;               /* ★dt兜底: 调用方没给有效间隔时用默认值 */
    if(dt <  0.05f) dt = PID_DT_S;              /* ★★第3批兜底: dt 过小会让"输出变化率上限"
                                                 *   把功率锁死在 ~0(实测卡在 3%), 一律按一拍算 */
    if(dt > 1.0f)  dt = 1.0f;                   /* 补丁8: 与 PC 端一致(钳位 0.1~1.0), 防长时间没数据时积分猛冲 */

    /* 输出限幅范围: 业务层初始化时已配置; 若未配置则默认0~100 */
    hi = pid_state[ch].max_out;
    lo = pid_state[ch].min_out;
    if(hi <= lo){ hi = 100.0f; lo = 0.0f; }

    /* 1、计算本次偏差 */
    err = pid_state[ch].target_temp - pid_state[ch].actual_temp;
    err_i = err;                                /* ★补丁18: 存真实误差 —— 积分不吃死区 */

    /* 1.5【新增】死区: 偏差很小时当0, 免得在设定点附近跟着采样台阶来回抖
     *     (PT100一个台阶约0.34℃, 不加死区时到线附近必抖) */
    /* ★补丁18(重要): 死区只作用于【P 项】。积分必须用真实误差 err_i ——
     *   否则一进死区带, P 和积分同时停手, 输出冻住, 温度会一路掉到带的下沿
     *   (实测: 设定 100/死区 1.0 -> 温度贴着 99.0℃ 即 AD 290 走)。
     *   积分用真实误差后, 纹波在一两个周期内自平均掉(净积分为 0),
     *   但均值会被稳稳拉到设定值, 不再有系统偏置。 */
    if(err < PID_SI_QU_C && err > -PID_SI_QU_C) err = 0.0f;

    /* 2、积分累加(★dt = 本拍真实采样间隔, 由调用方传入)
     * ★★第3批补丁6: 条件积分(抗饱和) —— 与 PC 端 pid_nei_he.py 同口径:
     *   "输出已经顶到限幅、且误差还在往同方向推"时, 这一拍不再累加积分。
     *   不加这条的后果(台面实测): 升温段输出一直顶 100%, 积分被撑满到量程上限,
     *   到线后要反向积很久(约 Ki×误差 = 0.3%/秒)才能把输出拉下来
     *   → 过冲 3℃、输出 100<->52 来回摆、周期约 3 分钟。 */
    if(!((pid_state[ch].output >= hi && err_i > 0.0f)
      || (pid_state[ch].output <= lo && err_i < 0.0f))){   /* ★补丁18: 抗饱和判据用真实误差 */
        pid_state[ch].integral += err_i * dt;    /* ★补丁18: 积分用真实误差(不吃死区) */
    }

    /* 3、积分限幅(抗积分饱和): 对积分项贡献(Ki×integral)限幅到输出量程,
     *    保证Ki很小时积分仍能累积到覆盖稳态所需功率, 消除稳态偏差(掉温);
     *    Ki=0时无积分作用, 跳过限幅防止除零 */
    if(Ki > 0.0001f){
        if(Ki * pid_state[ch].integral > hi) pid_state[ch].integral = hi / Ki;
        if(Ki * pid_state[ch].integral < lo) pid_state[ch].integral = lo / Ki;
    }

    /* 4【新增】微分: 先算原始微分, 再一阶滤波 —— 不平滑的话一个采样台阶(0.34C)
     *   会被放大成 Kd x 0.34/dt 的功率跳变, 这是屏幕上老跳变的主因
     * 补丁7: 微分改到【测量值】上算(与 PC 端 pid_nei_he.py:80-85 同口径):
     *   dT/dt 取负号后加到输出(温度升 -> 减输出)。设定值不变时与"作用在误差上"等价,
     *   差别在两处: (1) 改设定值不再来一次微分冲击;
     *              (2) 死区把误差打成 0 的那一刻, 微分里不再凭空多出 0.3/dt 的假台阶
     *                  (老写法每次进出死区都踢一下约 +-7%)。 */
    if(!pid_state[ch].have_temp){
        d_raw = 0.0f;                              /* 头一拍没有"上次温度", 微分当 0 */
    }else{
        d_raw = -(pid_state[ch].actual_temp - pid_state[ch].last_temp) / dt;
    }
    /* ★补丁16: D 项死区 —— 单格量化台阶(0.341℃)不该被当成"温度在变"。
     *   不加这道门, 它每拍都踢一下输出(Kd=10 时 +-7~14%), 而真正的慢速振荡
     *   (周期几分钟、斜率 ~0.02℃/s)本来就在这道门之下 —— 所以这道门只砍噪声。 */
    {   float d_men = PID_D_SI_QU * 0.1f / dt;  /* 门槛随 dt 归一: 1 个台阶 = 0.1℃/dt, dt=0.5 时 = 0.2 */
        if(d_raw < d_men && d_raw > -d_men) d_raw = 0.0f; }
    pid_state[ch].d_filt += PID_LV_BO_XI_SHU * (d_raw - pid_state[ch].d_filt);
    pid_state[ch].last_temp = pid_state[ch].actual_temp;   /* 补丁7: 存本拍温度供下一拍微分 */
    pid_state[ch].have_temp = 1;

    /* 5、位置式PID输出: out = Kp*err + Ki*∫err·dt + Kd*滤波后微分 */
    out  = Kp * err;
    out += Ki * pid_state[ch].integral;
    out += Kd * pid_state[ch].d_filt;

    /* 5.5、保存本次偏差, 供下一周期微分运算 */
    pid_state[ch].last_err = err;               /* 补丁7起不再参与运算, 仅为保持结构体字段原意 */

    /* 6、输出限幅(功率百分比0.0~100.0或业务层配置范围) */
    if(out > hi) out = hi;
    if(out < lo) out = lo;


    pid_state[ch].raw_out  = out;      /* ★补丁9: 记下限速前的原始输出(调试显示用) */
    /* 6.5【新增】输出变化率上限: 每拍最多变 PID_ZUI_DA_BIAN_HUA×dt 个百分点(dt=实测间隔),
     *     防止输出猛跳(整定工具的"最大单步功率变化"就是它管的)。
     * ★★第3批: 上升/下降都限速(与 PC 端一致) —— 原来只限上升、下降不限,
     *   微分项一反转就把输出瞬间砸掉几十个点, 再每拍只爬 5% 慢慢回来,
     *   这正是"温度稳但功率乱跳"的锯齿。
     * ★注意: 本段注释的收尾标记绝不能丢 —— 丢了会把下面三行一起注释掉,
     *   编译器只报 "C280: bu: unreferenced local variable", 限速就静默失效。 */
    bu = PID_ZUI_DA_BIAN_HUA * dt;
    if(out - pid_state[ch].last_out >  bu)  out = pid_state[ch].last_out + bu;
    if(out - pid_state[ch].last_out < -bu)  out = pid_state[ch].last_out - bu;

    pid_state[ch].output   = out;
    pid_state[ch].last_out = out;
}

/*==================== 自整定内部私有数据(仅本文件可见) ====================*/
/* 说明: 自整定运行期间业务层需要暂停普通 pid_calc 调用, 防止两套控制冲突。
 *       整定状态机依靠多次调度推进, 禁止阻塞while、禁止固定延时计时,
 *       时间一律用"调度次数 x 调用周期"折算(见PID_TUNE_TICK_S)。        */

#define TUNE_MAX_ITER       8               /* 迭代安全上限(温度稳定后提前完成, 上限仅防参数不收敛无限迭代) */
#define TUNE_DEADBAND       0.2f            /* 波峰/波谷识别死区(℃), 滤除采样噪声        */
#define TUNE_MAX_TICKS      20000           /* 整定总超时(调度次数), 约6000秒             */
#define TUNE_STUCK_TICKS    1200            /* 未采到任何波峰的停滞超时(约6分钟)          */
#define TUNE_STABLE_AMP     1.0f            /* 收敛判定: 峰谷幅值小于该值视为曲线趋于平稳  */
#define TUNE_MIN_DU         5.0f            /* PID输出摆幅下限, 低于该值用兑底摆幅计算     */
#define TUNE_RELAY_AMP      50.0f           /* 输出摆幅兑底值h(输出摆幅过小时使用)         */

/* Z-N整定阻尼系数(经典Z-N偏激进, 易产生大超调, 按工程经验阻尼化):
 *   Kp = 0.6Ku * ZN_KP_DAMP
 *   Ti = period * ZN_TI_FACTOR
 *   Td = period * ZN_TD_FACTOR
 * 配合pid_calc的积分限幅(Ki×integral限幅到输出量程)使用,
 * 可同时抑制超调和稳态掉温 */
#define ZN_KP_DAMP          0.75f           /* Kp阻尼系数(0.7~0.8)             */
#define ZN_TI_FACTOR        0.8f            /* 积分时间系数(0.8~1.0)           */
#define ZN_TD_FACTOR        0.125f          /* 微分时间系数(经典0.125, 可减半) */

/* 每通道整定采集上下文(私有状态机变量, 不对外暴露) */
typedef struct {
    uint8_t  state;                         /* 0=空闲 1=运行中 2=完成 3=失败 */
    int8_t   dir;                           /* 当前温度趋势: +1上升 -1下降 0未知 */
    uint16_t tick;                          /* 整定调度计数(兼作时间戳) */
    uint16_t peak_tick;                     /* 上一次波峰出现时的调度计数 */
    uint8_t  peak_cnt;                      /* 已采集波峰个数 */
    uint8_t  has_valley;                    /* 本次峰前是否已采到波谷(峰谷配对用) */
    uint8_t  iter_cnt;                      /* 已完成参数迭代更新次数 */
    float    peak;                          /* 最近一次波峰值(℃) */
    float    valley;                        /* 最近一次波谷值(℃) */
    float    prev;                          /* 上一次采样温度(℃) */
    float    out_max;                       /* 本组周期内PID输出最大值(%) */
    float    out_min;                       /* 本组周期内PID输出最小值(%) */
    float    Kp_keep;                       /* 最近一次迭代计算的Kp */
    float    Ki_keep;                       /* 最近一次迭代计算的Ki */
    float    Kd_keep;                       /* 最近一次迭代计算的Kd */
} TuneCtx;

static TuneCtx s_tune[PID_CH_NUM];          /* 7通道整定上下文(静态, 仅算法层内部) */

/*============================= pid_auto_tune =============================*/
/* 函数功能: 自整定(非阻塞状态机, 迭代式整定, 多次修正PID参数让温度曲线趋于平稳)
 * 输入参数: ch - 通道号0~6
 *           pKp/pKi/pKd - 参数输出指针(迭代更新或完成时写入计算结果)
 * 返回参数: 0=正在运行(继续调度调用)
 *           1=整定完成(温度曲线对称平稳)
 *           2=整定失败异常
 *           3=本轮参数已更新(业务层更新数据库后继续迭代)
 * 处理逻辑: 1) 每次调用采样一次实测温度与PID输出(由业务层写入 pid_state[ch])
 *           2) 死区滤波后判断温度趋势, 趋势反转处记录波峰/波谷
 *           3) 每完成一组峰→峰完整周期:
 *              a. 统计周期时长T与峰谷幅值A
 *              b. 收敛判定: A<阈值且峰/谷都贴近目标(对称平稳) -> 整定完成
 *              c. 按输出摆幅法计算临界增益 Ku=4h/(πa), h=本组输出摆幅的一半
 *              d. Z-N阻尼化+平滑后计算Kp/Ki/Kd写入指针并返回3
 *              e. 达到迭代安全上限 -> 整定完成(兑底防参数不收敛)
 * 约  束  : 修改传入指针指向的参数; 函数内部绝对不读写Flash;
 *           不操作IO; 不接触业务数据库
 * 备  注  : 整定模式由业务层调用pid_calc实现PID调温, 本状态机只做采集与参数迭代
===========================================================================*/
uint8_t pid_auto_tune(uint8_t ch, float *pKp, float *pKi, float *pKd)
{
    TuneCtx *p;
    float cur, diff, err;                   /* 本次采样温度、差值、目标偏差 */
    float period, amp;                      /* 本组周期时长(秒)、峰谷幅值(℃) */
    float du, h, a, Ku, Ti, Td;             /* Z-N整定中间量 */
    float Kp_new, Ki_new, Kd_new;           /* 本轮Z-N计算参数(平滑前) */
    float pa, va;                           /* 峰/谷相对目标的偏差(收敛判定用) */
    int8_t newdir;                          /* 本次趋势方向 */

    /* 参数合法性检查: 通道越界或指针为空, 直接判失败 */
    if(ch >= PID_CH_NUM || pKp == 0 || pKi == 0 || pKd == 0){
        return PID_TUNE_FAIL;
    }
    p = &s_tune[ch];

    /* 已结束的整定保持原结果, 防止重复触发;
     * 如需再次整定, 由业务层先调用 pid_tune_reset(ch) 复位 */
    if(p->state == 2){
        *pKp = p->Kp_keep;                  /* 完成态回写最近一次迭代参数 */
        *pKi = p->Ki_keep;
        *pKd = p->Kd_keep;
        return PID_TUNE_DONE;
    }
    if(p->state == 3) return PID_TUNE_FAIL;

    /* 首次进入: 初始化采集上下文 */
    if(p->state == 0){
        p->state      = 1;                  /* 进入运行中 */
        p->dir        = 0;                  /* 趋势未知 */
        p->tick       = 0;
        p->peak_tick  = 0;
        p->peak_cnt   = 0;
        p->has_valley = 0;
        p->iter_cnt   = 0;
        p->prev       = pid_state[ch].actual_temp;
        p->peak       = p->prev;
        p->valley     = p->prev;
        p->out_max    = pid_state[ch].output;
        p->out_min    = p->out_max;
        p->Kp_keep    = 0.0f;
        p->Ki_keep    = 0.0f;
        p->Kd_keep    = 0.0f;
        return PID_TUNE_RUNNING;
    }

    /* 调度计数自增(每次调用代表推进一个调度周期, 兼作时间戳) */
    p->tick++;
    if(p->tick >= TUNE_MAX_TICKS){          /* 总超时 → 整定失败 */
        p->state = 3;
        return PID_TUNE_FAIL;
    }

    /* 采样温度, 同时记录本组周期内PID输出的峰谷(用于计算激励摆幅) */
    cur  = pid_state[ch].actual_temp;       /* 实测温度由业务层周期性写入 */
    diff = cur - p->prev;
    p->prev = cur;
    if(pid_state[ch].output > p->out_max) p->out_max = pid_state[ch].output;
    if(pid_state[ch].output < p->out_min) p->out_min = pid_state[ch].output;

    if(diff >  TUNE_DEADBAND)      newdir =  1;   /* 上升 */
    else if(diff < -TUNE_DEADBAND) newdir = -1;   /* 下降 */
    else                           newdir = p->dir;  /* 死区内维持原趋势 */

    /* 趋势反转判定: 由升转降记录波峰, 由降转升记录波谷 */
    if(p->dir != 0 && newdir != p->dir){
        if(p->dir == 1){
            /* 上升转下降: 记录本次波峰 */
            p->peak = p->prev;
            p->peak_cnt++;
            if(p->peak_cnt >= 2 && p->has_valley){
                /* 峰→峰构成一个完整振荡周期: 统计周期时长与峰谷幅值 */
                period = (float)(p->tick - p->peak_tick) * PID_TUNE_TICK_S;
                amp = p->peak - p->valley;
                if(amp < 0) amp = -amp;     /* 幅值取绝对值 */

                /* 本组周期内PID输出摆幅 */
                du = p->out_max - p->out_min;

                /* 收敛判定(对称平稳): 峰谷幅值小于阈值 且 峰/谷都贴近目标,
                 * 说明温度曲线在设定温度附近对称平稳, 无超调/无稳态偏差 -> 完成;
                 * 若仅幅值小但峰或谷偏离目标(如+8/-1不对称), 不判完成继续迭代 */
                if(amp < TUNE_STABLE_AMP){
                    pa = p->peak - pid_state[ch].target_temp;
                    if(pa < 0) pa = -pa;    /* 峰相对目标的偏差绝对值 */
                    va = pid_state[ch].target_temp - p->valley;
                    if(va < 0) va = -va;    /* 谷相对目标的偏差绝对值 */
                    if(pa < TUNE_STABLE_AMP && va < TUNE_STABLE_AMP){
                        p->state = 2;
                        *pKp = p->Kp_keep;
                        *pKi = p->Ki_keep;
                        *pKd = p->Kd_keep;
                        return PID_TUNE_DONE;
                    }
                }

                /* 异常数据保护: 周期或幅值无效视为整定失败 */
                if(period <= 0.001f || amp <= 0.001f){
                    p->state = 3;
                    return PID_TUNE_FAIL;
                }

                /* Ziegler-Nichols临界比例整定(输出摆幅法):
                 * 临界增益 Ku = 4h/(π·a), h=本组输出摆幅的一半(du/2),
                 * a=振荡半幅值(峰谷幅值的一半); 输出摆幅过小时用兑底值 */
                h = du * 0.5f;
                if(h < (TUNE_MIN_DU * 0.5f)) h = TUNE_RELAY_AMP;
                a  = amp * 0.5f;
                Ku = (4.0f * h) / ((float)3.1415926 * a);

                /* Z-N阻尼化整定: Kp=0.6Ku×阻尼, Ti/Td按系数缩放,
                 * 阻尼化后Ki更小, 配合pid_calc积分限幅(按Ki换算)不会掉温 */
                Ti = ZN_TI_FACTOR * period;
                Td = ZN_TD_FACTOR * period;
                Kp_new = 0.6f * Ku * ZN_KP_DAMP;
                Ki_new = Kp_new / Ti;
                Kd_new = Kp_new * Td;

                /* 迭代参数平滑: 单轮参数最多变化30%, 防止采集瞬态
                 * 导致参数跳变/发散(第1轮无旧参数, 直接用计算值) */
                if(p->iter_cnt >= 1){
                    p->Kp_keep = p->Kp_keep + 0.3f * (Kp_new - p->Kp_keep);
                    p->Ki_keep = p->Ki_keep + 0.3f * (Ki_new - p->Ki_keep);
                    p->Kd_keep = p->Kd_keep + 0.3f * (Kd_new - p->Kd_keep);
                } else {
                    p->Kp_keep = Kp_new;
                    p->Ki_keep = Ki_new;
                    p->Kd_keep = Kd_new;
                }
                *pKp = p->Kp_keep;
                *pKi = p->Ki_keep;
                *pKd = p->Kd_keep;

                p->iter_cnt++;              /* 迭代次数+1 */
                /* 重置本组采集状态, 开始下一组振荡采集 */
                p->has_valley = 0;
                p->out_max = pid_state[ch].output;
                p->out_min = p->out_max;

                /* 迭代安全上限: 防止参数不收敛时无限迭代
                 * (正常情况下对称平稳后已提前完成, 不会走到这里) */
                if(p->iter_cnt >= TUNE_MAX_ITER){
                    p->state = 2;
                    return PID_TUNE_DONE;
                }
                return PID_TUNE_UPDATE;     /* 本轮参数已更新, 业务层更新数据库后继续 */
            }
            p->peak_tick = p->tick;         /* 更新时间戳, 作为下一周期起点 */
        } else {
            /* 下降转上升: 记录本次波谷 */
            p->valley = p->prev;
            p->has_valley = 1;
        }
    }
    p->dir = newdir;

    /* 停滞保护: 长时间连一个波峰都采不到, 说明温度无振荡:
     * 若温度已平稳在目标附近 -> 曲线已平稳, 无需再调整参数, 直接判整定完成;
     * 若温度远离目标 -> 加热能力不足或异常, 判整定失败 */
    if(p->peak_cnt == 0 && p->tick >= TUNE_STUCK_TICKS){
        err = pid_state[ch].target_temp - cur;
        if(err > -TUNE_STABLE_AMP && err < TUNE_STABLE_AMP){
            /* 已平稳在目标附近: 参数无需调整(保持原值), 整定完成 */
            p->state = 2;
            *pKp = p->Kp_keep;
            *pKi = p->Ki_keep;
            *pKd = p->Kd_keep;
            return PID_TUNE_DONE;
        }
        p->state = 3;
        return PID_TUNE_FAIL;
    }

    return PID_TUNE_RUNNING;                /* 本组振荡周期未完成, 继续运行 */
}

/*============================ pid_tune_reset ============================*/
/* 函数功能: 复位自整定状态机, 允许指定通道重新开始一次新的自整定
 * 输入参数: ch - 通道号0~6
 * 返回参数: 无
==========================================================================*/
void pid_tune_reset(uint8_t ch)
{
    if(ch >= PID_CH_NUM) return;            /* 通道越界保护 */

    s_tune[ch].state = 0;                   /* 回到空闲态, 下次调用重新初始化 */
}


#endif  /* __ACC_W1_PID_H__ */
