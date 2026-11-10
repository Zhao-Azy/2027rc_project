#include "go_control.h"
#include "usart.h"
#include "cmsis_os.h"

/*==============================================================================
 * 两台宇树 GO-M8010-6（RS485 半双工，一问一答）
 *-------------------------------------------------------------------------------
 * 硬件：DM-MC-Board02 板载 485 第 1 路 = USART2（PD5/PD6），方向脚 DE = PD4，
 *       Unitree_GO_Init() 自己把 PD4 配成推挽输出，不用在 gpio.c 里配。
 * 注意：一条 485 总线同一时刻只能有一个方向，所以两台电机必须在同一个任务里
 *       依次 SendRecv，不能开两个任务并行发。
 *============================================================================*/

/* 电机编号（必须和宇树 MotorTools 里烧进去的 ID 一致） */
#define GO_JOINT_ID0            0u
#define GO_JOINT_ID1            1u
#define GO_JOINT_RATIO          6.33f    /* GO-M8010-6 减速比 */

/* 上电动作参数：500ms 后使能，各自“当前位置 + 30°” */
#define GO_START_DELAY_MS       500u
#define GO_START_ANGLE_RAD      0.5235988f  /* 30° = 30 × π/180 */
#define GO_START_KP             20.0f       /* 关节侧刚度 N·m/rad */
#define GO_START_KD             0.6f        /* 关节侧阻尼 N·m·s/rad */

/* 轮询周期 ms（两台一轮约 0.6ms，5ms 足够） */
#define GO_PERIOD_MS            5u

Unitree_GO_HandleTypeDef go_joint[GO_JOINT_NUM];

void GO_App_Init(void)
{
    /* 两台都在同一路 485 上：串口和方向脚相同，只有 ID 不同 */
    Unitree_GO_Init(&go_joint[0], &UNITREE_GO_485_UART,
                    UNITREE_GO_DE_PORT, UNITREE_GO_DE_PIN,
                    GO_JOINT_ID0, GO_JOINT_RATIO);
    Unitree_GO_Init(&go_joint[1], &UNITREE_GO_485_UART,
                    UNITREE_GO_DE_PORT, UNITREE_GO_DE_PIN,
                    GO_JOINT_ID1, GO_JOINT_RATIO);
}

/**
 * @brief GO 电机轮询任务
 * @note  1) 上电 500ms 前只发“锁定”帧：既拿到当前位置反馈，又不会让关节自由下坠；
 *        2) 500ms 后目标 = 收到反馈里的当前位置 + 30°，用位置模式保持住；
 *        3) 某台电机没接 / 没上电时 ok_cnt 不涨，就一直停在锁定帧，不会盲发位置指令，
 *           等它上线后自动按“当时的位置 + 30°”补上动作。
 */
void GO_Task(void *argument)
{
    uint8_t ready[GO_JOINT_NUM] = {0u};      /* 1 = 已经算好目标角度 */
    float   target[GO_JOINT_NUM] = {0.0f};   /* 关节侧目标角度 rad */
    uint8_t i;

    for (;;)
    {
        /* HAL_GetTick() 从 HAL_Init 起算，约等于上电时刻 */
        uint8_t move_now = (uint8_t)(HAL_GetTick() >= GO_START_DELAY_MS);

        for (i = 0u; i < GO_JOINT_NUM; i++)
        {
            if ((move_now != 0u) && (ready[i] == 0u) && (go_joint[i].ok_cnt > 0u))
            {
                /* 用刚收到的反馈算目标：相对当前位置转 30° */
                target[i] = go_joint[i].joint_pos + GO_START_ANGLE_RAD;
                ready[i]  = 1u;
            }

            if (ready[i] != 0u)
            {
                Unitree_GO_PositionMode(&go_joint[i], target[i],
                                        GO_START_KP, GO_START_KD);
            }
            else
            {
                /* 还不知道当前位置：发锁定帧（不运动）来换反馈 */
                Unitree_GO_Lock(&go_joint[i]);
            }

            /* 485 半双工：一帧命令 + 一帧回包，两台必须串行 */
            Unitree_GO_SendRecv(&go_joint[i]);
        }

        osDelay(GO_PERIOD_MS);
    }
}
