/**
 * @file touch_panel.c
 * @brief 电容触摸屏驱动: 自动识别 GT911 / CST816 / FT6236
 *
 * 三种芯片都是 I2C 从机, 读到的是"点数 + 各触点的 XY 坐标": GT911/FT6236
 * 按硬件能力读多点; CST836U 兼容原厂单指帧并自适应解析双指 (未确认时回退单指).
 * 本驱动只读点, 手势识别在 input.c 里做 (按坐标增量判方向, 与分辨率无关).
 *
 * I2C 使用独立的 I2C_NUM_1 (音频 ES8311/ES7210 占用 I2C_NUM_0), 互不干扰.
 */
#include "touch_panel.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "TOUCH"

#define TP_I2C_PORT   I2C_NUM_1
#define TP_I2C_FREQ   (100 * 1000)   /* 飞线 100kHz 最稳; 稳定后可选 400kHz */

/* ==================== 后台触摸看门狗 (V1.0.68) ====================
 * 问题: 触摸芯片挂死/总线卡住时旧逻辑在 UI 任务里直接做恢复 (RST+重装I2C,
 * 阻塞 ~70-150ms) → 滑动中停顿"卡"; 重装 I2C 驱动与读取竞态还可能触发
 * 驱动断言 → 整机重启.
 * 方案: 独立看门狗任务检测异常 → 先置 s_recovering (读取立即返回, 不阻塞
 * UI) → 等总线空闲后由看门狗任务执行恢复. 恢复带节流, 防抖. */
#define WD_CHECK_MS        500      /* 看门狗检查周期 */
#define WD_FAIL_HARD       10       /* 窗口内失败 ≥ 此数 → 强制恢复 */
#define WD_READS_MIN       20       /* 窗口内读取 ≥ 此数才看失败率 */
#define WD_RATE_DIV        2        /* 失败率 > 1/WD_RATE_DIV → 软恢复 */
#define WD_STUCK_MS        10000    /* 坐标 10s 不变且按住 → 芯片卡死 */
#define WD_MAX_RECOVER     6        /* 3 分钟内最多恢复次数 */
#define WD_THROTTLE_MS     180000
#define WD_RECOVER_WAIT    200      /* 等总线空闲最长 200ms */

static volatile int      s_busy = 0;          /* 读取任务占用计数 */
static volatile bool     s_recovering = false;/* 恢复中: 读取免阻塞返回 */
static volatile bool     s_recover_request = false; /* input 请求恢复 */
static uint32_t          s_wd_win_reads = 0, s_wd_win_fails = 0;
static uint32_t          s_wd_stuck_ms = 0;
static int16_t           s_wd_last_x = -1, s_wd_last_y = -1;
static uint32_t          s_wd_last_ms = 0;
static uint32_t          s_wd_recover_times[WD_MAX_RECOVER] = {0};
static int               s_wd_recover_idx = 0;
static bool              s_wd_window_full = false;   /* 恢复记录窗口是否已填满(空位拉低"最旧", 会误判超频) */
static TaskHandle_t      s_wd_task = NULL;

static void touch_watchdog_task(void *arg);   /* 前向声明 (init 里启动) */
static void touch_panel_soft_recover(void);   /* 前向声明 */
static bool wd_wait_idle(void);

/* 7-bit 从机地址 */
#define GT911_ADDR_A   0x5D
#define GT911_ADDR_B   0x14
#define CST816_ADDR    0x15
#define FT6236_ADDR    0x38

/* GT911 寄存器 (16-bit 寄存器地址) */
#define GT911_REG_PID      0x8140   /* 产品 ID, 应为 "911" */
#define GT911_REG_STATUS   0x814E   /* bit7=数据就绪, bit0-3=点数 */
#define GT911_REG_POINT1   0x814F   /* 第 1 个点, 8 字节 */
#define GT911_REG_RES      0x8048   /* X/Y 分辨率 (0x8048/49=X, 0x804A/4B=Y) */

static tp_chip_t s_chip = TP_CHIP_NONE;
static uint8_t    s_addr = 0;
static int        s_res_x = TP_DEFAULT_RES_X;
static int        s_res_y = TP_DEFAULT_RES_Y;

/* ==================== I2C 底层 ==================== */

/* 探测 7-bit 地址是否有 ACK */
static bool tp_i2c_probe(uint8_t addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(TP_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK);
}

/* 8-bit 寄存器地址读 (CST816 / FT6236) */
static esp_err_t tp_i2c_read8(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(TP_I2C_PORT, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* 16-bit 寄存器地址读 (GT911) */
static esp_err_t tp_i2c_read16(uint16_t reg, uint8_t *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg & 0xFF, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(TP_I2C_PORT, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* 16-bit 寄存器地址写 (GT911) */
static esp_err_t tp_i2c_write16(uint16_t reg, const uint8_t *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg & 0xFF, true);
    if (len) {
        i2c_master_write(cmd, (uint8_t *)buf, len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(TP_I2C_PORT, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ==================== GT911 复位时序 ==================== */

static void gt911_reset(void) {
    /* INT / RST 先设为输出, 走 Goodix 上电复位时序 */
    gpio_set_direction(TP_INT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(TP_RST_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(TP_INT_PIN, 0);
    gpio_set_level(TP_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    gpio_set_level(TP_RST_PIN, 1);   /* 拉高 RST */
    vTaskDelay(pdMS_TO_TICKS(6));

    gpio_set_level(TP_INT_PIN, 0);   /* INT 保持低电平选地址 */
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(TP_INT_PIN, 1);   /* 释放 INT */
    vTaskDelay(pdMS_TO_TICKS(5));

    gpio_set_direction(TP_INT_PIN, GPIO_MODE_INPUT);  /* 恢复为输入(上拉) */
}

/* ==================== 初始化 ==================== */

static tp_chip_t tp_probe_chip(void);   /* 定义在下方 (探测芯片) */

tp_chip_t touch_panel_init(void) {
    /* I2C_NUM_1 独立总线 */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TP_SDA_PIN,
        .scl_io_num = TP_SCL_PIN,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = TP_I2C_FREQ,
    };
    esp_err_t ret = i2c_param_config(TP_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 配置失败: %s", esp_err_to_name(ret));
        return TP_CHIP_NONE;
    }
    ret = i2c_driver_install(TP_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C 驱动安装失败: %s", esp_err_to_name(ret));
        return TP_CHIP_NONE;
    }

    /* RST 输出(初始低), INT 输入+上拉 */
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << TP_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = false,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_cfg);
    gpio_set_level(TP_RST_PIN, 0);

    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << TP_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&int_cfg);

    /* 先做 GT911 复位时序 (对 CST816/FT6236 无害, 只是把 RST 拉高) */
    gt911_reset();
    gpio_set_level(TP_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(80));   /* 等触摸芯片上电就绪 */

    /* 按地址探测芯片 */
    tp_probe_chip();

    if (s_chip == TP_CHIP_NONE) {
        ESP_LOGW(TAG, "未检测到触摸芯片 (GT911/CST816/FT6236)");
    } else {
        const char *names[] = {"?", "GT911", "CST816", "FT6236"};
        ESP_LOGI(TAG, "触摸芯片: %s (I2C addr=0x%02X, 分辨率=%dx%d)",
                 names[s_chip], s_addr, s_res_x, s_res_y);
    }
    /* 启动后台触摸看门狗 (V1.0.68) */
    if (!s_wd_task) {
        xTaskCreate(touch_watchdog_task, "touch_wd", 4096, NULL, 2, &s_wd_task);
        ESP_LOGI(TAG, "触摸看门狗已启动");
    }
    return s_chip;
}

/* 探测 I2C 上的触摸芯片并设置 s_chip/s_addr/s_res_x/s_res_y */
static tp_chip_t tp_probe_chip(void) {
    s_chip = TP_CHIP_NONE;
    s_addr = 0;

    if (tp_i2c_probe(GT911_ADDR_A) || tp_i2c_probe(GT911_ADDR_B)) {
        /* GT911 有两种地址, 用 ACK 的那个 */
        s_addr = tp_i2c_probe(GT911_ADDR_A) ? GT911_ADDR_A : GT911_ADDR_B;
        /* 读产品 ID 验证 (0x8140..0x8143 应为 "911") */
        uint8_t pid[4] = {0};
        if (tp_i2c_read16(GT911_REG_PID, pid, 4) == ESP_OK) {
            ESP_LOGI(TAG, "GT911 产品 ID: %c%c%c%c (addr=0x%02X)",
                     pid[0], pid[1], pid[2], pid[3], s_addr);
            if (pid[0] == '9' && pid[1] == '1' && pid[2] == '1') {
                s_chip = TP_CHIP_GT911;
            }
        }
        /* PID 读不到也按 GT911 处理 (有些面板 PID 寄存器被锁) */
        if (s_chip == TP_CHIP_NONE) {
            s_chip = TP_CHIP_GT911;
        }
        /* 读面板分辨率 (0x8048/49=X, 0x804A/4B=Y), 用于点击 hit-test 的坐标映射 */
        if (s_chip == TP_CHIP_GT911) {
            uint8_t r[4] = {0};
            if (tp_i2c_read16(GT911_REG_RES, r, 4) == ESP_OK) {
                int rx = ((int)r[1] << 8) | r[0];
                int ry = ((int)r[3] << 8) | r[2];
                if (rx >= 200 && rx <= 4096 && ry >= 200 && ry <= 4096) {
                    s_res_x = rx;
                    s_res_y = ry;
                }
            }
        }
    } else if (tp_i2c_probe(CST816_ADDR)) {
        s_addr = CST816_ADDR;
        s_chip = TP_CHIP_CST816;
        /* 读芯片 ID/版本 (0xA7-0xAA) 便于确认具体型号 */
        uint8_t id[4] = {0};
        if (tp_i2c_read8(0xA7, id, 4) == ESP_OK) {
            ESP_LOGI(TAG, "CST ID: A7=%02X A8=%02X A9=%02X AA=%02X",
                     id[0], id[1], id[2], id[3]);
        }
    } else if (tp_i2c_probe(FT6236_ADDR)) {
        s_addr = FT6236_ADDR;
        s_chip = TP_CHIP_FT6236;
    }
    return s_chip;
}

/* 等总线空闲 (读取任务计数归零), 返回 true 表示就绪 */
static bool wd_wait_idle(void) {
    uint32_t t0 = xTaskGetTickCount();
    while (s_busy > 0) {
        if ((uint32_t)(xTaskGetTickCount() - t0) > pdMS_TO_TICKS(WD_RECOVER_WAIT)) {
            ESP_LOGW(TAG, "等总线空闲超时 (busy=%d)", s_busy);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return true;
}

/* 记录一次恢复时间戳, 超过节流上限返回 false */
static bool wd_recover_throttle(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    int idx = s_wd_recover_idx;
    s_wd_recover_times[idx] = now;
    s_wd_recover_idx = (idx + 1) % WD_MAX_RECOVER;
    if (s_wd_recover_idx == 0) s_wd_window_full = true;   /* 一圈填满 */
    if (!s_wd_window_full) return true;                    /* 窗口未满, 不节流 */
    uint32_t oldest = s_wd_recover_times[s_wd_recover_idx];
    if (now - oldest < WD_THROTTLE_MS) {
        ESP_LOGE(TAG, "3分钟内恢复次数过多, 节流暂停 (避免反复重启)");
        return false;
    }
    return true;
}

/* 非阻塞请求恢复 (input.c 在连续失败后调用, 由看门狗任务执行) */
void touch_panel_request_recover(void) {
    s_recover_request = true;
}

void touch_panel_recover(void);   /* 前向声明 (soft_recover 需要) */

/* 软恢复: 只做 RST 脉冲, 不重装 I2C (总线没卡死时最快恢复) */
static void touch_panel_soft_recover(void) {
    ESP_LOGW(TAG, "看门狗: 触摸异常, 软复位芯片 (RST 脉冲)");
    if (s_chip == TP_CHIP_GT911) {
        gt911_reset();
    } else if (s_chip != TP_CHIP_NONE) {
        gpio_set_direction(TP_RST_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(TP_RST_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(TP_RST_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    if (s_addr && !tp_i2c_probe(s_addr)) {
        ESP_LOGW(TAG, "软复位后无 ACK, 升级为完整恢复");
        touch_panel_recover();
    }
}

/* V1.0.68 fix: 连续 I2C 读失败 (触摸芯片挂死 / 总线卡住, 表现为触摸失灵需重启) 时
 * 自动恢复: RST 脉冲复位芯片 → 重装 I2C 驱动清除卡死总线 → 重新探测.
 * 必须在看门狗任务里调用 (s_recovering 已置位, 总线已空闲). */
void touch_panel_recover(void) {
    if (!wd_wait_idle()) return;
    ESP_LOGW(TAG, "执行完整恢复 (RST 脉冲 + 重装 I2C + 重探测)");
    /* 1. 复位芯片 */
    if (s_chip == TP_CHIP_GT911) {
        gt911_reset();
    } else {
        gpio_set_direction(TP_RST_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(TP_RST_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(TP_RST_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    /* 2. 重装 I2C 驱动 (SDA 被从机拉低卡死时, 重装可清掉总线状态) */
    esp_err_t del = i2c_driver_delete(TP_I2C_PORT);
    if (del != ESP_OK && del != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "i2c_driver_delete 返回 %s, 继续", esp_err_to_name(del));
    }
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TP_SDA_PIN,
        .scl_io_num = TP_SCL_PIN,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = TP_I2C_FREQ,
    };
    i2c_param_config(TP_I2C_PORT, &conf);
    esp_err_t ins = i2c_driver_install(TP_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ins != ESP_OK && ins != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_driver_install 失败: %s", esp_err_to_name(ins));
        return;
    }
    /* 3. 重新探测 */
    tp_probe_chip();
    if (s_chip == TP_CHIP_NONE) {
        ESP_LOGE(TAG, "恢复后仍未探测到触摸芯片");
    } else {
        ESP_LOGI(TAG, "触摸恢复成功: %s", s_chip == TP_CHIP_GT911 ? "GT911" :
                 s_chip == TP_CHIP_CST816 ? "CST816" : "FT6236");
    }
}

/* 看门狗任务: 周期检查触摸健康, 异常则强制恢复 */
static void touch_watchdog_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WD_CHECK_MS));
        if (s_recovering || s_chip == TP_CHIP_NONE) {
            if (s_recover_request) s_recover_request = false;   /* 已禁用触摸, 丢弃请求 */
            s_wd_win_reads = s_wd_win_fails = 0;
            s_wd_stuck_ms = 0;
            continue;
        }
        uint32_t fails = s_wd_win_fails, reads = s_wd_win_reads;
        uint32_t stuck = s_wd_stuck_ms;   /* 不清零: 由读取路径累计/复位 */
        s_wd_win_reads = s_wd_win_fails = 0;

        bool do_full = s_recover_request;
        bool do_soft = false;
        if (s_recover_request) s_recover_request = false;
        if (fails >= WD_FAIL_HARD) { do_full = true; }
        else if (reads >= WD_READS_MIN && fails * WD_RATE_DIV > reads) { do_soft = true; }
        else if (stuck >= WD_STUCK_MS) { do_soft = true; }
        if (!do_full && !do_soft) continue;

        ESP_LOGW(TAG, "看门狗判定异常: fails=%u/%u stuck=%ums %s",
                 (unsigned)fails, (unsigned)reads, (unsigned)stuck,
                 do_full ? "→完整恢复" : "→软恢复");
        if (!wd_recover_throttle()) continue;
        s_recovering = true;
        if (do_full) touch_panel_recover();
        else         touch_panel_soft_recover();
        s_recovering = false;
    }
}

tp_chip_t touch_panel_get_chip(void) {
    return s_chip;
}

/* V1.0.69: 是否检测到触摸屏 (有触摸机型 = true, 无触摸机型 = false).
 * 供 input.c 按机型分支物理键语义用. */
bool touch_panel_is_present(void) {
    return (s_chip != TP_CHIP_NONE);
}

void touch_panel_get_resolution(int *max_x, int *max_y) {
    if (max_x) *max_x = s_res_x;
    if (max_y) *max_y = s_res_y;
}

/* ==================== 读点 ==================== */

/* 读取多点 (每点 8 字节, 点1@0x814F; GT911 最多 5 点) */
static bool gt911_read_points(tp_point_t pts[TP_MAX_POINTS], int *count) {
    uint8_t status = 0;
    if (tp_i2c_read16(GT911_REG_STATUS, &status, 1) != ESP_OK) {
        return false;
    }
    if (!(status & 0x80)) {
        /* 无新数据 */
        for (int i = 0; i < TP_MAX_POINTS; i++) pts[i].pressed = false;
        *count = 0;
        return true;
    }
    int n = status & 0x0F;
    if (n <= 0) {
        for (int i = 0; i < TP_MAX_POINTS; i++) pts[i].pressed = false;
        *count = 0;
        /* 清状态, 准备下一次中断 */
        uint8_t z = 0;
        tp_i2c_write16(GT911_REG_STATUS, &z, 1);
        return true;
    }
    /* 一次读满 TP_MAX_POINTS 个点的原始数据 (5*8=40 字节) */
    uint8_t p[TP_MAX_POINTS * 8] = {0};
    if (tp_i2c_read16(GT911_REG_POINT1, p, sizeof(p)) != ESP_OK) {
        return false;
    }
    int cnt = (n > TP_MAX_POINTS) ? TP_MAX_POINTS : n;
    for (int i = 0; i < cnt; i++) {
        const uint8_t *pp = p + i * 8;
        pts[i].pressed = true;
        pts[i].id = (uint8_t)i;
        pts[i].x = (int16_t)((pp[2] << 8) | pp[1]);   /* X = high<<8 | low */
        pts[i].y = (int16_t)((pp[4] << 8) | pp[3]);   /* Y = high<<8 | low */
    }
    for (int i = cnt; i < TP_MAX_POINTS; i++) pts[i].pressed = false;
    *count = cnt;
    /* 读完清状态位, GT911 才会重新拉低 INT 上报下一次触摸 */
    uint8_t z = 0;
    tp_i2c_write16(GT911_REG_STATUS, &z, 1);
    return true;
}

/*
 * ==================== CST836U 双指读取 ====================
 *
 * 帧布局 (以 skill ch03 逻辑分析仪实测帧为准, 非家族通识猜测):
 *   原厂每次中断读两段: reg0x02 手势(1B) + reg0x01 起 5 字节简帧. 实测
 *   F1 简帧 = 01 81 77 01 11, F7 抬起帧 = 00 41 77 01 11, 由此严格对齐
 *   CST816 标准寄存器模型 (本驱动从 reg0x00 连读 13 字节, d[] 下标=寄存器号):
 *
 *     d[0]   reg0x00 保留
 *     d[1]   reg0x01 触点数 n (实测: 按下=01, 抬起 F7=00 —— 抬起判定只看它)
 *     d[2..7]  reg0x02..0x07 点1: X高 X低 Y高 Y低 权重 面积
 *     d[8..13] reg0x08..0x0D 点2: 同上
 *
 *   12bit 坐标: X=(X高&0x0F)<<8|X低; X高高 4 bit=事件(0x8 接触),
 *               低 4 bit=手指编号(1 第一点 / 2 可区分的第二点).
 *   实测校验: F1 点1 d[2..5]=81 77 01 11 -> X=0x177, Y=0x011, 与抓包一致.
 *
 * 几何限制 (skill ch04 长边投影判据): 竖屏下两指沿长边(竖直)投影错开编号才
 *   给 2; 严格左右平齐退化为单指标识. 故第二点以 n==2 为主判据, 坐标合法性
 *   兜底, 手势层用第二点实时坐标做补偿, 不把编号位当唯一来源.
 *
 * 为什么需要跨帧状态机:
 *   本驱动是轮询而非 INT 中断, 双指在边缘姿态可能瞬间抖出幽灵第二点. 用
 *   "连续 N 帧确认"仲裁: 进入需连续确认帧, 一旦本帧丢失立即退出 (松手跟手).
 *   灵敏优先档把确认帧降到 1, 跟手优先, 接受反射屏偶发误触待真机微调. */

#define CST_FRAME_BYTES   13    /* reg0x00..0x0C: 保留 + 点数 + 两个 6 字节点 */
#define CST_TWO_CONFIRM   1     /* 第二点连续确认帧: 1=灵敏优先(跟手), 误触可调大 */

/* 驱动内双指总开关: 真机验证异常时可置 0 一键回退到原厂单指行为 */
#define CST_TWO_FINGER    1

static struct {
    bool     two_active;                 /* 已确认的双指状态 (对外生效) */
    uint8_t  confirm;                    /* 第二点连续出现帧计数 */
    int16_t  last_x[TP_MAX_POINTS];      /* 上一帧两点坐标, 用于就近/冻结判定 */
    int16_t  last_y[TP_MAX_POINTS];
    bool     last_valid[TP_MAX_POINTS];  /* 上一帧该点是否有效 */
} s_cst = {0};

/* 判断解析出的坐标是否落在面板合理范围 (滤除 0x000/0xFFF 之类冻结/噪声值) */
static bool cst_coord_valid(int x, int y) {
    return (x >= 0 && x <= s_res_x && y >= 0 && y <= s_res_y);
}

/* 按 CST816 标准 6 字节/点布局解析点 k (k=0->reg0x02, k=1->reg0x08).
 * 严格对齐 skill ch03 实测帧; 取代早期 4/5 字节步进的双布局猜测. */
static void cst_decode_point(const uint8_t *d, int k,
                             uint8_t *xh, int *x, int *y) {
    int b = 2 + k * 6;
    *xh = d[b];
    *x = (int)(((d[b] & 0x0F) << 8) | d[b + 1]);
    *y = (int)(((d[b + 2] & 0x0F) << 8) | d[b + 3]);
}

static bool cst816_read_points(tp_point_t pts[TP_MAX_POINTS], int *count) {
    uint8_t d[CST_FRAME_BYTES] = {0};
    if (tp_i2c_read8(0x00, d, sizeof(d)) != ESP_OK) {
        return false;
    }

    /* 抬起帧判定只看手指数 reg0x01 (实测 F7=00); 编号位在 F7 仍为 1, 不能用. */
    int n = (int)d[1];
    if (n <= 0) {
        for (int i = 0; i < TP_MAX_POINTS; i++) {
            pts[i].pressed = false;
            pts[i].id = 0;
            s_cst.last_valid[i] = false;
        }
        s_cst.two_active = false;
        s_cst.confirm = 0;
        *count = 0;
        return true;
    }

    /* 点1: reg0x02..0x05. 坐标非法(冻结/噪声帧)时保持上一帧, 避免跳变. */
    uint8_t xh1;
    int x1, y1;
    cst_decode_point(d, 0, &xh1, &x1, &y1);
    if (!cst_coord_valid(x1, y1)) {
        return true;
    }

    pts[0].pressed = true;
    pts[0].x = (int16_t)x1;
    pts[0].y = (int16_t)y1;
    pts[0].id = 0;
    s_cst.last_x[0] = pts[0].x;
    s_cst.last_y[0] = pts[0].y;
    s_cst.last_valid[0] = true;

    /* ---- 第二点 (总开关关闭时直接回退单指) ----
     * 主判据 n==2; 点1 编号位==2 作辅助 (长边投影错开时芯片置 2).
     * 点2 在 reg0x08..0x0B, 仍要求坐标落在面板内, 滤冻结/噪声. */
    bool two_raw = false;
    int x2 = 0, y2 = 0;
#if CST_TWO_FINGER
    if (n >= 2 || (xh1 & 0x0F) == 0x02) {
        uint8_t xh2;
        cst_decode_point(d, 1, &xh2, &x2, &y2);
        bool touching = (xh2 & 0xF0) == 0x80 || (xh2 & 0x0F) == 0x02;
        if (touching && cst_coord_valid(x2, y2)) {
            two_raw = true;
        }
    }
#endif

    /* 跨帧确认: 连续 CST_TWO_CONFIRM 帧有可信点2 才对外置双指;
     * 一旦本帧丢失立即退出 (松手/退化姿态要跟手). */
    if (two_raw) {
        if (s_cst.confirm < 255) s_cst.confirm++;
        if (s_cst.confirm >= CST_TWO_CONFIRM) s_cst.two_active = true;
    } else {
        s_cst.confirm = 0;
        s_cst.two_active = false;
    }

    if (s_cst.two_active) {
        pts[1].pressed = true;
        pts[1].x = (int16_t)x2;
        pts[1].y = (int16_t)y2;
        pts[1].id = 1;
        s_cst.last_x[1] = pts[1].x;
        s_cst.last_y[1] = pts[1].y;
        s_cst.last_valid[1] = true;
        *count = 2;
    } else {
        pts[1].pressed = false;
        pts[1].id = 0;
        s_cst.last_valid[1] = false;
        *count = 1;
    }
    return true;
}

static bool ft6236_read_points(tp_point_t pts[TP_MAX_POINTS], int *count) {
    uint8_t d[10] = {0};
    if (tp_i2c_read8(0x00, d, 10) != ESP_OK) {
        return false;
    }
    int n = d[0] & 0x0F;   /* 0x00 = TD_STATUS 点数 */
    if (n <= 0) {
        for (int i = 0; i < TP_MAX_POINTS; i++) pts[i].pressed = false;
        *count = 0;
        return true;
    }
    int cnt = (n > 2) ? 2 : n;   /* FT6236 硬件最多 2 点 */
    for (int i = 0; i < cnt; i++) {
        const uint8_t *q = d + 1 + i * 4;
        pts[i].pressed = true;
        pts[i].id = (uint8_t)i;
        pts[i].x = (int16_t)(((q[0] & 0x0F) << 8) | q[1]);
        pts[i].y = (int16_t)(((q[2] & 0x0F) << 8) | q[3]);
    }
    for (int i = cnt; i < TP_MAX_POINTS; i++) pts[i].pressed = false;
    *count = cnt;
    return true;
}

/* 诊断: 读失败计数 (每 2s 打一次日志, 判断是 I2C 问题还是数据噪声) */
static uint32_t s_diag_fail = 0, s_diag_ok = 0, s_diag_tick = 0;

bool touch_panel_read_points(tp_point_t pts[TP_MAX_POINTS], int *count) {
    if (!pts || !count || s_chip == TP_CHIP_NONE) {
        return false;
    }
    if (s_recovering) {
        /* 看门狗正在恢复: 免阻塞直接返回, UI 不卡顿, 保持上一帧状态 */
        return false;
    }
    s_busy++;
    bool ok;
    switch (s_chip) {
        case TP_CHIP_GT911:  ok = gt911_read_points(pts, count);  break;
        case TP_CHIP_CST816: ok = cst816_read_points(pts, count); break;
        case TP_CHIP_FT6236: ok = ft6236_read_points(pts, count); break;
        default:             s_busy--; return false;
    }
    s_busy--;
    tp_point_t *pt = &pts[0];
    if (ok) { s_diag_ok++; if (pt->pressed) s_diag_ok++; }
    else    { s_diag_fail++; }
    /* 看门狗统计 */
    {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        s_wd_win_reads++;
        if (!ok) {
            s_wd_win_fails++;
        } else if (pt->pressed) {
            if (pt->x == s_wd_last_x && pt->y == s_wd_last_y) {
                s_wd_stuck_ms += (now - s_wd_last_ms);
            } else {
                s_wd_stuck_ms = 0;
                s_wd_last_x = pt->x;
                s_wd_last_y = pt->y;
            }
        } else {
            s_wd_stuck_ms = 0;
        }
        s_wd_last_ms = now;
    }
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - s_diag_tick >= 2000) {
        if (s_diag_fail > 0 || (s_diag_ok > 0 && (s_diag_fail + s_diag_ok) > 100)) {
            ESP_LOGI(TAG, "诊断: 2s内 读成功=%u 失败=%u (失败率=%u%%)",
                     (unsigned)s_diag_ok, (unsigned)s_diag_fail,
                     (unsigned)(s_diag_fail * 100 / ((s_diag_fail + s_diag_ok) ? (s_diag_fail + s_diag_ok) : 1)));
        }
        s_diag_tick = now;
        s_diag_fail = s_diag_ok = 0;
    }
    return ok;
}

bool touch_panel_read(tp_point_t *pt) {
    tp_point_t pts[TP_MAX_POINTS];
    int count = 0;
    if (!pt) return false;
    bool ok = touch_panel_read_points(pts, &count);
    *pt = pts[0];
    return ok;
}

/* V1.0.68: 禁用触摸屏: 卸载 I2C 驱动(释放驱动内存) + 复位芯片状态.
 * 之后 touch_panel_read 恒返回 false (零开销), 触摸不再工作. */
void touch_panel_deinit(void) {
    if (s_chip != TP_CHIP_NONE) {
        s_recovering = true;            /* 通知看门狗/读取任务让路 */
        wd_wait_idle();
        i2c_driver_delete(TP_I2C_PORT);
        s_chip = TP_CHIP_NONE;
        s_addr = 0;
        s_res_x = TP_DEFAULT_RES_X;
        s_res_y = TP_DEFAULT_RES_Y;
        s_wd_win_reads = s_wd_win_fails = 0;
        s_wd_stuck_ms = 0;
        s_recovering = false;
        ESP_LOGI(TAG, "触摸屏已禁用, I2C 驱动已释放");
    }
}
