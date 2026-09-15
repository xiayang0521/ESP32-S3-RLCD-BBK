#ifndef BOOK_READER_H
#define BOOK_READER_H

#include <stdbool.h>
#include <stdint.h>

#include "st7305.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 打开电子书 (从 /sdcard/books 下的 txt 文件), 成功返回 true */
bool book_reader_open(const char *path);

bool book_reader_is_open(void);

/* 关闭阅读器: 释放文本缓冲 */
void book_reader_close(void);

/* 阅读器按键处理 (menu_action_t 值); 返回 true = 已消费 */
bool book_reader_handle_action(int action);

/* V1.0.68: 阅读器触摸命中 (屏幕坐标 400x300, 已含旋转映射).
 * 上半页=上一页, 下半页=下一页, 中间=进设置菜单. 返回 true=已消费. */
bool book_reader_handle_touch(int x, int y);

/* 渲染当前页 (含状态栏/页码) */
void book_reader_render(st7305_handle_t *lcd);

/* 主循环每帧轮询 (保留接口, 返回 false) */
bool book_reader_poll(void);

/* 当前文件名 (无路径, 用于标题) */
const char *book_reader_title(void);

/* 麦克风敲击检测已移除, 恒返回 false */
bool book_reader_knock_active(void);

/* 应用电子书设置: 夜间模式 / 显示页码 / 旋转方向
 * / 字体大小(0=20 1=24 2=28 3=32) / 边距(0窄 1中 2宽) / 行高(0紧凑 1标准 2宽松)
 * / 字距(0标准 1宽松) / 段落缩进(0关 1开)
 * 在打开书籍前或设置变更时调用 */
void book_reader_set_settings(bool knock, int sens, bool night, bool pagenum, int rot,
                              int fontstyle, int fontsize, int margin, int lineh, int gap,
                              int indent);

/* ==== 双指手势 (CST836U 两点触控) ====
 * 设计原则: 捏合途中只做轻量字号预览 (不重新分页), 双指抬起时一次性提交重排,
 * 避免逐档重建分页导致卡顿并跳回第 0 页。 */

/* 捏合增量预览: steps>0 放大一档, <0 缩小一档, 一次可跨多档。
 * 仅更新预览档位与画面 (夹取 0..3), 不切字体、不重新分页。
 * 返回 true 表示当前处于可预览的阅读页 (调用方应刷新画面)。 */
bool book_reader_pinch_font_delta(int steps);

/* 捏合结束: 提交预览档位。若档位确有变化则切字体并尽量按原阅读位置重新分页,
 * 通过 out_fontsize 回传最终生效档 (0..3), changed 回传是否真的改档。
 * 返回 true 表示当前处于阅读页且已处理 (调用方应刷新画面并持久化字号)。 */
bool book_reader_pinch_font_commit(int *out_fontsize, bool *changed);

/* 双指左右滑翻章: dir<0=上一章, dir>0=下一章。无章节或已到边界时返回 false。
 * 成功跳转返回 true。 */
bool book_reader_goto_adjacent_chapter(int dir);

/* 双指点击: 打开内部阅读菜单 (与长按 BOOT 同路径)。仅纯净阅读页可开, 返回是否已处理。 */
bool book_reader_open_menu(void);

/* V1.1.1: 阅读菜单交给外部 (os_dialog 列表模板) 呈现.
 * 打开阅读器时置 true → 阅读器内部不再自绘菜单, 由 page_book 用列表弹窗驱动. */
void book_reader_set_external_menu(bool en);
bool book_reader_menu_active(void);         /* 阅读菜单(目录/书签等)是否打开 */
int  book_reader_menu_count(void);          /* 当前菜单项数 */
const char *book_reader_menu_text(int i);   /* 当前菜单项文字 */
void book_reader_menu_choose(int idx);      /* 选中菜单项 (执行跳转/添加书签/关闭等) */
void book_reader_menu_cancel(void);         /* 关闭菜单回阅读 */
const char *book_reader_menu_take_msg(void);/* 取走一次性提示 (已添加书签/已清空), 无则空串 */

#ifdef __cplusplus
}
#endif

#endif
