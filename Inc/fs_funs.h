/*******************************************************************************
* Module Name      : fs_funs.h
* Create Date      : 2016-08-01
* Copyright        :
* Description      : fatfs 使用说明
  FA_READ          指定读访问对象。可以从文件中读取数据。与 FA_WRITE 结合可以进行读写访问。
  FA_WRITE         指定写访问对象。可以向文件中写入数据。与 FA_READ 结合可以进行读写访问。
  FA_OPEN_EXISTING 打开文件。如果文件不存在，则打开失败。 ( 默认 )
  FA_OPEN_ALWAYS   如果文件存在，则打开；否则，创建一个新文件。
  FA_CREATE_NEW    创建一个新文件。如果文件已存在，则创建失败。
  FA_CREATE_ALWAYS 创建一个新文件。如果文件已存在，则它将被截断并覆盖。
* Revision History :
* Date          Author        Version        Notes
  2016-08-01    huanglong     V1.0           创建
*******************************************************************************/
#ifndef _FS_FUNS_H
#define _FS_FUNS_H

#include <stdint.h>

#include "fatfs.h"

/* 得到磁盘总容量和剩余容量 */
extern  uint8_t get_free(uint8_t *drv, uint32_t *total, uint32_t *free);
/* 扫描指定目录 */
extern  uint8_t scan_files(uint8_t * path);
/* 追加打开文件 */
extern  FRESULT open_append(FIL* fp, const char* path);
/* 获取文件大小 */
extern  unsigned long get_size(char *path);
// 判断文件存在
extern  FRESULT is_exist(const char* path);

extern  void printf_ff_error(FRESULT fresult);

#endif


