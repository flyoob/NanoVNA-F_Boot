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
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "fatfs.h"
#include "nt35510.h"

#if _USE_LFN
char Lfname[512];
#endif

/*******************************************************************************
* Function Name : get_free
* Description   : 得到磁盘剩余容量
* Param         : drv    : 磁盘编号     ("0:"/"1:")
*                 total  : 总容量      （单位KB）
*                 free   : 剩余容量    （单位KB）
* Return Code   : 0      : 正常
                  others : 错误代码
*******************************************************************************/
uint8_t get_free(uint8_t *drv, uint32_t *total, uint32_t *free)
{
    FRESULT res;
    FATFS *fs;
    uint32_t fre_clust=0, fre_sect=0, tot_sect=0;

    // 得到磁盘信息及空闲簇数量
    res = f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs);

    if (res == 0) {
        tot_sect = (fs->n_fatent-2)*fs->csize;       // 得到总扇区数
        fre_sect = fre_clust*fs->csize;              // 得到空闲扇区数
#if _MAX_SS != 512                                   // 扇区大小不是512字节,则转换为512字节
        tot_sect *= fs->ssize/512;
        fre_sect *= fs->ssize/512;
#endif
        *total = tot_sect>>1;     // 单位为KB
        *free = fre_sect>>1;      // 单位为KB
    }

    /* 通过 UART2 直接转发给 MCU */
    lcd_debug("total: %9d KB, free: %9d KB\r\n", *total, *free);

    return res;
}

/*******************************************************************************
* Function Name : get_file_size
* Description   : 追加模式打开一个文件
* Param         :
* Return Code   : DWORD/
*******************************************************************************/
DWORD get_size(char *path)
{     
    FIL fp;  
    f_open(&fp, path, FA_READ);
    DWORD size = f_size(&fp);
    f_close(&fp);
    return size;
}

FRESULT scan_path(void)
{
#if _FS_RPATH >= 2
    uint8_t path[10] = {0};

    f_getcwd((TCHAR *)path, sizeof(path));

    lcd_debug("current path : %s\r\n", path);
#endif
    return FR_OK;
}

/*******************************************************************************
* Function Name : scan_files
* Description   : 遍历文件
* Param         : path : 路径
* Return Code   : 执行结果
*******************************************************************************/
uint8_t scan_files(uint8_t * path)
{
    FRESULT res;
    char *fn;          // This function is assuming non-Unicode cfg.
    DIR dir;
    FILINFO fileinfo;  // 文件信息

    // 打开目录
    res = f_opendir(&dir, (const TCHAR*)path);
    if (res == FR_OK) {
#if _USE_LFN
        fileinfo.lfname = Lfname;
        fileinfo.lfsize = 512;
#endif
        while(1)
        {
            res = f_readdir(&dir, &fileinfo);                   // 读取目录下的一个文件
            if (res != FR_OK || fileinfo.fname[0] == 0) {
                break;     // 错误了/到末尾了,退出
            }
            if (fileinfo.fname[0] == '.') {
                continue;  // 忽略上级目录
            }
#if _USE_LFN               // 是否长文件名
    fn = *fileinfo.lfname ? fileinfo.lfname : fileinfo.fname;
#else
    fn = fileinfo.fname;
#endif
            // debug_info("%s/%s\r\n", (char *)path, (char *)fn);  /* 打印路径、文件名 */
            /* 通过 UART2 直接转发给 MCU */
            lcd_debug("<FILE> %10d    %s/%s\r\n", get_size(fn), (char *)path, (char *)fn);
        }
    }

    return res;
}

/*******************************************************************************
* Function Name : mf_scan_files
* Description   : 追加模式打开一个文件
* Param         :
* Return Code   :
*******************************************************************************/
FRESULT open_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
)
{
    FRESULT fr;

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK)
            f_close(fp);
    }
    return fr;
}

/*
=======================================
    判断文件是否存在
    FR_OK: File exist
    FR_NO_FILE: File not exist
    others: An error occured
=======================================
*/
FRESULT is_exist(const char* path)
{
    FRESULT fr;
    FILINFO fno;
    fr = f_stat(path, &fno);
    return fr;
}

void printf_ff_error(FRESULT fresult)
{
  switch(fresult)
  {
  case FR_OK:                   //(0)
    lcd_debug("FR_OK\r\n");
    break;
  case FR_DISK_ERR:             //(1)
    lcd_debug("FR_DISK_ERR\r\n");
    break;
  case FR_INT_ERR:              //(2)
    lcd_debug("FR_INT_ERR\r\n");
    break;
  case FR_NOT_READY:            //(3)
    lcd_debug("FR_NOT_READY\r\n");
    break;
  case FR_NO_FILE:              //(4)
    lcd_debug("FR_NO_FILE\r\n");
    break;
  case FR_NO_PATH:              //(5)
    lcd_debug("FR_NO_PATH\r\n");
    break;
  case FR_INVALID_NAME:         //(6)
    lcd_debug("FR_INVALID_NAME\r\n");
    break;
  case FR_DENIED:               //(7)
  case FR_EXIST:                //(8)
    lcd_debug("FR_EXIST\r\n");
    break;
  case FR_INVALID_OBJECT:       //(9)
    lcd_debug("FR_INVALID_OBJECT\r\n");
    break;
  case FR_WRITE_PROTECTED:      //(10)
    lcd_debug("FR_WRITE_PROTECTED\r\n");
    break;
  case FR_INVALID_DRIVE:        //(11)
    lcd_debug("FR_INVALID_DRIVE\r\n");
    break;
  case FR_NOT_ENABLED:          //(12)
    lcd_debug("FR_NOT_ENABLED\r\n");
    break;
  case FR_NO_FILESYSTEM:        //(13)
    lcd_debug("FR_NO_FILESYSTEM\r\n");
    break;
  case FR_MKFS_ABORTED:         //(14)
    lcd_debug("FR_MKFS_ABORTED\r\n");
    break;
  case FR_TIMEOUT:              //(15)
    lcd_debug("FR_TIMEOUT\r\n");
    break;
  case FR_LOCKED:               //(16)
    lcd_debug("FR_LOCKED\r\n");
    break;
  case FR_NOT_ENOUGH_CORE:      //(17)
    lcd_debug("FR_NOT_ENOUGH_CORE\r\n");
    break;
  case FR_TOO_MANY_OPEN_FILES:  //(18)
    lcd_debug("FR_TOO_MANY_OPEN_FILES\r\n");
    break;
  case FR_INVALID_PARAMETER:    //(19)
    lcd_debug("FR_INVALID_PARAMETER\r\n");
    break;
  }
}
