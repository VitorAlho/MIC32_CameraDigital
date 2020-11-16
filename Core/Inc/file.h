/*
 * file.h
 *
 *  Created on: 15 de nov de 2020
 *      Author: vitor.alho
 */

#ifndef SRC_FILE_H_
#define SRC_FILE_H_

#ifdef __cplusplus
	extern "C" {
#endif

	#include "main.h"
	#include "fatfs.h"
	#include "fatfs_sd.h"
	#include <stdlib.h>
	#include <stdio.h>

	typedef enum {
		FILE_MOUNT_ERROR = 1,
		FILE_OPEN_ERROR,
		FILE_READ_ERROR,
		FILE_WRITE_ERROR,
		FILE_CLOSE_ERROR,
	} fileErrorCode;

	fileErrorCode fileOpen (
							   FIL* fp,			/* Pointer to the blank file object */
							   const TCHAR* path,	/* Pointer to the file name */
							   BYTE mode			/* Access mode and file open mode flags */
							  );

	fileErrorCode fileClose (FIL* fp		/* Pointer to the file object to be closed */
				   	   	   	   );

	fileErrorCode fileRead (   FIL* fp, 	/* Pointer to the file object */
							   void* buff,	/* Pointer to data buffer */
							   UINT btr,	/* Number of bytes to read */
							   UINT* br	/* Pointer to number of bytes read */
							  );

	fileErrorCode fileWrite (	FIL* fp,			/* Pointer to the file object */
								const void* buff,	/* Pointer to the data to be written */
								UINT btw,			/* Number of bytes to write */
								UINT* bw			/* Pointer to number of bytes written */
							   );

	fileErrorCode fileMount (	FATFS* fs,			/* Pointer to the file system object (NULL:unmount)*/
								const TCHAR* path,	/* Logical drive number to be mounted/unmounted */
								BYTE opt			/* Mode option 0:Do not mount (delayed mount), 1:Mount immediately */
							   );

	fileErrorCode checkFileError ( fileErrorCode ferror, UART_HandleTypeDef *huart );

	uint8_t testFileLibrary( UART_HandleTypeDef *huart );

#ifdef __cplusplus
	}
#endif

#endif /* SRC_FILE_H_ */
