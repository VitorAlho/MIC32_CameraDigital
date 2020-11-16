/*
 * file.c
 *
 *  Created on: 15 de nov de 2020
 *      Author: vitor.alho
 */

#include "file.h"

fileErrorCode fileOpen (
			   FIL* fp,			/* Pointer to the blank file object */
			   const TCHAR* path,	/* Pointer to the file name */
			   BYTE mode			/* Access mode and file open mode flags */
			  ){

	FRESULT fr;						/* FatFs function common result code */

	//Abre o arquivo como leitura

	fr = f_open( fp, path, mode );

	if(fr != FR_OK){

		return FILE_OPEN_ERROR;

	}

	return FR_OK;
}

fileErrorCode fileClose (FIL* fp		/* Pointer to the file object to be closed */
			   ){

	FRESULT fr;						/* FatFs function common result code */

	fr = f_close(fp);

	if( fr != FR_OK )
	{

		return FILE_CLOSE_ERROR;

	}

	return FR_OK;

}

fileErrorCode fileRead (FIL* fp, 	/* Pointer to the file object */
			   void* buff,	/* Pointer to data buffer */
			   UINT btr,	/* Number of bytes to read */
			   UINT* br	/* Pointer to number of bytes read */
			  ){

	FRESULT fr;						/* FatFs function common result code */

	fr = f_read( fp, buff, btr,	br );

	//Se ocorrer algum erro mostra na UART e aborta

	if( fr != FR_OK )
	{

		return FILE_READ_ERROR;

	}

	return FR_OK;

}

fileErrorCode fileWrite (FIL* fp,			/* Pointer to the file object */
				const void* buff,	/* Pointer to the data to be written */
				UINT btw,			/* Number of bytes to write */
				UINT* bw			/* Pointer to number of bytes written */
			   ){

	FRESULT fr;						/* FatFs function common result code */

	fr = f_write( fp, buff, btw, bw ); /* Escreve dados no arquivo */

	if( fr != FR_OK )
	{

		return FILE_WRITE_ERROR;

	}

	return FR_OK;

}

fileErrorCode fileMount (FATFS* fs,			/* Pointer to the file system object (NULL:unmount)*/
				const TCHAR* path,	/* Logical drive number to be mounted/unmounted */
				BYTE opt			/* Mode option 0:Do not mount (delayed mount), 1:Mount immediately */
			   ) {

	FRESULT fr;						/* FatFs function common result code */

	//Prepara a área de trabalho para o FatFs

	fr = f_mount( fs, path, opt );

	if( fr != FR_OK )
	{
		return FILE_MOUNT_ERROR;
	}

	return FR_OK;

}

fileErrorCode checkFileError ( fileErrorCode ferror, UART_HandleTypeDef *huart ){

	char mensagem[ 100 ];

	uint32_t backupAddr = (uint32_t)mensagem;

	char *pointer = (char *)mensagem;

	uint32_t size = 0;

	switch( ferror ){

		case FILE_MOUNT_ERROR:

			sprintf( mensagem, "Erro ao montar a unidade" );

		break;

		case FILE_OPEN_ERROR:

			sprintf( mensagem, "Erro ao abrir o arquivo" );

		break;

		case FILE_CLOSE_ERROR:

			sprintf( mensagem, "Erro ao fechar o arquivo" );

		break;

		case FILE_READ_ERROR:

			sprintf( mensagem, "Erro ao ler o arquivo" );

		break;

		case FILE_WRITE_ERROR:

			sprintf( mensagem, "Erro ao escrever no arquivo" );

		break;

		default:

			return FR_OK;

		break;

	}

	while(*pointer++ != 0) size++;

	pointer = (char *)backupAddr;

	HAL_UART_Transmit( huart, (uint8_t *)mensagem, size, 100 );

	return ferror;

}

uint8_t testFileLibrary( UART_HandleTypeDef *huart ) {

	FATFS fs0, fs1;					/* Work area (filesystem object) for logical drives */

	FIL fsrc, fdst;					/* File objects */

	BYTE buffer[4096];   			/* File copy buffer */

	fileErrorCode fr;						/* FatFs function common result code */

	UINT br, bw;					/* File read/write count */

	int32_t size;

	char texto[50];

	/////////////////////////////////////////////////

	//Prepara a área de trabalho para o FatFs

	fr = fileMount( &fs0, "", 0 );

	if(checkFileError ( fr, huart ) == FR_OK){

		//Abre o arquivo como leitura
		fr = fileOpen( &fsrc, "testeFoto.txt", FA_READ | FA_WRITE | FA_CREATE_ALWAYS );

		if(checkFileError ( fr, huart ) == FR_OK){

			fileWrite( &fsrc, "Funcionou!\r\n", 11, &bw ); /* Escreve dados no arquivo */

			fr = fileClose( &fsrc );

			if(checkFileError ( fr, huart ) == FR_OK){

				fr = fileOpen( &fsrc, "testeFoto.txt", FA_READ );

				if(checkFileError ( fr, huart ) == FR_OK){
					//Loop de leitura de setores (512 bytes), lê até acabar o arquivo

					do {
						//Lê um setor do arquivo e armazena no buffer temporário

						fr = fileRead( &fsrc, buffer, 512, (UINT *)&br );

						if(checkFileError ( fr, huart ) == FR_OK){

							buffer[br] = 0;

							HAL_UART_Transmit( huart, buffer, 12, 100 );

						}

					}
					while(br == 512);

					fr = fileClose(&fsrc);
				}
			}
		}
	}

	return 0;
	/////////////////////////////////////////////////////////////////////////////////////////////
}
