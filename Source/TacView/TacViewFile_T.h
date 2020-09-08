// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           TacViewFile_T.h
*   @brief          TacView 文件生成。
*   @details        TacView 文件生成，详见 ACMI 格式说明。
*   @author         GaoYang
*   @date           20181126
*   @version        1.0.0.1
*   @par Copyright
*                   GaoYang
*   @par History
*                   1.0.0.1: GaoYang, 20181126, 首次创建
*                   1.0.0.2: lidaiwei, 20200808, 修正bug
*/

#ifndef Frame_IO_TacView_File_H
#define Frame_IO_TacView_File_H

#include ".\TacViewDefine.h"

#include <stdio.h>


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          TacView 命名空间。
*   @details        TacView 命名空间。
*/
namespace TacView
{

	// --------------------------------------------------------------------------------------------------------------------------------
	/**
	*   @class          TacViewFile_T
	*   @brief          TacViewFile_T 用于 TacView 文件的生成。
	*   @details        TacViewFile_T 用于 TacView 文件的生成，详见 ACMI 格式说明。
	*/
	class TacViewFile_T
	{
	public:

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          构造函数。
		*   @details        构造函数，创建时默认被执行的函数。
		*/
		TacViewFile_T();

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          析构函数。
		*   @details        析构函数，销毁时默认被执行的函数。
		*/
		~TacViewFile_T();

	public:

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          文件打开函数。
		*   @details        文件打开函数。
		*   @param[in]      file_name       文件名，包含路径。
		*   @param[in]      header          战场基本信息，详见 ACMI 格式说明。
		*   @retval         0               正常
		*   @retval         1               错误
		*   @retval         2               错误 文件打开失败
		*/
		int Open(
			const char* file_name,
			const Header_T &header);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          文件关闭函数。
		*   @details        文件关闭函数。
		*   @retval         0               正常
		*   @retval         1               错误
		*   @retval         2               错误 文件未打开
		*/
		int Close();

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          在文件中添加记录。
		*   @details        在文件中添加记录。
		*   @param[in]      state           仿真推进内容，详见 ACMI 格式说明。
		*   @retval         0               正常
		*   @retval         1               错误
		*   @retval         2               错误 文件未打开
		*/
		int Step(
			const State_T &state);

	private:

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          将战场基本信息转化为 ACMI 格式的字符串。
		*   @details        将战场基本信息转化为 ACMI 格式的字符串。
		*   @param[in]      header          战场基本信息，详见 ACMI 格式说明。
		*   @param[in]      string_length   字符串长度。
		*   @param[out]     string          字符串，详见 ACMI 格式说明。
		*   @retval         0               正常
		*   @retval         1               错误
		*/
		int BuildString(
			const Header_T &header,
			const int string_length,
			char* string);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          将仿真推进内容转化为 ACMI 格式的字符串。
		*   @details        将仿真推进内容转化为 ACMI 格式的字符串。
		*   @param[in]      state           仿真推进内容，详见 ACMI 格式说明。
		*   @param[in]      string_length   字符串长度。
		*   @param[out]     string          字符串，详见 ACMI 格式说明。
		*   @retval         0               正常
		*   @retval         1               错误
		*/
		int BuildString(
			const State_T &state,
			const int string_length,
			char* string);

	private:

		FILE*							m_file;							//!< 保存的文件
		static char                     file_str[net_buffer_size];      //!< 保存缓存。
	};
}


#endif // Frame_IO_TacView_File_H