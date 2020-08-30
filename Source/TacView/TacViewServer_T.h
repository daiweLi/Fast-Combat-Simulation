// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           TacViewServer_T.h
*   @brief          TacView 服务器。
*   @details        TacView 服务器，支持多个客户端连入。
*   @author         GaoYang
*   @date           20181126
*   @version        1.0.0.1
*   @par Copyright
*                   GaoYang
*   @par History
*                   1.0.0.1: GaoYang, 20181126, 首次创建
*                   1.0.0.2: lidaiwei, 20200808, 修正bug
*/

#ifndef Frame_IO_TacView_Server_H
#define Frame_IO_TacView_Server_H

#include ".\TacViewDefine.h"

#include <windows.h>
#include <stdio.h>
#include <string>
#include <list>

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          TacView 命名空间。
*   @details        TacView 命名空间。
*/
namespace TacView
{

	// --------------------------------------------------------------------------------------------------------------------------------
	/**
	*   @class          TacViewServer_T
	*   @brief          TacViewServer_T 用于 TacView 文件的生成。
	*   @details        TacViewServer_T 用于 TacView 文件的生成，详见 ACMI 格式说明。
	*/
	class TacViewServer_T
	{
	public:

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          构造函数。
		*   @details        构造函数，创建时默认被执行的函数。
		*/
		TacViewServer_T();

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          析构函数。
		*   @details        析构函数，销毁时默认被执行的函数。
		*/
		~TacViewServer_T();

	public:

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          服务器打开函数。
		*   @details        服务器打开函数。
		*   @param[in]      port            端口号，默认 42674 。
		*   @param[in]      header          战场基本信息，详见 ACMI 格式说明。
		*   @retval         0               正常
		*   @retval         1               错误
		*   @retval         2               错误 已经打开
		*   @retval         3               错误 WSAStartup 失败
		*   @retval         4               错误 Socket 初始化失败
		*   @retval         5               错误 Socket 绑定失败
		*   @retval         6               错误 Socket 监听失败
		*/
		int Open(
			const int port,
			const Header_T &header);
		
		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          服务器关闭函数。
		*   @details        服务器关闭函数。
		*   @retval         0               正常
		*   @retval         1               错误
		*   @retval         2               错误 未打开
		*/
		int Close();

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          向所有客户端填写仿真记录。
		*   @details        向所有客户端填写仿真记录。
		*   @param[in]      state           仿真推进内容，详见 ACMI 格式说明。
		*   @retval         0               正常
		*   @retval         1               错误
		*/
		int Send(
			const State_T &state);

	private:

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          监听。
		*   @details        监听，在独立线程中运行，对连入的客户端进行管理。
		*   @param[in,out]  para            多线程注册函数格式要求，创建线程时可传入参数。
		*   @retval         0               正常
		*   @retval         1               错误
		*/
		static unsigned long __stdcall Listen(
			void* para);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          接收数据。
		*   @details        接收数据，每个客户端都在独立线程中运行。
		*   @param[in,out]  para            多线程注册函数格式要求，创建线程时可传入参数。
		*   @retval         0               正常
		*   @retval         1               错误
		*/
		static unsigned long __stdcall Receive_0(
			void* para);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          接收数据。
		*   @details        接收数据，每个客户端都在独立线程中运行。
		*   @param[in,out]  para            多线程注册函数格式要求，创建线程时可传入参数。
		*   @retval         0               正常
		*   @retval         1               错误
		*/
		static unsigned long __stdcall Receive_1(
			void* para);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          接收数据。
		*   @details        接收数据，每个客户端都在独立线程中运行。
		*   @param[in,out]  para            多线程注册函数格式要求，创建线程时可传入参数。
		*   @retval         0               正常
		*   @retval         1               错误
		*/
		static unsigned long __stdcall Receive_2(
			void* para);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          接收数据。
		*   @details        接收数据，每个客户端都在独立线程中运行。
		*   @param[in,out]  para            多线程注册函数格式要求，创建线程时可传入参数。
		*   @retval         0               正常
		*   @retval         1               错误
		*/
		static unsigned long __stdcall Receive_3(
			void* para);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          接收数据。
		*   @details        接收数据，每个客户端都在独立线程中运行。
		*   @param[in,out]  para            多线程注册函数格式要求，创建线程时可传入参数。
		*   @retval         0               正常
		*   @retval         1               错误
		*/
		static unsigned long __stdcall SendAll(
			void* para);

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
		
		static bool						m_open;							//!< 是否已经打开，true 已经打开，false 未打开。
		static HANDLE					m_idle;							//!< 静态变量是否空闲。
		static SOCKET					m_server_socket;				//!< 服务器 Socket 。
		static HANDLE					m_listen_thread;				//!< 服务器 监听线程。

		static std::list<int>			m_client_id_idle_list;			//!< 客户端 id 闲置列表，初始 [0, 1, 2, 3] 。
		static std::list<int>			m_client_id_work_list;			//!< 客户端 id 工作列表，初始 [] 。
		static SOCKET					m_client_socket_list[4];		//!< 客户端 Socket 列表。
		static HANDLE					m_recv_thread_list[4];			//!< 客户端 接收线程列表。
		static char						m_recv_buffer_list[4][max_str];	//!< 客户端 接收缓冲区列表。
		static HANDLE					m_send_thread;					//!< 客户端 填写线程。
		static HANDLE					m_send_message_nonempty;		//!< 客户端 有未填写数据标志。
		static std::list<std::string>	m_send_message_list;			//!< 客户端 待填写消息列表。
		static std::string				m_send_history;					//!< 客户端 填写历史。
		static char                     file_str[net_buffer_size];      //!< 客户端 填写缓存。
		
	};
}


#endif // Frame_IO_TacView_Server_H