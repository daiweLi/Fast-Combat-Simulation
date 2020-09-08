// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           TacViewFile_T.cpp
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

#include "TacViewFile_T.h"

#include <windows.h>

using namespace TacView;

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           静态变量。
*   @{
*/
char                            TacViewFile_T::file_str[net_buffer_size];      
/** @}  */

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          构造函数。
*   @details        构造函数，创建时默认被执行的函数。
*/
TacViewFile_T::TacViewFile_T()
{
	m_file = NULL;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          析构函数。
*   @details        析构函数，销毁时默认被执行的函数。
*/
TacViewFile_T::~TacViewFile_T()
{
	if (m_file != NULL)
	{
		fclose(m_file);
		m_file = NULL;
	}
}

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
int TacViewFile_T::Open(
	const char* file_name,
	const Header_T &header)
{
	// 判断是否已经打开
	if (m_file != NULL)
	{
		fclose(m_file);
		m_file = NULL;
	}

	// 生成数据
	BuildString(header, net_buffer_size, file_str);

	// 打开文件，写入数据
	errno_t err_code = fopen_s(&m_file, file_name, "w+t");
	if (err_code == 0)
	{
		fwrite(file_str, lstrlenA(file_str), 1, m_file);
	}
	else
	{
		// 文件打开失败
		return 2;
	}

	return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          文件关闭函数。
*   @details        文件关闭函数。
*   @retval         0               正常
*   @retval         1               错误
*   @retval         2               错误 文件未打开
*/
int TacViewFile_T::Close()
{
	// 判断文件是否打开
	if (m_file == NULL)
	{
		// 文件未打开
		return 2;
	}

	fclose(m_file);
	m_file = NULL;

	delete[] file_str;
	return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          在文件中添加记录。
*   @details        在文件中添加记录。
*   @param[in]      state           仿真推进内容，详见 ACMI 格式说明。
*   @retval         0               正常
*   @retval         1               错误
*   @retval         2               错误 文件未打开
*/
int TacViewFile_T::Step(
	const State_T &state)
{
	// 判断文件是否打开
	if (m_file == NULL)
	{
		// 文件未打开
		return 2;
	}

	// 生成数据
	BuildString(state, net_buffer_size, file_str);

	// 写入文件
	fwrite(file_str, lstrlenA(file_str), 1, m_file);

	return 0;
}

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
int TacViewFile_T::BuildString(
	const Header_T &header,
	const int string_length,
	char* string)
{
	// 初始化数据
	memset(string, 0, sizeof(char) * string_length);

	char file_str_line[max_str];
	memset(file_str_line, 0, sizeof(file_str_line));

	// 构建数据内容
	snprintf(file_str_line, sizeof(file_str_line), "FileType=text/acmi/tacview\n");
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "FileVersion=2.2\n");
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,DataSource=%s\n", header.data_source);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,DataRecorder=%s\n", header.data_recorder);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,ReferenceTime=%s\n", header.reference_time);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,RecordingTime=%s\n", header.recording_time);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,Author=%s\n", header.author);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,Title=%s\n", header.title);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,Category=%s\n", header.category);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,Briefing=%s\n", header.briefing);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,Debriefing=%s\n", header.debriefing);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,Comments=%s\n", header.comments);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,ReferenceLongitude=%.7f\n", header.reference_longitude);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	snprintf(file_str_line, sizeof(file_str_line), "0,ReferenceLatitude=%.7f\n", header.reference_latitude);
	strcat_s(string, sizeof(char) * string_length, file_str_line);

	return 0;
}

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
int TacViewFile_T::BuildString(
	const State_T &state,
	const int string_length,
	char* string)
{
	// 初始化数据
	memset(string, 0, sizeof(char) * string_length);

	char file_str_line[max_str];
	memset(file_str_line, 0, sizeof(file_str_line));

	// 构建数据内容
	snprintf(file_str_line, sizeof(file_str_line), "#%.2f\n", state.time);
	strcat_s(string, sizeof(char) * string_length, file_str_line);
	for (int index = 0; index < state.object_count && index < max_object; index++)
	{
		// id
		snprintf(file_str_line, sizeof(file_str_line), "%d,", state.object[index].id);
		strcat_s(string, sizeof(char) * string_length, file_str_line);

		// 坐标
		if (state.object[index].coordinate_valid == 1)
		{
			snprintf(file_str_line, sizeof(file_str_line), "T=%.7f|%.7f|%.2f", state.object[index].coordinate_longitude, state.object[index].coordinate_latitude, state.object[index].coordinate_altitude);
			strcat_s(string, sizeof(char) * string_length, file_str_line);

			switch (state.object[index].coordinate_type)
			{
				// LLA
			case 0:
			{
				break;
			}

			// LLAUV
			case 1:
			{
				snprintf(file_str_line, sizeof(file_str_line), "|%.2f|%.2f", state.object[index].coordinate_u, state.object[index].coordinate_v);
				strcat_s(string, sizeof(char) * string_length, file_str_line);
				break;
			}

			// LLARPY
			case 2:
			{
				snprintf(file_str_line, sizeof(file_str_line), "|%.1f|%.1f|%.1f", state.object[index].coordinate_roll, state.object[index].coordinate_pitch, state.object[index].coordinate_yaw);
				strcat_s(string, sizeof(char) * string_length, file_str_line);
				break;
			}

			// LLARPYUVH
			case 3:
			{
				snprintf(file_str_line, sizeof(file_str_line), "|%.1f|%.1f|%.1f|%.2f|%.2f|%.2f", state.object[index].coordinate_roll, state.object[index].coordinate_pitch, state.object[index].coordinate_yaw, state.object[index].coordinate_u, state.object[index].coordinate_v, state.object[index].coordinate_heading);
				strcat_s(string, sizeof(char) * string_length, file_str_line);
				break;
			}

			// 错误
			default:
			{
				break;
			}
			}
		}

		// 基本信息
		if (state.object[index].base_valid == 1)
		{
			snprintf(file_str_line, sizeof(file_str_line), ",Name=%s", state.object[index].base_name);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Type=%s", state.object[index].base_type);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Pilot=%s", state.object[index].base_pilot);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Country=%s", state.object[index].base_country);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Coalition=%s", state.object[index].base_coalition);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Color=%s", state.object[index].base_color);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Label=%s", state.object[index].base_label);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Length=%.2f", state.object[index].base_length);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Width=%.2f", state.object[index].base_width);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Height=%.2f", state.object[index].base_height);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
		}

		// 状态
		if (state.object[index].state_valid == 1)
		{
			snprintf(file_str_line, sizeof(file_str_line), ",FocusTarget=%d", state.object[index].state_focus_target);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",Radius=%.2f", state.object[index].state_radius);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",EngagementRange=%.2f", state.object[index].state_engagement_range);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
		}

		// 雷达
		if (state.object[index].radar_valid == 1)
		{
			snprintf(file_str_line, sizeof(file_str_line), ",RadarMode=%d", state.object[index].radar_mode);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",RadarAzimuth=%.2f", state.object[index].radar_azimuth);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",RadarElevation=%.2f", state.object[index].radar_elevation);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",RadarRange=%.2f", state.object[index].radar_range);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",RadarHorizontalBeamwidth=%.2f", state.object[index].radar_horizontal_beamwidth);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",RadarVerticalBeamwidth=%.2f", state.object[index].radar_vertical_beamwidth);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
		}

		// 锁定目标
		if (state.object[index].locked_target_valid == 1)
		{
			snprintf(file_str_line, sizeof(file_str_line), ",LockedTarget=%d", state.object[index].locked_target_id);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",LockedTargetMode=%d", state.object[index].locked_target_mode);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",LockedTargetAzimuth=%.2f", state.object[index].locked_target_azimuth);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",LockedTargetElevation=%.2f", state.object[index].locked_target_elevation);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
			snprintf(file_str_line, sizeof(file_str_line), ",LockedTargetRange=%.2f", state.object[index].locked_target_range);
			strcat_s(string, sizeof(char) * string_length, file_str_line);
		}

		snprintf(file_str_line, sizeof(file_str_line), "\n");
		strcat_s(string, sizeof(char) * string_length, file_str_line);
	}

	return 0;
}
