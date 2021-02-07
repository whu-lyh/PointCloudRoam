#ifdef TEMPLATE_POINTCLOUD_IO

#include <iomanip>
#include <iostream>

#include <liblas/liblas.hpp>

#include <boost/version.hpp>
#include <boost/filesystem.hpp>

#include "../include/PointCloudIO.h"
#include "../include/PointType.h"

namespace VisualTool {
	using namespace boost;

	namespace PointIO
	{
		const float Epsilon = std::numeric_limits<float>::epsilon();

		//parse las file less than 400M
		template <typename T>
		void parseLAS(liblas::Reader& reader, const typename pcl::PointCloud<T>::Ptr& cloud, const Offset& offset)
		{
		}

		template <>
		inline void parseLAS<PointXYZINTF>(liblas::Reader& reader, const pcl::PointCloud<PointXYZINTF>::Ptr& cloud, const Offset& offset)
		{
			while (reader.ReadNextPoint())
			{
				const liblas::Point& p = reader.GetPoint();
				PointXYZINTF pt;
				pt.x = p.GetX() - offset.x;
				pt.y = p.GetY() - offset.y;
				pt.z = p.GetZ() - offset.z;
				pt.intensity = p.GetIntensity();
				pt.num_of_returns = p.GetNumberOfReturns();
				pt.return_number = p.GetReturnNumber();
				pt.classification = p.GetClassification().GetClass();
				pt.gps_time = p.GetTime();
				pt.flighting_line_edge = p.GetFlightLineEdge();
				cloud->push_back(pt);
			}
		}

		template <>
		inline void parseLAS<PointXYZRGBINTF>(liblas::Reader& reader, const pcl::PointCloud<PointXYZRGBINTF>::Ptr& cloud, const Offset& offset)
		{
			while (reader.ReadNextPoint())
			{
				const liblas::Point& p = reader.GetPoint();
				PointXYZRGBINTF pt;
				pt.x = p.GetX() - offset.x;
				pt.y = p.GetY() - offset.y;
				pt.z = p.GetZ() - offset.z;
				pt.r = p.GetColor().GetRed() >> 8;
				pt.g = p.GetColor().GetGreen() >> 8;
				pt.b = p.GetColor().GetBlue() >> 8;
				pt.intensity = p.GetIntensity();
				pt.num_of_returns = p.GetNumberOfReturns();
				pt.return_number = p.GetReturnNumber();
				pt.classification = p.GetClassification().GetClass();
				pt.gps_time = p.GetTime();
				pt.flighting_line_edge = p.GetFlightLineEdge();

				cloud->push_back(pt);
			}
		}

		template <>
		inline void parseLAS<pcl::PointXYZ>(liblas::Reader& reader, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Offset& offset)
		{
			while (reader.ReadNextPoint())
			{
				const liblas::Point& p = reader.GetPoint();
				pcl::PointXYZ pt;
				pt.x = p.GetX() - offset.x;
				pt.y = p.GetY() - offset.y;
				pt.z = p.GetZ() - offset.z;
				cloud->push_back(pt);
			}
		}

		template <>
		inline void parseLAS<pcl::PointXYZRGB>(liblas::Reader& reader, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const Offset& offset)
		{
			while (reader.ReadNextPoint())
			{
				const liblas::Point& p = reader.GetPoint();
				pcl::PointXYZRGB pt;
				pt.x = p.GetX() - offset.x;
				pt.y = p.GetY() - offset.y;
				pt.z = p.GetZ() - offset.z;
				pt.r = p.GetColor().GetRed() >> 8; //liblas uint16 to store r color
				pt.g = p.GetColor().GetGreen() >> 8;
				pt.b = p.GetColor().GetBlue() >> 8;

				cloud->push_back(pt);
			}
		}

		//parse las file larger than 400M
		template <typename T>
		void parseLASmmf(const char* pFile, uint32_t pts_num, const liblas::Header& header, uint32_t pt_length,
			const typename pcl::PointCloud<T>::Ptr& cloud, const Offset& offset)
		{
		}

		template <>
		inline void parseLASmmf<PointXYZINTF>(const char* pFile, uint32_t pts_num, const liblas::Header& header, uint32_t pt_length,
			const pcl::PointCloud<PointXYZINTF>::Ptr& cloud, const Offset& offset)
		{
			switch (header.GetDataFormatId())
			{
			case liblas::ePointFormat0:
			case liblas::ePointFormat1:
			case liblas::ePointFormat2:
			case liblas::ePointFormat3:
			{
				for (int i = 0; i < pts_num; i++)
				{
					liblas::Point p(&header);
					//p.getdata() return  std::vector<uint8_t>
					memcpy(reinterpret_cast<char*>(&(p.GetData().front())), pFile + i * pt_length, pt_length);
					LIBLAS_SWAP_BYTES_N(p.GetData().front(), pt_length);
					PointXYZINTF pt;
					pt.x = p.GetX() - offset.x;
					pt.y = p.GetY() - offset.y;
					pt.z = p.GetZ() - offset.z;
					pt.intensity = p.GetIntensity();
					pt.num_of_returns = p.GetNumberOfReturns();
					pt.return_number = p.GetReturnNumber();
					pt.classification = p.GetClassification().GetClass();
					pt.gps_time = p.GetTime();
					pt.flighting_line_edge = p.GetFlightLineEdge();
					cloud->push_back(pt);
				}
				break;
			}
			default:
			{
				LOG(ERROR) << "暂不支持Point Data Record Format 4及以上";
				return;
			}
			}
		}

		template <>
		inline void parseLASmmf<PointXYZRGBINTF>(const char* pFile, uint32_t pts_num, const liblas::Header& header, uint32_t pt_length,
			const pcl::PointCloud<PointXYZRGBINTF>::Ptr& cloud, const VisualTool::Offset& offset)
		{
			switch (header.GetDataFormatId())
			{
			case liblas::ePointFormat0:
			case liblas::ePointFormat1:
			case liblas::ePointFormat2:
			case liblas::ePointFormat3:
			{
				for (int i = 0; i < pts_num; i++)
				{
					liblas::Point p(&header);
					memcpy(reinterpret_cast<char*>(&(p.GetData().front())), pFile + i * pt_length, pt_length);
					LIBLAS_SWAP_BYTES_N(p.GetData().front(), pt_length);
					PointXYZRGBINTF pt;
					pt.x = p.GetX() - offset.x;
					pt.y = p.GetY() - offset.y;
					pt.z = p.GetZ() - offset.z;
					pt.r = p.GetColor().GetRed() >> 8;
					pt.g = p.GetColor().GetGreen() >> 8;
					pt.b = p.GetColor().GetBlue() >> 8;
					pt.intensity = p.GetIntensity();
					pt.num_of_returns = p.GetNumberOfReturns();
					pt.return_number = p.GetReturnNumber();
					pt.classification = p.GetClassification().GetClass();
					pt.gps_time = p.GetTime();
					pt.flighting_line_edge = p.GetFlightLineEdge();
					cloud->push_back(pt);
				}
				break;
			}
			default:
			{
				LOG(ERROR) << "暂不支持Point Data Record Format 4及以上";
				return;
			}
			}
		}

		template <>
		inline void parseLASmmf<pcl::PointXYZ>(const char* pFile, uint32_t pts_num, const liblas::Header& header, uint32_t pt_length,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Offset& offset)
		{
			switch (header.GetDataFormatId())
			{
			case liblas::ePointFormat0:
			case liblas::ePointFormat1:
			case liblas::ePointFormat2:
			case liblas::ePointFormat3:
			{
				for (int i = 0; i < pts_num; i++)
				{
					liblas::Point p(&header);
					memcpy(reinterpret_cast<char*>(&(p.GetData().front())), pFile + i * pt_length, pt_length);
					LIBLAS_SWAP_BYTES_N(p.GetData().front(), pt_length);
					pcl::PointXYZ pt;
					pt.x = p.GetX() - offset.x;
					pt.y = p.GetY() - offset.y;
					pt.z = p.GetZ() - offset.z;
					cloud->push_back(pt);
				}
				break;
			}
			default:
			{
				LOG(ERROR) << "暂不支持Point Data Record Format 4及以上";
				return;
			}
			}
		}

		template <>
		inline void parseLASmmf<pcl::PointXYZRGB>(const char* pFile, uint32_t pts_num, const liblas::Header& header, uint32_t pt_length,
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const Offset& offset)
		{
			switch (header.GetDataFormatId())
			{
			case liblas::ePointFormat0:
			case liblas::ePointFormat1:
			case liblas::ePointFormat2:
			{
				for (int i = 0; i < pts_num; i++)
				{
					liblas::Point p(&header);
					memcpy(reinterpret_cast<char*>(&(p.GetData().front())), pFile + i * pt_length, pt_length);
					LIBLAS_SWAP_BYTES_N(p.GetData().front(), pt_length);
					pcl::PointXYZRGB pt;
					pt.x = p.GetX() - offset.x;
					pt.y = p.GetY() - offset.y;
					pt.z = p.GetZ() - offset.z;
					pt.r = p.GetColor().GetRed() >> 8;
					pt.g = p.GetColor().GetGreen() >> 8;
					pt.b = p.GetColor().GetBlue() >> 8;
					cloud->push_back(pt);
				}
				break;
			}
			case liblas::ePointFormat3:
			default:
			{
				LOG(ERROR) << "暂不支持Point Data Record Format 4及以上";
				return;
			}
			}
		}

		template <typename T>
		bool loadSingleLAS(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, Point3d& las_offset)
		{
			if (!FileUtility::FileExist(filename))
			{
				std::cerr << "'" << filename << "' doesn't exist!" << std::endl;
				return false;
			}

			if (FileUtility::IsDirectory(filename))
			{
				std::cerr << "'" << filename << "' is a directory!" << std::endl;
				return false;
			}

			if (cloud == nullptr)
			{
				std::cerr << "pointer 'cloud' is nullptr!" << std::endl;
				return false;
			}

			LOG(INFO) << "Load file: " << filename << std::endl;
			try
			{
				//check extension
				std::string ext = filesystem::extension(filename);
				if (ext.compare(".las"))
					std::cerr << "It's a inappropriate file format." << std::endl;

				//open file
				std::ifstream ifs;
				ifs.open(filename, std::ios::in | std::ios::binary);
				if (!ifs)
					return false;

				liblas::ReaderFactory f;
				liblas::Reader reader = f.CreateWithStream(ifs);
				const liblas::Header header = reader.GetHeader();
				uint8_t major_version = header.GetVersionMajor();
				uint8_t minor_version = header.GetVersionMinor();
				if (major_version > 1 || minor_version > 3)
				{
					std::cerr << "Currently this app doesn't support version newer than 1.4" << std::endl;
					return false;
				}

				//num of points
				uint32_t pts_count = header.GetPointRecordsCount();
				uint32_t offset = header.GetDataOffset();
				uint32_t pt_length = header.GetDataRecordLength();
				liblas::PointFormatName las_format = header.GetDataFormatId();
				double offset_x = header.GetOffsetX();
				double offset_y = header.GetOffsetY();
				double offset_z = header.GetOffsetZ();
				las_offset = Point3d(offset_x, offset_y, offset_z);

				//bounding box
				liblas::Bounds<double> bound = header.GetExtent();
				if (bound.empty())
				{
					std::cerr << "The header of this las doesn't contain extent. The cache cannot be built." << std::endl;
					return false;
				}

				//file size
				uintmax_t sz = filesystem::file_size(filename);

				if (0/*sz >= 400 * 1024 * 1024*/) //larger than 400M bug!!!
				{
					HANDLE file_handle = CreateFile(filename.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING,
						FILE_ATTRIBUTE_NORMAL, NULL);
					if (INVALID_HANDLE_VALUE == file_handle)
					{
						LOG(ERROR) << "Failed because of err." << std::endl;
						return false;
					}

					LARGE_INTEGER file_sz;
					GetFileSizeEx(file_handle, &file_sz);
					HANDLE mapping_handle = CreateFileMapping(file_handle, NULL, PAGE_READONLY, 0, 0, "LAS FILE MAPPING");
					if (INVALID_HANDLE_VALUE == mapping_handle)
					{
						std::cerr << "Mapping file failed." << std::endl;
						return false;
					}

					//system info
					SYSTEM_INFO sys_info;
					GetSystemInfo(&sys_info);
					DWORD allocation_granularity = sys_info.dwAllocationGranularity;

					LARGE_INTEGER cur_size;
					cur_size.QuadPart = 0;
					const uint32_t EACH_POINT_NUM = allocation_granularity * 200;
					const uint32_t EACH_SIZE = EACH_POINT_NUM * pt_length;
					char *pFile = nullptr;
					pFile = (char*)MapViewOfFile(mapping_handle, FILE_MAP_READ, 0, 0, EACH_SIZE * 1.1);

					DWORD err = ::GetLastError();
					assert(pFile != nullptr);

					pFile += offset;
					parseLASmmf<T>(pFile, EACH_POINT_NUM, header, pt_length, cloud, las_offset);
					UnmapViewOfFile(pFile - offset);
					cur_size.QuadPart += EACH_SIZE;
					while (true)
					{
						if (cur_size.QuadPart + EACH_SIZE * 1.1 < file_sz.QuadPart)
						{
							pFile = (char*)MapViewOfFile(mapping_handle, FILE_MAP_READ, cur_size.HighPart, cur_size.LowPart, EACH_SIZE * 1.1);
							pFile += offset;
							parseLASmmf<T>(pFile, EACH_POINT_NUM, header, pt_length, cloud, las_offset);
							UnmapViewOfFile(pFile - offset);
							cur_size.QuadPart += EACH_SIZE;
						}
						else //the last section
						{
							uint32_t rest_sz = file_sz.QuadPart - cur_size.QuadPart;
							pFile = (char*)MapViewOfFile(mapping_handle, FILE_MAP_READ, cur_size.HighPart, cur_size.LowPart, rest_sz);
							pFile += offset;
							uint32_t rest_count = pts_count - cur_size.QuadPart / pt_length;
							parseLASmmf<T>(pFile, rest_count, header, pt_length, cloud, las_offset);
							UnmapViewOfFile(pFile);
							break;
						}
					}
					CloseHandle(mapping_handle);
					CloseHandle(file_handle);
				}
				else //less than 400M
				{
					parseLAS<T>(reader, cloud, las_offset);
				}

				ifs.close();
			}
			catch (std::exception* e)
			{
				std::cerr << "Error occured when parsing las file: " << e->what() << std::endl;
			}

			return true;
		}

		template <typename T>
		bool saveLAS(const std::string& filepath, const typename pcl::PointCloud<T>::Ptr& cloud,
			const Point3d& offset)
		{
			if (cloud == nullptr)
			{
				std::cerr << "Pointer 'cloud' is nullptr!" << std::endl;
				return false;
			}

			if (cloud->empty())
			{
				std::cerr << "Point cloud is empty!" << std::endl;
				return false;
			}

			std::ofstream ofs;
			ofs.open(filepath, std::ios::out | std::ios::binary);
			ofs.setf(std::ios::fixed, std::ios::floatfield);
			ofs.precision(6);

			if (ofs.is_open())
			{
				liblas::Header header;
				header.SetDataFormatId(liblas::ePointFormat3);
				header.SetVersionMajor(1);
				header.SetVersionMinor(2);
				header.SetOffset(offset.x, offset.y, offset.z);
				header.SetScale(0.000001, 0.000001, 0.0001);
				header.SetPointRecordsCount(cloud->size());

				liblas::Writer writer(ofs, header);
				liblas::Point pt(&header);

				for (int i = 0; i < cloud->size(); ++i)
				{
					double x = static_cast<double>(cloud->points[i].x) + offset.x;
					double y = static_cast<double>(cloud->points[i].y) + offset.y;
					double z = static_cast<double>(cloud->points[i].z) + offset.z;

					pt.SetCoordinates(x, y, z);
					pt.SetIntensity(10);
					pt.SetColor(liblas::Color(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b));
					writer.WritePoint(pt);
				}
				ofs.flush();
				ofs.close();
			}

			LOG(INFO) << "Save file: " << filepath << std::endl;
			return true;
		}

		template <typename T>
		bool loadPCD(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud)
		{
			if (cloud == nullptr)
			{
				std::cerr << "pointer 'cloud' is nullptr!" << std::endl;
				return false;
			}

			if (pcl::io::loadPCDFile(filename, *cloud) != -1)
			{
				LOG(INFO) << "Load PCD file: '" << filename << "'" << std::endl;
				return true;
			}
			return false;
		}

		template <typename T>
		bool savePCD(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud,
			const Point3d& offset)
		{
			if (cloud == nullptr)
			{
				std::cerr << "pointer 'cloud' is nullptr!" << std::endl;
				return false;
			}

			if (cloud->empty())
			{
				std::cerr << "point cloud is empty!" << std::endl;
				return false;
			}

			if (std::fabs(offset.x) <= Epsilon && std::fabs(offset.y) <= Epsilon && std::fabs(offset.z) <= Epsilon)
			{
				pcl::io::savePCDFileBinary(filename, *cloud);
			}
			else
			{
				pcl::PointCloud<T> out_cloud;
				for (const auto& pt : cloud->points)
				{
					T out_pt = pt;
					out_pt.x/* += offset.x*/;
					out_pt.y/* += offset.y*/;
					out_pt.z/* += offset.z*/;
					out_cloud.push_back(out_pt);
				}

				pcl::io::savePCDFileBinary(filename, out_cloud);
			}

			LOG(INFO) << "Save PCD file: '" << filename << "'" << std::endl;

			return true;
		}

	} // namespace PointIO
} // namespace VisualTool

#endif // TEMPLATE_POINTCLOUD_IO

