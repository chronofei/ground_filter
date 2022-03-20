#ifndef GROUND_FILTER_TOOLS_FILE_MANAGER_HPP_
#define GROUND_FILTER_TOOLS_FILE_MANAGER_HPP_

// C/C++
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace ground_filter
{

class FileManager
{
public:
  static bool IsFileExist(std::string path);
  static bool CreateFile(std::ofstream& ofs, std::string file_path);
  static bool CreateDirectory(std::string directory_path);
  static bool EmptyFile(std::ofstream &ofs, std::string file_path);
  static bool getFileNameWithPattern(const std::string& path, const std::string& pattern, std::vector<std::string>& file_name);
  static bool endWith(const std::string &original, const std::string &pattern);
  static std::vector<std::string> split(const std::string &str, const std::string &delimiter);
  
}; // end class FileManager

} // end namespace ground_filter
#endif
