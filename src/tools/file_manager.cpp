#include "ground_filter/tools/file_manager.hpp"

namespace ground_filter
{

bool FileManager::IsFileExist(std::string path)
{
  if (access(path.c_str(), F_OK) == 0)
    return true;

  return false;
}

bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path)
{
  if (ofs.is_open())
    ofs.close();

  ofs.open(file_path.c_str(), std::ios::out);

  if (ofs.is_open())
    return true;

  return false;
}

bool FileManager::CreateDirectory(std::string directory_path)
{
  if (IsFileExist(directory_path))
    return true;

  if (mkdir(directory_path.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) == 0)
    return true;

  return false;
}

bool FileManager::EmptyFile(std::ofstream &ofs, std::string file_path)
{
  if(ofs.is_open())
    ofs.close();

  ofs.open(file_path.c_str(), std::ios::trunc);

  if (ofs.is_open())
    return true;

  return false;
}

bool FileManager::endWith(const std::string& original, const std::string& pattern)
{
  return original.length() >= pattern.length() &&
         original.substr(original.length() - pattern.length()) == pattern;
}

std::vector<std::string> FileManager::split(const std::string& str, const std::string& delimiter)
{
    std::vector<std::string> res;
    std::string::size_type last_index = str.find_first_not_of(delimiter, 0);
    std::string::size_type index = str.find_first_of(delimiter, last_index);

    while (std::string::npos != index || std::string::npos != last_index)
    {
        res.push_back(str.substr(last_index, index - last_index));
        last_index = str.find_first_not_of(delimiter, index);
        index = str.find_first_of(delimiter, last_index);
    }
    return res;
}

bool FileManager::getFileNameWithPattern(const std::string& path, const std::string& pattern, std::vector<std::string>& file_name)
{
  assert(!path.empty());
  assert(!pattern.empty());

  file_name.clear();

  DIR *dir;
  struct dirent *ptr;
  if ((dir = opendir(path.c_str())) == NULL)
  {
    std::cout << "Cann't open directory " << path << ", Please check!" << std::endl;
    return false;
  }

  while ((ptr = readdir(dir)) != NULL)
  {
    if (ptr->d_type == 8)
    {
      std::string name = ptr->d_name;
      if (endWith(name, pattern))
      {
        file_name.push_back(path + "/" + name);
      }
    }
  }

  closedir(dir);

  if (file_name.empty())
  {
    std::cout << "No file of type *" << pattern << " found in " << path << std::endl;
    return false;
  }
  
  std::sort(file_name.begin(), file_name.end());
  return true;
}

} // end namespace ground_filter