#include "ground_filter/tools/parse_config.hpp"

namespace ground_filter
{

static std::shared_ptr<ParseConfig> parseConfig = nullptr;
static std::once_flag parseConfigFlag;

std::shared_ptr<ParseConfig> ParseConfig::GetInstance()
{
  std::call_once(parseConfigFlag, [&]{parseConfig = std::shared_ptr<ParseConfig>(new ParseConfig());});
  return parseConfig;
}

const Json::Value& ParseConfig::getConfig(const std::string& name)
{
  if(config.find(name) == config.end())
    return Json::Value();
  else
    return config[name];
}


ParseConfig::ParseConfig()
{
  std::vector<std::string> file_names;

  assert(FileManager::getFileNameWithPattern(WORK_SPACE_PATH + "/config", ".json", file_names));

  for (auto iter=file_names.begin(); iter!=file_names.end(); iter++)
  {
    std::ifstream ifs;
    ifs.open(*iter);
    assert(ifs.is_open());

    Json::Reader reader;
    Json::Value root;

    assert(reader.parse(ifs, root, false));

    config.insert(std::pair<std::string, Json::Value>(iter->substr(iter->find_last_of("/")+1, iter->find_last_of(".")-iter->find_last_of("/")-1), root));

    ifs.close();
  }
}

ParseConfig::~ParseConfig()
{
  // TODO
}

} // end namespace ground_filter