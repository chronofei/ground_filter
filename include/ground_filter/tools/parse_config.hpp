#ifndef GROUND_FILTER_TOOLS_PARSE_CONFIG_HPP_
#define GROUND_FILTER_TOOLS_PARSE_CONFIG_HPP_

// C/C++
#include <map>
#include <string>
#include <memory>
#include <mutex>
#include <assert.h>

// JSON
#include <jsoncpp/json/json.h>

// project
#include "ground_filter/global_defination/global_defination.hpp"
#include "ground_filter/tools/file_manager.hpp"

namespace ground_filter
{

class ParseConfig
{
public:
	~ParseConfig();

	static std::shared_ptr<ParseConfig> GetInstance();
	const Json::Value& getConfig(const std::string& name);

private:
	ParseConfig();

private:
	std::map<std::string, Json::Value> config;

}; // end class ParseConfig

} // end namespace ground_filter

#endif