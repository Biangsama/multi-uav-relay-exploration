#pragma once

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

template<typename T>
T getYamlValue(const YAML::Node& node, const std::string& key)
{
    try
    {
        if (!node[key])
        {
            throw std::runtime_error("Missing key: " + key);
        }
        return node[key].as<T>();
    }
    catch (const YAML::TypedBadConversion<T>& e)
    {
        throw std::runtime_error("Bad conversion for key '" + key + "': " + e.what());
    }
}
