#pragma once

#include <string>

#include "adas/adas_app.h"

/// Load the runtime config from JSON. \param[out] ok False when the file did not parse.
inline AdasApp::Config loadAdasRuntimeConfig(const std::string& path, bool* ok = nullptr)
{
  return AdasApp::Config::loadFromFile(path, ok);
}
