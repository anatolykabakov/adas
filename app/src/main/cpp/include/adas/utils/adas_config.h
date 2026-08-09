#pragma once

#include <string>

#include "adas/adas_app.h"

inline AdasApp::Config loadAdasRuntimeConfig(const std::string& path, bool* ok = nullptr)
{
  return AdasApp::Config::loadFromFile(path, ok);
}
