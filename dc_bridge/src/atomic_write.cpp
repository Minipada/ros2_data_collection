// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

#include "dc_bridge/atomic_write.hpp"

#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace dc_bridge
{

void write_file_atomically(const std::string& path, const std::string& content)
{
  const std::string tmp_path = path + ".tmp";
  {
    std::ofstream out(tmp_path, std::ios::binary | std::ios::trunc);
    if (!out.good())
    {
      throw std::runtime_error("failed to open '" + tmp_path + "' for writing");
    }
    out << content;
  }
  // No fsync (same tradeoff as the intent queue, #265): the rename itself is what makes
  // the write crash-atomic; losing the last few milliseconds of writes on a hard power
  // loss is accepted.
  if (std::rename(tmp_path.c_str(), path.c_str()) != 0)
  {
    throw std::runtime_error("failed to rename '" + tmp_path + "' into place at '" + path + "'");
  }
}

}  // namespace dc_bridge
