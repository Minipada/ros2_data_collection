# SPDX-FileCopyrightText: 2022-2026 David Bensoussan
# SPDX-License-Identifier: MPL-2.0

# base64_library's exported link interface carries Boost::headers (string_utils.hpp
# includes boost/ headers directly), so consumers need that target defined before
# dc_utilConfig's export file resolves the interface. CONFIG, not the deprecated
# FindBoost module (CMP0167).
find_package(Boost CONFIG REQUIRED)
