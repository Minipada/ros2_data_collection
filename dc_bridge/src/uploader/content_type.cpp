#include "dc_bridge/uploader/content_type.hpp"

#include <cctype>

namespace dc_bridge::content_type
{

namespace
{
bool starts_with(const std::string& buf, const char* lit, std::size_t n)
{
  return buf.size() >= n && std::equal(lit, lit + n, buf.begin());
}

// "P<digit>" followed by whitespace, the Netpbm magic shape.
bool pnm_magic(const std::string& buf, char digit)
{
  return buf.size() >= 3 && buf[0] == 'P' && buf[1] == digit && std::isspace(static_cast<unsigned char>(buf[2]));
}

// Minimal UTF-8 validity check (matches Rust's std::str::from_utf8().is_ok()).
bool is_valid_utf8(const std::string& s)
{
  std::size_t i = 0;
  const std::size_t n = s.size();
  while (i < n)
  {
    unsigned char c = static_cast<unsigned char>(s[i]);
    std::size_t extra;
    if (c < 0x80)
    {
      extra = 0;
    }
    else if ((c & 0xE0) == 0xC0 && c >= 0xC2)
    {
      extra = 1;
    }
    else if ((c & 0xF0) == 0xE0)
    {
      extra = 2;
    }
    else if ((c & 0xF8) == 0xF0 && c <= 0xF4)
    {
      extra = 3;
    }
    else
    {
      return false;
    }
    if (i + extra >= n)
    {
      return false;
    }
    for (std::size_t k = 1; k <= extra; ++k)
    {
      if ((static_cast<unsigned char>(s[i + k]) & 0xC0) != 0x80)
      {
        return false;
      }
    }
    i += extra + 1;
  }
  return true;
}
}  // namespace

std::string sniff(const std::string& buf)
{
  if (starts_with(buf, "\x89PNG\r\n\x1a\n", 8))
  {
    return "image/png";
  }
  if (starts_with(buf, "\xff\xd8\xff", 3))
  {
    return "image/jpeg";
  }
  if (starts_with(buf, "GIF87a", 6) || starts_with(buf, "GIF89a", 6))
  {
    return "image/gif";
  }
  // ISO base media file format (mp4/mov): a `ftyp` box at offset 4.
  if (buf.size() >= 12 && buf.compare(4, 4, "ftyp") == 0)
  {
    return "video/mp4";
  }
  if (buf.size() >= 12 && starts_with(buf, "RIFF", 4) && buf.compare(8, 4, "AVI ") == 0)
  {
    return "video/x-msvideo";
  }
  // Matroska/WebM EBML header.
  if (starts_with(buf, "\x1a\x45\xdf\xa3", 4))
  {
    return "video/x-matroska";
  }
  // Netpbm map formats (nav map PGM being the DC-relevant one).
  if (pnm_magic(buf, '2') || pnm_magic(buf, '5'))
  {
    return "image/x-portable-graymap";
  }
  if (pnm_magic(buf, '3') || pnm_magic(buf, '6'))
  {
    return "image/x-portable-pixmap";
  }
  if (pnm_magic(buf, '1') || pnm_magic(buf, '4'))
  {
    return "image/x-portable-bitmap";
  }
  if (starts_with(buf, "%PDF-", 5))
  {
    return "application/pdf";
  }
  // Printable UTF-8 (YAML metadata, JSON, logs…): valid UTF-8 with no NUL.
  if (!buf.empty() && buf.find('\0') == std::string::npos && is_valid_utf8(buf))
  {
    return "text/plain; charset=utf-8";
  }
  return "application/octet-stream";
}

bool is_video(const std::string& content_type)
{
  return content_type.rfind("video/", 0) == 0;
}

}  // namespace dc_bridge::content_type
