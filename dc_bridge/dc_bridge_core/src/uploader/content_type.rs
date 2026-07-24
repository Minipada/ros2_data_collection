//! Content-type sniffing for uploaded Files, replacing the Humble Go plugin's
//! `http.DetectContentType` call. Magic-byte based for the formats DC Measurements
//! actually produce (camera JPEGs, map PGM/PNG/YAML, videos), with a UTF-8 text
//! fallback — pure, no I/O, so it's trivially unit-testable.

/// Sniffs a content type from the first bytes of a File (512 bytes is plenty, matching
/// what `http.DetectContentType` consumed on Humble).
pub fn sniff(buf: &[u8]) -> &'static str {
    if buf.starts_with(b"\x89PNG\r\n\x1a\n") {
        return "image/png";
    }
    if buf.starts_with(b"\xff\xd8\xff") {
        return "image/jpeg";
    }
    if buf.starts_with(b"GIF87a") || buf.starts_with(b"GIF89a") {
        return "image/gif";
    }
    // ISO base media file format (mp4/mov): a `ftyp` box at offset 4.
    if buf.len() >= 12 && &buf[4..8] == b"ftyp" {
        return "video/mp4";
    }
    if buf.len() >= 12 && buf.starts_with(b"RIFF") && &buf[8..12] == b"AVI " {
        return "video/x-msvideo";
    }
    // Matroska/WebM EBML header.
    if buf.starts_with(b"\x1a\x45\xdf\xa3") {
        return "video/x-matroska";
    }
    // Netpbm map formats (nav map PGM being the DC-relevant one).
    if pnm_magic(buf, b'2') || pnm_magic(buf, b'5') {
        return "image/x-portable-graymap";
    }
    if pnm_magic(buf, b'3') || pnm_magic(buf, b'6') {
        return "image/x-portable-pixmap";
    }
    if pnm_magic(buf, b'1') || pnm_magic(buf, b'4') {
        return "image/x-portable-bitmap";
    }
    if buf.starts_with(b"%PDF-") {
        return "application/pdf";
    }
    // Printable UTF-8 (YAML metadata files, JSON, logs…).
    if !buf.is_empty() && std::str::from_utf8(buf).is_ok_and(|s| !s.contains('\0')) {
        return "text/plain; charset=utf-8";
    }
    "application/octet-stream"
}

/// `P<digit>` followed by whitespace, the Netpbm magic shape.
fn pnm_magic(buf: &[u8], digit: u8) -> bool {
    buf.len() >= 3 && buf[0] == b'P' && buf[1] == digit && buf[2].is_ascii_whitespace()
}

/// Whether a sniffed content type is a video — the gate for duration probing. Unlike the
/// Humble plugin (which also probed `application/octet-stream`), only `video/*` types
/// are probed, so ffprobe never runs on arbitrary binary Files.
pub fn is_video(content_type: &str) -> bool {
    content_type.starts_with("video/")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sniffs_the_formats_measurements_produce() {
        assert_eq!(sniff(b"\x89PNG\r\n\x1a\nrest"), "image/png");
        assert_eq!(sniff(b"\xff\xd8\xff\xe0jfif"), "image/jpeg");
        assert_eq!(sniff(b"P5\n640 480\n255\n"), "image/x-portable-graymap");
        assert_eq!(sniff(b"\x00\x00\x00\x20ftypisom....."), "video/mp4");
        assert_eq!(
            sniff(b"image: map.pgm\nresolution: 0.05\n"),
            "text/plain; charset=utf-8"
        );
        assert_eq!(sniff(b"\x00\x01\x02\x03"), "application/octet-stream");
        assert_eq!(sniff(b""), "application/octet-stream");
    }

    #[test]
    fn only_video_types_are_videos() {
        assert!(is_video("video/mp4"));
        assert!(is_video("video/x-msvideo"));
        assert!(!is_video("image/png"));
        assert!(!is_video("application/octet-stream"));
    }
}
