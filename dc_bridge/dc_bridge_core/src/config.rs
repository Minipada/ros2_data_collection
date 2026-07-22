//! Bridge configuration: which topics to subscribe to and the Fluent Forward tag each
//! one is sent under. Kept separate from `forwarder` so the wire protocol module never
//! has to know about ROS topic names.

/// One subscribed topic and the Fluent Forward tag its Records are sent under.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicConfig {
    pub topic: String,
    pub tag: String,
}

impl TopicConfig {
    /// Build a topic config, deriving the Fluent Forward tag from the topic name
    /// (leading `/` stripped, remaining `/` replaced with `.`) when no explicit tag
    /// is given.
    pub fn new(topic: impl Into<String>, tag: Option<String>) -> Self {
        let topic = topic.into();
        let tag = tag.unwrap_or_else(|| Self::derive_tag(&topic));
        Self { topic, tag }
    }

    fn derive_tag(topic: &str) -> String {
        let stripped = topic.strip_prefix('/').unwrap_or(topic);
        if stripped.is_empty() {
            "dc".to_string()
        } else {
            stripped.replace('/', ".")
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeConfig {
    pub topics: Vec<TopicConfig>,
    /// host:port the vendored Vector process listens on for Fluent Forward.
    pub vector_forward_addr: String,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            topics: Vec::new(),
            vector_forward_addr: "127.0.0.1:24224".to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derives_dotted_tag_from_topic_name() {
        let cfg = TopicConfig::new("/dc/measurement/uptime", None);
        assert_eq!(cfg.tag, "dc.measurement.uptime");
    }

    #[test]
    fn explicit_tag_overrides_derived_one() {
        let cfg = TopicConfig::new("/dc/measurement/uptime", Some("custom.tag".to_string()));
        assert_eq!(cfg.tag, "custom.tag");
    }

    #[test]
    fn root_topic_falls_back_to_default_tag() {
        let cfg = TopicConfig::new("/", None);
        assert_eq!(cfg.tag, "dc");
    }
}
