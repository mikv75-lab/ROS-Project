#pragma once
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <chrono>

namespace topics {

struct TopicSpec {
  std::string id;    // stabile Kurz-ID
  std::string name;  // absoluter ROS-Topic-Name
  std::string type;  // z.B. geometry_msgs/msg/TwistStamped
  std::string qos;   // Profilname aus qos.yaml
};

class TopicsLoader {
public:
  TopicsLoader(const std::string& topics_yaml_path,
               const std::string& qos_yaml_path)
  : topics_yaml_path_(topics_yaml_path), qos_yaml_path_(qos_yaml_path)
  {
    YAML::Node root = YAML::LoadFile(topics_yaml_path_);
    if (!root["topics"] || !root["topics"].IsMap())
      throw std::runtime_error("'topics' mapping missing in topics.yaml");

    auto troot = root["topics"];
    for (auto it = troot.begin(); it != troot.end(); ++it) {
      const std::string node_key = it->first.as<std::string>();
      const YAML::Node  section  = it->second;
      if (!section.IsMap()) continue;

      if (section["subscribe"]) {
        for (const auto& s : section["subscribe"]) {
          TopicSpec spec;
          spec.name = s["name"].as<std::string>();
          spec.type = s["type"].as<std::string>();
          spec.qos  = s["qos"].as<std::string>();
          spec.id   = s["id"] ? s["id"].as<std::string>() : spec.name;
          subs_by_id_[node_key][spec.id] = spec;
          subscribe_specs_[node_key].push_back(spec);
        }
      }
      if (section["publish"]) {
        for (const auto& s : section["publish"]) {
          TopicSpec spec;
          spec.name = s["name"].as<std::string>();
          spec.type = s["type"].as<std::string>();
          spec.qos  = s["qos"].as<std::string>();
          spec.id   = s["id"] ? s["id"].as<std::string>() : spec.name;
          pubs_by_id_[node_key][spec.id] = spec;
          publish_specs_[node_key].push_back(spec);
        }
      }
    }

    qos_root_ = YAML::LoadFile(qos_yaml_path_);
    if (!qos_root_["profiles"] || !qos_root_["profiles"].IsMap())
      throw std::runtime_error("'profiles' missing in qos.yaml");
  }

  // ---- Nur-ID-API ---------------------------------------------------------
  std::string publish_topic(const std::string& node_key, const std::string& id) const {
    const TopicSpec* s = must_find(pubs_by_id_, node_key, id, "publish");
    return s->name;
  }
  std::string subscribe_topic(const std::string& node_key, const std::string& id) const {
    const TopicSpec* s = must_find(subs_by_id_, node_key, id, "subscribe");
    return s->name;
  }
  rclcpp::QoS qos_by_id(const std::string& dir,
                        const std::string& node_key,
                        const std::string& id) const {
    const TopicSpec* s =
      (dir == "publish") ? must_find(pubs_by_id_, node_key, id, dir)
                         : must_find(subs_by_id_, node_key, id, dir);
    return qos_profile(s->qos);
  }

  // optional für Einsicht/Debug
  const std::vector<TopicSpec>& publish_specs(const std::string& node_key) const {
    static const std::vector<TopicSpec> empty;
    auto it = publish_specs_.find(node_key);
    return it == publish_specs_.end() ? empty : it->second;
  }
  const std::vector<TopicSpec>& subscribe_specs(const std::string& node_key) const {
    static const std::vector<TopicSpec> empty;
    auto it = subscribe_specs_.find(node_key);
    return it == subscribe_specs_.end() ? empty : it->second;
  }

private:
  rclcpp::QoS qos_profile(const std::string& profile_name) const {
    auto profs = qos_root_["profiles"];
    if (!profs[profile_name]) {
      throw std::runtime_error("QoS profile not found: " + profile_name);
    }
    auto p = profs[profile_name];

    const std::string history = p["history"] ? p["history"].as<std::string>() : "KEEP_LAST";
    int depth = p["depth"] ? p["depth"].as<int>() : 10;

    rclcpp::QoS q = (history == "KEEP_ALL")
      ? rclcpp::QoS(rclcpp::KeepAll())
      : rclcpp::QoS(depth);

    if (history == "KEEP_ALL") q.keep_all(); else q.keep_last(depth);

    if (p["reliability"]) {
      const auto s = p["reliability"].as<std::string>();
      if (s == "BEST_EFFORT") q.best_effort();
      else if (s == "RELIABLE") q.reliable();
    }
    if (p["durability"]) {
      const auto s = p["durability"].as<std::string>();
      if (s == "TRANSIENT_LOCAL") q.transient_local();
      else if (s == "VOLATILE") q.durability_volatile();
    }
    if (p["lifespan_ms"]) {
      auto ms = std::chrono::milliseconds(p["lifespan_ms"].as<int64_t>());
      q.lifespan(ms);
    }
    return q;
  }

  static const TopicSpec* must_find(
      const std::unordered_map<std::string, std::unordered_map<std::string, TopicSpec>>& by_dir,
      const std::string& node_key,
      const std::string& id,
      const std::string& dir_label)
  {
    auto n = by_dir.find(node_key);
    if (n == by_dir.end())
      throw std::runtime_error("node_key '" + node_key + "' not found in topics (" + dir_label + ")");
    auto t = n->second.find(id);
    if (t == n->second.end())
      throw std::runtime_error("topic id '" + id + "' not found for node '" + node_key + "' (" + dir_label + ")");
    return &t->second;
  }

  std::string topics_yaml_path_;
  std::string qos_yaml_path_;
  YAML::Node qos_root_;

  std::unordered_map<std::string, std::unordered_map<std::string, TopicSpec>> pubs_by_id_;
  std::unordered_map<std::string, std::unordered_map<std::string, TopicSpec>> subs_by_id_;

  std::unordered_map<std::string, std::vector<TopicSpec>> publish_specs_;
  std::unordered_map<std::string, std::vector<TopicSpec>> subscribe_specs_;
};

} // namespace topics
