/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Open Source Robotics Foundation, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Open Source Robotics Foundation, Inc. nor the
*     names of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

// rosbag_snapshot/test/test_write_perf.cpp
//
// Write-performance test for rosbag_snapshot
//
//   1. read every message from an input bag and re-publish it to a running `snapshot` node
//   2. wait until the node reports every message as buffered
//   3. time the /trigger_snapshot service call (= the snapshot write), N times
//   4. re-read each written bag and check that every ingested message is in it
//
// Messages are compared by serialized payload, not by timestamp: the snapshotter stamps
// buffered messages with their receipt time, so timestamps in the output differ from the input.

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag_snapshot_msgs/SnapshotStatus.h>
#include <rosbag_snapshot_msgs/TriggerSnapshot.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace fs = boost::filesystem;

// topic -> sorted payload hashes (sorted => order-independent "is it present" check)
using Digest = std::map<std::string, std::vector<std::size_t>>;

std::size_t hashPayload(const rosbag::MessageInstance& m)
{
  std::vector<uint8_t> buf(m.size());
  ros::serialization::OStream os(buf.data(), buf.size());
  m.write(os);
  return std::hash<std::string>()(std::string(buf.begin(), buf.end()));
}

Digest digestOf(const std::string& path)
{
  rosbag::Bag bag(path);  // read mode
  rosbag::View view(bag);
  Digest d;
  for (const rosbag::MessageInstance& m : view)
    d[m.getTopic()].push_back(hashPayload(m));
  for (auto& kv : d)
    std::sort(kv.second.begin(), kv.second.end());
  return d;
}

void expectSameDigest(const Digest& want, const Digest& got)
{
  for (const auto& kv : want)
  {
    auto it = got.find(kv.first);
    ASSERT_NE(it, got.end()) << "topic missing from snapshot: " << kv.first;
    EXPECT_EQ(kv.second.size(), it->second.size()) << "message count differs on " << kv.first;
    EXPECT_TRUE(kv.second == it->second) << "payload mismatch on " << kv.first;
  }
  EXPECT_EQ(want.size(), got.size()) << "snapshot contains unexpected topics";
}

template <typename Pred>
bool waitUntil(Pred pred, double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (pred())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return pred();
}

// Tracks how many messages the snapshotter currently holds per topic (from /snapshot_status).
class StatusListener
{
public:
  explicit StatusListener(ros::NodeHandle& nh) : sub_(nh.subscribe("snapshot_status", 10, &StatusListener::cb, this))
  {
  }
  std::map<std::string, int> counts()
  {
    std::lock_guard<std::mutex> l(m_);
    return counts_;
  }

private:
  void cb(const rosbag_snapshot_msgs::SnapshotStatus::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> l(m_);
    counts_.clear();
    for (const auto& t : msg->topics)
      counts_[t.topic] = t.delivered_msgs;
  }
  ros::Subscriber sub_;
  std::mutex m_;
  std::map<std::string, int> counts_;
};
}  // namespace

TEST(SnapshotWritePerf, IngestWriteVerify)
{
  ros::NodeHandle nh, pnh("~");
  std::string bag_path, out_dir;
  int repeats;
  pnh.param<std::string>("bag", bag_path, "");
  pnh.param<std::string>("out_dir", out_dir, fs::temp_directory_path().string());
  pnh.param("repeats", repeats, 3);

  if (bag_path.empty())
  {
    std::cout << "[  SKIPPED ] no bag given (set SNAPSHOT_TEST_BAG or pass bag:=...)" << std::endl;
    return;
  }

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // ---- 1. inspect input bag, build the expected digest -------------------------------------
  const Digest expected = digestOf(bag_path);

  rosbag::Bag in_bag(bag_path);
  rosbag::View view(in_bag);

  std::map<std::string, int> expected_counts;
  std::size_t payload_bytes = 0;
  for (const rosbag::MessageInstance& m : view)
  {
    expected_counts[m.getTopic()]++;
    payload_bytes += m.size();
  }

  std::map<std::string, ros::Publisher> pubs;
  std::vector<std::string> topics;
  for (const rosbag::ConnectionInfo* c : view.getConnections())
  {
    if (pubs.count(c->topic))
      continue;  // one publisher per topic
    ros::AdvertiseOptions opts(c->topic, std::max(expected_counts[c->topic], 1), c->md5sum, c->datatype, c->msg_def);
    pubs[c->topic] = nh.advertise(opts);
    topics.push_back(c->topic);
  }
  ASSERT_FALSE(pubs.empty()) << "bag has no messages";

  StatusListener status(nh);

  // ---- 2. ingest: wait for the snapshot node to subscribe, then publish everything ---------
  bool connected = waitUntil(
      [&]
      {
        for (auto& kv : pubs)
          if (kv.second.getNumSubscribers() < 1)
            return false;
        return true;
      }
      , 60.0);
  ASSERT_TRUE(connected) << "snapshot node did not subscribe to all bag topics";

  for (const rosbag::MessageInstance& m : view)
    pubs[m.getTopic()].publish(m);  // MessageInstance is directly publishable (same as `rosbag play`)

  bool ingested = waitUntil(
      [&]
      {
        const auto c = status.counts();
        for (const auto& kv : expected_counts)
        {
          auto it = c.find(kv.first);
          if (it == c.end() || it->second != kv.second)
            return false;
        }
        return true;
      }
      , 120.0);
  std::string err;
  if (!ingested)
  {
    std::stringstream ss;
    for (const auto& kv : expected_counts)
      ss << "  " << kv.first << ": expected " << kv.second << ", buffered " << status.counts()[kv.first] << "\n";
    err = ss.str();
  }
  ASSERT_TRUE(ingested) << "snapshot node did not buffer all published messages "
                           "(dropped on the subscriber side, or a buffer limit is active)\n" + err;

  // ---- 3. + 4. write snapshot (timed) and verify -------------------------------------------
  ASSERT_TRUE(ros::service::waitForService("trigger_snapshot", ros::Duration(30.0)));

  std::vector<double> seconds;
  std::uintmax_t out_bytes = 0;
  for (int i = 0; i < repeats; ++i)
  {
    const std::string out = (fs::path(out_dir) / fs::unique_path("snapshot_perf_%%%%-%%%%.bag")).string();

    rosbag_snapshot_msgs::TriggerSnapshot srv;
    srv.request.filename = out;
    srv.request.topics = topics;  // only the bag's topics; start/stop_time left at 0 = whole buffer

    const auto t0 = std::chrono::steady_clock::now();
    const bool called = ros::service::call("trigger_snapshot", srv);
    const auto t1 = std::chrono::steady_clock::now();

    ASSERT_TRUE(called) << "trigger_snapshot call failed";
    ASSERT_TRUE(srv.response.success) << srv.response.message;
    ASSERT_TRUE(fs::exists(out)) << out;

    seconds.push_back(std::chrono::duration<double>(t1 - t0).count());
    out_bytes = fs::file_size(out);

    ASSERT_NO_FATAL_FAILURE(expectSameDigest(expected, digestOf(out))) << "run " << i;
    fs::remove(out);
  }

  // ---- report ------------------------------------------------------------------------------
  std::sort(seconds.begin(), seconds.end());
  const double best = seconds.front();
  const double median = seconds[seconds.size() / 2];
  std::size_t n_msgs = 0;
  for (const auto& kv : expected_counts)
    n_msgs += kv.second;

  std::cout << std::fixed << std::setprecision(3) << "\n[ PERF ] snapshot write\n"
            << "  messages        : " << n_msgs << " on " << topics.size() << " topics\n"
            << "  payload         : " << payload_bytes / 1e6 << " MB\n"
            << "  bag on disk     : " << out_bytes / 1e6 << " MB\n"
            << "  write time      : best " << best << " s, median " << median << " s (" << seconds.size() << " runs)\n"
            << "  throughput      : " << n_msgs / median << " msg/s, " << payload_bytes / 1e6 / median << " MB/s\n"
            << std::endl;

  RecordProperty("write_seconds_best", std::to_string(best).c_str());
  RecordProperty("write_seconds_median", std::to_string(median).c_str());
  RecordProperty("messages", static_cast<int>(n_msgs));
  RecordProperty("payload_mb", std::to_string(payload_bytes / 1e6).c_str());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "snapshot_write_perf");
  return RUN_ALL_TESTS();
}
