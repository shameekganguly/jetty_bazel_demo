#include <chrono>
#include <condition_variable>
#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/convert/StdTypes.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <mutex>
#include <thread>

using gz::transport::Node;

using namespace std::chrono_literals;

class GzSimConnection {
public:
  // Invalidated once this class is destroyed.
  Node *GetNode() { return &node; }

  // Connect to Gazebo by subscribing to the stats topic and waiting for
  // simulation to be unpaused and for 100ms sim time to elapse.
  bool Initialize(std::chrono::steady_clock::duration timeout) {
    constexpr std::chrono::steady_clock::duration kMinSimTimeForInit = 100ms;
    std::mutex callback_mutex;
    std::chrono::steady_clock::duration last_sim_time{0};
    bool init_failed = false;
    std::condition_variable loop_cv;
    std::function<void(const gz::msgs::WorldStatistics &stats_msg)>
        callback_fn = [&callback_mutex, &last_sim_time, &init_failed,
                       &loop_cv](const gz::msgs::WorldStatistics &stats_msg) {
          std::unique_lock l(callback_mutex);
          std::chrono::steady_clock::duration sim_time =
              gz::msgs::Convert(stats_msg.sim_time());
          if (sim_time < last_sim_time) {
            gzerr << "Sim time was reset before initialization completed.\n";
            init_failed = true;
            loop_cv.notify_all();
            return;
          }
          last_sim_time = gz::msgs::Convert(stats_msg.sim_time());
          loop_cv.notify_all();
        };
    node.Subscribe("/world/default/stats", callback_fn);
    std::unique_lock l(callback_mutex);
    bool finished = loop_cv.wait_for(
        l, timeout, [kMinSimTimeForInit, &init_failed, &last_sim_time] {
          return init_failed || (last_sim_time > kMinSimTimeForInit);
        });
    if (!finished) {
      gzerr << "Timed out while waiting for server initialization.\n";
      return false;
    }
    return !init_failed;
  }

private:
  Node node;
};

int main(int argc, char **argv) {
  gz::common::Console::SetVerbosity(4);
  gzmsg << "Starting demo client!\n";
  GzSimConnection conn;
  if (!conn.Initialize(5s)) {
    gzerr << "Failed to initialize.\n";
    return -1;
  }
  gzmsg << "Connection initialized\n";

  // Spawn the model.
  constexpr std::string_view kModelPath = "bazel.sdf";
  gz::msgs::EntityFactory ef_msg;
  ef_msg.set_sdf_filename(kModelPath);
  gz::msgs::Boolean reply;
  bool result;
  // bool success = conn.GetNode()->Request("/world/default/create", ef_msg,
  // 1000,
  //                                        reply, result);

  std::mutex serviced_mutex;
  bool serviced = false;
  std::condition_variable serviced_cv;
  std::function<void(const gz::msgs::Boolean &, const bool)> callback_fn =
      [&serviced_mutex, &serviced, &serviced_cv, &reply,
       &result](const gz::msgs::Boolean &reply_in, const bool result_in) {
        reply = reply_in;
        result = result_in;
        std::unique_lock l(serviced_mutex);
        serviced = true;
        serviced_cv.notify_all();
      };
  bool success =
      conn.GetNode()->Request("/world/default/create", ef_msg, callback_fn);
  if (!success) {
    gzerr << "Failed to request model to be spawned!";
    return -1;
  }
  {
    std::unique_lock l(serviced_mutex);
    bool finished =
        serviced_cv.wait_for(l, 1s, [&serviced] { return serviced; });
    if (!finished) {
      gzerr << "Service call timed out";
      return -1;
    }
  }
  if (!result) {
    gzerr << "Create service call result negative!";
    return -1;
  }
  gzmsg << "Requested model to be spawned\n";

  return 0;
}
