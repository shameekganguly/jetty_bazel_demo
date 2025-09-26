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
    const std::string kStatsTopic{"/world/default/stats"};

    std::mutex callback_mutex;
    std::chrono::steady_clock::duration last_sim_time{0};
    bool paused = false;
    bool init_failed = false;
    std::condition_variable loop_cv;
    std::function<void(const gz::msgs::WorldStatistics &stats_msg)>
        callback_fn = [&callback_mutex, &last_sim_time, &paused, &init_failed,
                       &loop_cv](const gz::msgs::WorldStatistics &stats_msg) {
          std::unique_lock l(callback_mutex);
          std::chrono::steady_clock::duration sim_time =
              gz::msgs::Convert(stats_msg.sim_time());
          paused = stats_msg.paused();
          if (sim_time < last_sim_time) {
            gzerr << "Sim time was reset before initialization completed.\n";
            init_failed = true;
            loop_cv.notify_all();
            return;
          }
          last_sim_time = gz::msgs::Convert(stats_msg.sim_time());
          loop_cv.notify_all();
        };

    // Subscribe and wait for updates from the stats topic to be processed.
    node.Subscribe(kStatsTopic, callback_fn);
    std::unique_lock l(callback_mutex);
    bool finished = loop_cv.wait_for(
        l, timeout, [kMinSimTimeForInit, &init_failed, &last_sim_time] {
          return init_failed || (last_sim_time > kMinSimTimeForInit);
        });
    node.Unsubscribe(kStatsTopic);
    if (!finished) {
      gzerr << "Timed out while waiting for server initialization.\n";
      if (paused) {
        gzerr << "Unpause the simulator and try again.\n";
      }
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
  if (!conn.Initialize(/*timeout=*/5s)) {
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
  bool success =
      conn.GetNode()->Request("/world/default/create", ef_msg,
                              /*timeout=*/1000 /*ms*/, reply, result);
  if (!success) {
    gzerr << "Failed to request model to be spawned!\n";
    return -1;
  }
  if (!result) {
    gzerr << "Create service call result negative!\n";
    return -1;
  }
  gzmsg << "Requested model to be spawned\n";

  return 0;
}
