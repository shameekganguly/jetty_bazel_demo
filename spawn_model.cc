/* Gazebo demo client

This client program talks to a running Gazebo Simulation and spawns a model
repeatedly until it is stopped.
- Gazebo must be running unpaused.
- `GZ_SIM_RESOURCE_PATH` must be set to point to the `resources/` directory.

CLI Help:
```shell
bazel run spawn_model -- --help
```
*/
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/common/SignalHandler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/convert/Pose.hh>
#include <gz/msgs/convert/StdTypes.hh>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>
#include <gz/utils/cli/CLI.hpp>

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
          last_sim_time = sim_time;
          loop_cv.notify_all();
        };

    // Subscribe and wait for updates from the stats topic to be processed.
    Node::Subscriber sub = node.CreateSubscriber(kStatsTopic, callback_fn);
    std::unique_lock l(callback_mutex);
    bool finished = loop_cv.wait_for(
        l, timeout, [kMinSimTimeForInit, &init_failed, &last_sim_time] {
          return init_failed || (last_sim_time > kMinSimTimeForInit);
        });
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

// Spawns the given model, then waits for it to settle down, then removes it and
// spawns it again. Removes the model on exit.
class SpawnController {
public:
  SpawnController(std::string_view model_path,
                  std::string_view model_entity_name,
                  gz::math::Pose3d spawn_pose, Node &node)
      : model_path_(model_path), model_entity_name_(model_entity_name),
        spawn_pose_(std::move(spawn_pose)), node_(&node) {
    sig_handler_.AddCallback(
        std::bind(&SpawnController::OnSignal, this, std::placeholders::_1));
  }

  ~SpawnController() {
    if (!Remove()) {
      gzerr << "Failed to remove object in controller destructor.\n";
    }
  }

  bool Run() {
    while (!stopped_) {
      if (!Spawn()) {
        return false;
      }
      WaitForSettle();
      // Wait for some time after the model has settled, then remove it.
      std::this_thread::sleep_for(1s);
      if (!Remove()) {
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    return true;
  }

private:
  void OnSignal(int sig) { stopped_ = true; }

  bool Spawn() {
    gz::msgs::EntityFactory req;
    req.set_sdf_filename(model_path_);
    req.set_name(model_entity_name_);
    gz::msgs::Set(req.mutable_pose(), spawn_pose_);
    gz::msgs::Boolean reply;
    bool result;
    bool success = node_->Request("/world/default/create", req,
                                  /*timeout=*/1000 /*ms*/, reply, result);
    if (!success) {
      gzerr << "Failed to request model to be spawned!\n";
      return false;
    }
    if (!result) {
      gzerr << "Create service call result negative!\n";
      return false;
    }
    gzdbg << "Requested model to be spawned\n";
    return true;
  }

  bool Remove() {
    gz::msgs::Entity req;
    req.set_type(gz::msgs::Entity::MODEL);
    req.set_name(model_entity_name_);
    gz::msgs::Boolean reply;
    bool result;
    bool success = node_->Request("/world/default/remove", req,
                                  /*timeout=*/1000 /*ms*/, reply, result);
    if (!success) {
      gzerr << "Failed to request model to be removed!\n";
      return false;
    }
    if (!result) {
      gzerr << "Remove service call result negative!\n";
      return false;
    }
    gzdbg << "Requested model to be removed\n";
    return true;
  }

  std::optional<gz::math::Vector3d> GetModelLinearVelocity() {
    std::mutex mtx;
    std::optional<gz::math::Pose3d> last_pose;
    std::optional<std::chrono::steady_clock::duration> last_time;
    std::optional<gz::math::Vector3d> linear_velocity;
    std::condition_variable cv;
    std::function<void(const gz::msgs::Pose_V &)> callback_fn =
        [&mtx, &last_pose, &last_time, &linear_velocity, &cv,
         this](const gz::msgs::Pose_V &poses_msg) {
          std::unique_lock l(mtx);
          std::optional<gz::math::Pose3d> pose;
          for (const gz::msgs::Pose &pose_msg : poses_msg.pose()) {
            if (pose_msg.name() == model_entity_name_) {
              pose = gz::msgs::Convert(pose_msg);
              break;
            }
          }
          if (!pose.has_value()) {
            return;
          }
          std::chrono::steady_clock::duration time =
              gz::msgs::Convert(poses_msg.header().stamp());
          if (last_pose.has_value() && last_time.has_value()) {
            std::chrono::duration<double> t_elapsed = (time - *last_time);
            linear_velocity =
                (pose->Pos() - last_pose->Pos()) / t_elapsed.count();
            cv.notify_all();
            return;
          }
          last_pose = pose;
          last_time = time;
        };
    Node::Subscriber sub =
        node_->CreateSubscriber("/world/default/pose/info", callback_fn);
    std::unique_lock l(mtx);
    bool finished = cv.wait_for(
        l, 1s, [&linear_velocity] { return linear_velocity.has_value(); });
    if (!finished) {
      return std::nullopt;
    }
    gzdbg << "Model linear velocity: " << *linear_velocity << "\n";
    return linear_velocity;
  }

  bool WaitForSettle() {
    std::chrono::steady_clock::time_point t_settle_start =
        std::chrono::steady_clock::now();
    while (!stopped_) {
      if ((std::chrono::steady_clock::now() - t_settle_start) > 5s) {
        gzwarn << "Model did not settle within wait time\n";
        return false;
      }
      std::this_thread::sleep_for(1s);
      if (std::optional<gz::math::Vector3d> vel = GetModelLinearVelocity();
          !vel.has_value()) {
        gzwarn << "Failed to get model linear velocity!\n";
        return false;
      } else {
        if (vel->Length() < 1e-1) {
          gzdbg << "Model settled\n";
          break;
        }
      }
    }
    return true;
  }

  const std::string model_path_;
  const std::string model_entity_name_;
  gz::math::Pose3d spawn_pose_;
  Node *node_;
  gz::common::SignalHandler sig_handler_;
  std::atomic<bool> stopped_ = false;
};

int main(int argc, char **argv) {
  gz::common::Console::SetVerbosity(3);
  CLI::App app("Jetty Bazel spawn model demo client");
  std::vector<double> spawn_pose_cli{0, 0, 0, 0, 0, 0};
  app.add_option(
         "--pose", spawn_pose_cli,
         "Optional pose offset e.g. 10 0 0 0 0 0 to apply when spawning the"
         "model. Units are meters for translation indices and radians for "
         "rotation indices.")
      ->expected(6);

  app.add_flag_callback("-v,--verbose",
                        [] { gz::common::Console::SetVerbosity(4); });

  CLI11_PARSE(app, argc, argv);
  gz::math::Pose3d spawn_pose(spawn_pose_cli[0], spawn_pose_cli[1],
                              spawn_pose_cli[2], spawn_pose_cli[3],
                              spawn_pose_cli[4], spawn_pose_cli[5]);

  gzmsg << "Starting demo client!\n";
  GzSimConnection conn;
  if (!conn.Initialize(/*timeout=*/5s)) {
    gzerr << "Failed to initialize.\n";
    return -1;
  }
  gzmsg << "Connection initialized, starting spawn controller.\n";
  gzmsg << "Press Ctrl+C to stop demo\n";

  // Create a SpawnController and run it.
  {
    constexpr std::string_view kModelPath = "bazel.sdf";
    SpawnController spawn_controller(kModelPath, /*model_entity_name=*/"Bazel",
                                     spawn_pose, *conn.GetNode());
    if (!spawn_controller.Run()) {
      gzerr << "Spawn controller failed, exiting\n";
      return -1;
    }
  }
  gzmsg << "Demo complete\n";

  return 0;
}
