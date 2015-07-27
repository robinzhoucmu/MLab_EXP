#include "push_exp.h"
#include "geometry_msgs/WrenchStamped.h"

// Global flag indicating whether the robot is pushing the object.
bool flag_is_pushing;
std::vector<geometry_msgs::Wrench> ft_wrenches;

// Asynchronous force logging.
void CallBackForceLogging(const geometry_msgs::WrenchStamped& msg_force) {
  if (flag_is_pushing) {
    ft_wrenches.push_back(msg_force.wrench);
  }
}

void PushExp::LogPushForceAsync() {
  // Clear previous logged data.
  ft_wrenches.clear();
  // Start async thread for logging force.
  async_spinner.start();
}

void PushExp::AcquireObjectStablePose() {

}

int main(int argc, char* argv[]) {
  return 0;
}
