#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {

void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
}

bool withinJointLimits(const std::array<double, 7>& q) {
  constexpr std::array<double, 7> q_min = {
      -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  constexpr std::array<double, 7> q_max = {
       2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};

  for (size_t i = 0; i < 7; ++i) {
    if (q[i] < q_min[i] || q[i] > q_max[i]) {
      std::cerr << "Joint " << i + 1 << " target " << q[i]
                << " rad is outside limits [" << q_min[i] << ", " << q_max[i] << "]\n";
      return false;
    }
  }
  return true;
}

// Executes a smooth joint-space motion from current position to q_goal.
void moveToTarget(franka::Robot& robot,
                  const std::array<double, 7>& q_goal,
                  double duration) {
  std::array<double, 7> q_start{};
  double time = 0.0;
  bool initialized = false;

  robot.control([&](const franka::RobotState& robot_state,
                    franka::Duration period) -> franka::JointPositions {
    time += period.toSec();

    if (!initialized) {
      q_start = robot_state.q_d;
      initialized = true;
    }

    const double s_raw = time / duration;
    const double s     = std::min(1.0, s_raw);
    const double blend = 0.5 - 0.5 * std::cos(M_PI * s);

    std::array<double, 7> q_cmd{};
    for (size_t i = 0; i < 7; ++i) {
      q_cmd[i] = q_start[i] + blend * (q_goal[i] - q_start[i]);
    }

    franka::JointPositions output(q_cmd);

    if (s >= 1.0) {
      std::cout << "Motion finished.\n";
      return franka::MotionFinished(output);
    }

    return output;
  });
}

}  // namespace

int main() {
  const std::string robot_ip = "172.16.0.3";

  // -----------------------------------------------------------------------
  // EDIT THESE 7 JOINT TARGETS IN RADIANS.
  // -----------------------------------------------------------------------
  const std::array<double, 7> q_target = {
       -0.3001,
      -0.9919,
       1.6632,
      -2.3879,
       1.2062,
       2.0396,
       1.3540};

  // Franka Emika Panda factory home / ready pose
  const std::array<double, 7> q_home = {
       0.0,
      -0.785398,
       0.0,
      -2.356194,
       0.0,
       1.570796,
       0.785398};
  // NOTE: Adjust q_home above to your own preferred home configuration if needed.

  constexpr double kMotionDuration = 4.0;

  // -----------------------------------------------------------------------
  // Interactive menu
  // -----------------------------------------------------------------------
  std::cout << "========================================\n";
  std::cout << "  Franka Panda Joint Controller\n";
  std::cout << "========================================\n";
  std::cout << "  [1]  Move to target pose\n";
  std::cout << "  [2]  Go to HOME position\n";
  std::cout << "  [q]  Quit\n";
  std::cout << "----------------------------------------\n";
  std::cout << "Select option: ";

  char choice{};
  std::cin >> choice;
  std::cin.ignore();  // consume leftover newline

  if (choice == 'q' || choice == 'Q') {
    std::cout << "Exiting.\n";
    return 0;
  }

  bool goHome = false;
  if (choice == '1') {
    goHome = false;
  } else if (choice == '2') {
    goHome = true;
  } else {
    std::cerr << "Unknown option '" << choice << "'. Exiting.\n";
    return 1;
  }

  const std::array<double, 7>& q_goal = goHome ? q_home : q_target;
  const std::string label = goHome ? "HOME" : "TARGET";

  if (!withinJointLimits(q_goal)) {
    return 1;
  }

  std::cout << "\nThis program will move the Panda to " << label << " pose:\n";
  for (size_t i = 0; i < q_goal.size(); ++i) {
    std::cout << "  q[" << i << "] = " << q_goal[i] << " rad\n";
  }
  std::cout << "\nMake sure the workspace is clear and hold the user stop button.\n";
  std::cout << "Press Enter to start...";
  std::cin.get();

  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    moveToTarget(robot, q_goal, kMotionDuration);
  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << '\n';
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Standard exception: " << e.what() << '\n';
    return 1;
  }

  return 0;
}