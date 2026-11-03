#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  // 调试显示用：平滑帧率、推理耗时
  double fps = 0.0;
  double infer_ms = 0.0;
  auto last_loop = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 推理计时（含预处理 + OpenVINO推理 + 后处理）
    auto infer_start = std::chrono::steady_clock::now();
    auto armors = detector.detect(img);
    auto infer_end = std::chrono::steady_clock::now();
    infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();

    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    cboard.send(command);

    // ================= 调试画面 =================
    // 整体循环帧率（指数平滑）
    auto loop_now = std::chrono::steady_clock::now();
    double loop_dt = std::chrono::duration<double>(loop_now - last_loop).count();
    last_loop = loop_now;
    if (loop_dt > 0.0) {
      double inst_fps = 1.0 / loop_dt;
      fps = (fps == 0.0) ? inst_fps : 0.9 * fps + 0.1 * inst_fps;
    }

    cv::Mat show = img.clone();

    // 绘制每个装甲板的四点框和标签
    for (const auto & armor : armors) {
      std::vector<cv::Point> pts;
      for (const auto & p : armor.points) {
        pts.emplace_back(cv::Point(cvRound(p.x), cvRound(p.y)));
      }
      cv::polylines(show, pts, true, cv::Scalar(0, 255, 0), 2);

      auto label = fmt::format(
        "{} {} {:.2f}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
        armor.confidence);
      tools::draw_text(
        show, label,
        cv::Point(cvRound(armor.points[0].x), cvRound(armor.points[0].y) - 10),
        cv::Scalar(0, 255, 0), 0.6, 2);
    }

    // 左上角：帧率 / 推理耗时 / 装甲板数量
    auto head = fmt::format(
      "FPS: {:.1f} | Infer: {:.1f} ms | Armors: {}", fps, infer_ms, armors.size());
    tools::draw_text(show, head, cv::Point(10, 30), cv::Scalar(0, 255, 255), 0.8, 2);

    cv::imshow("auto_aim", show);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}