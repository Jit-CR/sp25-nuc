#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/standard3.yaml | yaml配置文件路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;
  io::Camera camera(config_path);
  auto_aim::multithread::MultiThreadDetector detector(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  // 预先把推理流水线填满（后台并行推理）
  for (int i = 0; i < 4; ++i) {
    camera.read(img, t);
    detector.push(img, t);
  }

  double fps = 0.0;
  auto last_loop = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    // 采新帧并送入异步推理（不等待）
    camera.read(img, t);
    detector.push(img, t);

    // 取最早一帧（此时后台通常已推理完成，几乎不等待）
    auto [show_img, armors, frame_t] = detector.debug_pop();

    // 帧率统计
    auto loop_now = std::chrono::steady_clock::now();
    double loop_dt = std::chrono::duration<double>(loop_now - last_loop).count();
    last_loop = loop_now;
    if (loop_dt > 0.0) {
      double inst_fps = 1.0 / loop_dt;
      fps = (fps == 0.0) ? inst_fps : 0.9 * fps + 0.1 * inst_fps;
    }

    cv::Mat show = show_img.clone();

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

    auto head = fmt::format("FPS: {:.1f} (async) | Armors: {}", fps, armors.size());
    tools::draw_text(show, head, cv::Point(10, 30), cv::Scalar(0, 255, 255), 0.8, 2);

    cv::Mat show_small;
    cv::resize(show, show_small, {}, 0.5, 0.5);
    cv::imshow("async_detect", show_small);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
