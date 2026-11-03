#include <fmt/core.h>

#include <chrono>
#include <list>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/standard3.yaml | yaml配置文件路径}"
  "{tradition t    | false                  | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path, false);
  auto_aim::YOLO yolo(config_path, false);

  // 平滑帧率、推理耗时
  double fps = 0.0;
  double infer_ms = 0.0;
  auto last_loop = std::chrono::steady_clock::now();

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) break;

    // 推理计时（预处理 + OpenVINO推理 + 后处理）
    std::list<auto_aim::Armor> armors;
    auto infer_start = std::chrono::steady_clock::now();
    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);
    auto infer_end = std::chrono::steady_clock::now();
    infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();

    // 整体帧率（指数平滑）
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

    cv::imshow("detect_view", show);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
