// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.
#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/rect.pb.h"

#include "mediapipe/framework/calculator_graph.h"
#include "mediapipe/framework/port/logging.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/formats/landmark.pb.h"


constexpr char kInputStream[] = "image";
constexpr char kOutputStream[] = "face_landmarks";
constexpr char kWindowName[] = "MediaPipe";

ABSL_FLAG(std::string, calculator_graph_config_file, "/home/shirlyyang/mediapipe/mediapipe/modules/face_landmark/face_landmark_cpu.pbtxt",
          "Name of file containing text format CalculatorGraphConfig proto.");
// ABSL_FLAG(std::string, input_video_path, "",
ABSL_FLAG(std::string, input_video_path, "/home/shirlyyang/桌面/acc9c292ab4feef21a9d39efa3af58bd.mov",
          "Full path of video to load. "
          "If not provided, attempt to use a webcam.");
ABSL_FLAG(std::string, output_video_path, "",
          "Full path of where to save result (.mp4 only). "
          "If not provided, show result in a window.");
// Returns an example region of interest rectangle.
mediapipe::NormalizedRect GetTestRoi() {
  mediapipe::NormalizedRect result;
  result.set_x_center(0.5);
  result.set_y_center(0.5);
  result.set_width(1);
  result.set_height(1);
  return result;
}
absl::Status RunMPPGraph() {
  LOG(INFO) << "INTO RunMPPGraph";

  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
      absl::GetFlag(FLAGS_calculator_graph_config_file),
      &calculator_graph_config_contents));
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  LOG(INFO) << "Initialize the camera or load the video.";
  cv::VideoCapture capture;
  // const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
  // if (load_video) {
    capture.open(absl::GetFlag(FLAGS_input_video_path));
  // } else {
  //   LOG(INFO) << "use webcam";

  //   capture.open(0);
  // }
  RET_CHECK(capture.isOpened());
  LOG(INFO) << "Video opened.";
  // cv::VideoWriter writer;
//   const bool save_video = !absl::GetFlag(FLAGS_output_video_path).empty();
//   if (!save_video) {
//     cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
// #if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
//     capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
//     capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
//     capture.set(cv::CAP_PROP_FPS, 30);
// #endif
  // }

  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                   graph.AddOutputStreamPoller(kOutputStream));
  // graph.ObserveOutputStream("face_landmarks", [&](const mediapipe::Packet& p) {
  //   // output.push_back(p);
  //   LOG(INFO) << "callback called";
  //   return absl::OkStatus();
  // });
  LOG(INFO) << "Start running the calculator graph.";

  MP_RETURN_IF_ERROR(graph.StartRun({
    // side_packet
    {
      "with_attention", mediapipe::MakePacket<bool>(true)
    }
  }));

  LOG(INFO) << "Put roi into graph";

  // MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
  //     "roi", MakePacket<mediapipe::NormalizedRect>(GetTestRoi())
  //                .At(mediapipe::Timestamp(20000))));

  LOG(INFO) << "Start grabbing and processing frames.";

  bool grab_frames = true;
  // int frame_num = 0;
  while (grab_frames) {
    // frame_num ++;
    
    // Capture opencv camera or video frame.
    cv::Mat camera_frame_raw;
    capture >> camera_frame_raw;
    if (camera_frame_raw.empty()) {
      // if (!load_video) {
      //   LOG(INFO) << "Ignore empty frames from camera.";
      //   continue;
      // }
      LOG(INFO) << "Empty frame, end of video reached.";
      break;
    }
    // if (frame_num < 10) {
    //   LOG(INFO) << "skip frame";
    //   continue;
    // }
    cv::Mat camera_frame;
    cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGB);
    // if (!load_video) {
      // cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
    // }
    LOG(INFO) << "Grabbed frame";
    LOG(INFO) << "video size" << camera_frame.cols << "," << camera_frame.rows;

    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
        mediapipe::ImageFrame::kDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);
    // LOG(INFO) << "copyed frame";

    // Send image packet into the graph.
    size_t frame_timestamp_us =
        (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
    // LOG(INFO) << "frame_timestamp_us" << frame_timestamp_us;
    
    MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
        kInputStream, mediapipe::Adopt(input_frame.release())
                          .At(mediapipe::Timestamp(frame_timestamp_us))));
    MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
    "roi", mediapipe::MakePacket<mediapipe::NormalizedRect>(GetTestRoi())
               .At(mediapipe::Timestamp(frame_timestamp_us))));

    LOG(INFO) << "sent frame";
    

    // Get the graph result packet, or stop if that fails.
    mediapipe::Packet packet;
    // LOG(INFO) << "poller 000";

    if (!poller.Next(&packet)) {
      LOG(INFO) << "poller break|||||";
      break;
    }
    auto& output = packet.Get<mediapipe::NormalizedLandmarkList>();
    for (int i = 0; i < 478; i++) {
      mediapipe::NormalizedLandmark landmark = output.landmark(i);
      LOG(INFO) << "landmark x " << i << ":" << landmark.x();
    }
    LOG(INFO) << "poller next||||||";
   
  }

  LOG(INFO) << "Shutting down.";
  // if (writer.isOpened()) writer.release();
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  return graph.WaitUntilDone();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  LOG(ERROR) << "Start!";
  absl::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}
