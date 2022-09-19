#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
// #include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
// #include "mediapipe/framework/port/opencv_highgui_inc.h"
// #include "mediapipe/framework/port/opencv_imgproc_inc.h"
// #include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/rect.pb.h"

#include "mediapipe/framework/calculator_graph.h"
#include "mediapipe/framework/port/logging.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include <emscripten/bind.h>

constexpr char kInputStream[] = "image";
constexpr char kOutputStream[] = "face_landmarks";

class Landmark {
  public:
  float x, y, z;
  Landmark(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  Landmark() {}
};

class MppGraphManager {
  public:
    mediapipe::CalculatorGraph graph;
    // mediapipe::OutputStreamPoller poller;
    absl::Status Initialize();
    absl::Status SetSize(int pwidth, int pheight) {
      width = pwidth;
      height = pheight;
    };
    Landmark * list;
    absl::Status Send(uint8_t* buffer, size_t nSize);
    Landmark * GetLandmarkList();
    int runCounter = 0;
    int width = 540;
    int height = 960;
    MppGraphManager() {
    };
    MppGraphManager(int pwidth, int pheight) {
      width = pwidth;
      height = pheight;
      
    };
};

mediapipe::NormalizedRect GetTestRoi() {
  mediapipe::NormalizedRect result;
  result.set_x_center(0.5);
  result.set_y_center(0.5);
  result.set_width(1);
  result.set_height(1);
  return result;
};
ABSL_FLAG(std::string, calculator_graph_config_file, "/Users/shirlyyang/workspace/mediapipe/mediapipe/modules/face_landmark/face_landmark_cpu.pbtxt",
          "Name of file containing text format CalculatorGraphConfig proto.");
// 初始化graph
absl::Status MppGraphManager::Initialize() {
  LOG(INFO) << "Initialize Start";
  // 从pbtxt读取文件
  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
      absl::GetFlag(FLAGS_calculator_graph_config_file),
      &calculator_graph_config_contents));
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  // 解析config
  mediapipe::CalculatorGraphConfig config =
     mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
         calculator_graph_config_contents);
  LOG(INFO) << "Initialize the calculator graph.";
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  // ASSIGN_OR_RETURN(poller, graph.AddOutputStreamPoller(kOutputStream));
  
  graph.ObserveOutputStream(kOutputStream, [&](const mediapipe::Packet& packet) {
    // 获取输出
    list = new Landmark[478];

    auto& faces = packet.Get<mediapipe::NormalizedLandmarkList>();
    for (int i = 0; i < 478; i++) {
      mediapipe::NormalizedLandmark landmark = faces.landmark(i);
      list[i].x = landmark.x();
      list[i].y = landmark.y();
      list[i].z = landmark.z();
      LOG(INFO) << "landmark x" << i << ":" << list[i].z;
    }
    
    return absl::OkStatus();

  });
  LOG(INFO) << "Start running the calculator graph.";

  MP_RETURN_IF_ERROR(graph.StartRun({
    {
      "with_attention", mediapipe::MakePacket<bool>(true)
    }
  }));
  return graph.WaitUntilDone();
};
// 返回LandMark数组
absl::Status MppGraphManager::Send(uint8_t* buffer, size_t nSize) {
  uint8_t* data;
  uint8_t* imgPtr = buffer;
  for (int ptr = 0; ptr < nSize; ptr += 4) {
    // rgba
    data[ptr] = *imgPtr;
    imgPtr ++;
    data[ptr + 1] = *imgPtr;
    imgPtr ++;
    data[ptr + 2] = *imgPtr; //(255*w) / 500;
    imgPtr ++;
    data[ptr + 3] = *imgPtr;
    imgPtr ++;
  }
  auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGBA, width, height,
        mediapipe::ImageFrame::kDefaultAlignmentBoundary);
  int img_data_size = 640 * 480 * 4;
  std::memcpy(input_frame->MutablePixelData(), data, img_data_size);

  size_t frame_timestamp_us =
    runCounter * 1e6;
  runCounter ++;
  graph.AddPacketToInputStream(
      kInputStream, mediapipe::Adopt(input_frame.release())
                        .At(mediapipe::Timestamp(frame_timestamp_us)));
  graph.AddPacketToInputStream(
    "roi", MakePacket<mediapipe::NormalizedRect>(GetTestRoi())
               .At(mediapipe::Timestamp(frame_timestamp_us)));
  LOG(INFO) << "sent frame";
  LOG(INFO) << "Shutting down.";
  graph.CloseInputStream(kInputStream);
  return absl::OkStatus();
  // // 获取输出
  // mediapipe::Packet packet;
  // Landmark list[468];

  // // // LOG(INFO) << "poller 000";
  // // if (!poller.Next(&packet)) {
  // //   LOG(INFO) << "poller break|||||";
  // //   return list;
  // // }
  // // LOG(INFO) << "poller get result";

  // auto& faces = packet.Get<mediapipe::NormalizedLandmarkList>();
  // for (int i = 0; i < 468; i++) {
  //   mediapipe::NormalizedLandmark landmark = faces.landmark(i);
  //   list[i].x = landmark.x();
  //   list[i].y = landmark.y();
  //   list[i].z = landmark.z();
  //   LOG(INFO) << "landmark x" << list[i].z;
  // }

  // 返回结果
  // return list;
};
Landmark* MppGraphManager::GetLandmarkList() {
  return list;
}


EMSCRIPTEN_BINDINGS(face_mesh) {
  emscripten::class_<MppGraphManager>("MppGraphManager")
    .constructor()
    .constructor<int, int>()
    .function("SetSize", &MppGraphManager::SetSize)
    .function("Initialize", &MppGraphManager::Initialize)
    .function("Send", &MppGraphManager::Send, emscripten::allow_raw_pointers())
    .function("GetLandmarkList", &MppGraphManager::GetLandmarkList, emscripten::allow_raw_pointers());

  emscripten::class_<Landmark>("Landmark")
    .property("x", &Landmark::x)
    .property("y", &Landmark::y)
    .property("z", &Landmark::z);
  
  
  emscripten::register_vector<Landmark>("VectorLandmark>");
}
// int main(){}