// #include <cstdlib>

// #include "absl/flags/flag.h"
// #include "absl/flags/parse.h"
// #include "mediapipe/framework/calculator_framework.h"
// #include "mediapipe/framework/formats/image_frame.h"
// #include "mediapipe/framework/formats/image_frame_opencv.h"
// #include "mediapipe/framework/port/file_helpers.h"
// #include "mediapipe/framework/port/opencv_highgui_inc.h"
// #include "mediapipe/framework/port/opencv_imgproc_inc.h"
// #include "mediapipe/framework/port/opencv_video_inc.h"
// #include "mediapipe/framework/port/parse_text_proto.h"
// #include "mediapipe/framework/port/status.h"
// #include "mediapipe/framework/calculator_framework.h"
// #include "mediapipe/framework/formats/rect.pb.h"

// #include "mediapipe/framework/calculator_graph.h"
// #include "mediapipe/framework/port/logging.h"
// #include "mediapipe/framework/port/status.h"
// #include "mediapipe/framework/formats/landmark.pb.h"
// #include <emscripten/bind.h>

// constexpr char kInputStream[] = "image";
// constexpr char kOutputStream[] = "face_landmarks";

// class Landmark {
//   public:
//   float x, y, z;
//   Landmark(float x, float y, float z) {
//     this->x = x;
//     this->y = y;
//     this->z = z;
//   }
//   Landmark() {}
// };

// class MppGraphManager {
//   public:
//     // MppGraphManager() {};
//     mediapipe::CalculatorGraph graph;
//     mediapipe::OutputStreamPoller poller;
//     absl::Status Initialize();
//     Landmark * Send(uint8_t* buffer, size_t nSize);
// };

// mediapipe::NormalizedRect GetTestRoi() {
//   mediapipe::NormalizedRect result;
//   result.set_x_center(0.5);
//   result.set_y_center(0.5);
//   result.set_width(1);
//   result.set_height(1);
//   return result;
// };
// ABSL_FLAG(std::string, calculator_graph_config_file, "/Users/shirlyyang/workspace/mediapipe/mediapipe/modules/face_landmark/face_landmark_cpu.pbtxt",
//           "Name of file containing text format CalculatorGraphConfig proto.");
// // 初始化graph
// absl::Status MppGraphManager::Initialize() {
//   LOG(INFO) << "Initialize Start";
//   // 从pbtxt读取文件
//   std::string calculator_graph_config_contents;
//   MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
//       absl::GetFlag(FLAGS_calculator_graph_config_file),
//       &calculator_graph_config_contents));
//   LOG(INFO) << "Get calculator graph config contents: "
//             << calculator_graph_config_contents;
//   // 解析config
//   mediapipe::CalculatorGraphConfig config =
//      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
//          calculator_graph_config_contents);
//   LOG(INFO) << "Initialize the calculator graph.";
//   MP_RETURN_IF_ERROR(graph.Initialize(config));
//   ASSIGN_OR_RETURN(poller, graph.AddOutputStreamPoller(kOutputStream));
//   LOG(INFO) << "Start running the calculator graph.";

//   MP_RETURN_IF_ERROR(graph.StartRun({}));
//   return graph.WaitUntilDone();
// };
// // 返回LandMark数组
// Landmark * MppGraphManager::Send(uint8_t* buffer, size_t nSize) {
//   cv::Mat raw_data = cv::Mat(1, nSize, CV_8UC1, buffer);
//   cv::Mat camera_frame_raw = cv::imdecode(raw_data, cv::IMREAD_GRAYSCALE);
//   cv::Mat camera_frame;
//   // opencv使用的是bgr空间
//   cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGB);
//   auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
//         mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
//         mediapipe::ImageFrame::kDefaultAlignmentBoundary);
//   cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
//   // 画面输入到packet中
//   camera_frame.copyTo(input_frame_mat);

//   size_t frame_timestamp_us =
//         (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
//   graph.AddPacketToInputStream(
//       kInputStream, mediapipe::Adopt(input_frame.release())
//                         .At(mediapipe::Timestamp(frame_timestamp_us)));
//   graph.AddPacketToInputStream(
//     "roi", MakePacket<mediapipe::NormalizedRect>(GetTestRoi())
//                .At(mediapipe::Timestamp(frame_timestamp_us)));
//   LOG(INFO) << "sent frame";
//   LOG(INFO) << "Shutting down.";
//   graph.CloseInputStream(kInputStream);
//   // 获取输出
//   mediapipe::Packet packet;
//   Landmark list[468];

//   // LOG(INFO) << "poller 000";
//   if (!poller.Next(&packet)) {
//     LOG(INFO) << "poller break|||||";
//     return list;
//   }
//   LOG(INFO) << "poller get result";

//   auto& faces = packet.Get<mediapipe::NormalizedLandmarkList>();
//   for (int i = 0; i < 468; i++) {
//     mediapipe::NormalizedLandmark landmark = faces.landmark(i);
//     list[i].x = landmark.x();
//     list[i].y = landmark.y();
//     list[i].z = landmark.z();
//     LOG(INFO) << "landmark x" << list[i].z;
//   }

//   // 返回结果
//   return list;
// };


// EMSCRIPTEN_BINDINGS(face_mesh) {
//   emscripten::class_<Landmark>("Landmark")
//     .property("x", &Landmark::x)
//     .property("y", &Landmark::y)
//     .property("z", &Landmark::z);
//   // emscripten::class_<MppGraphManager>("MppGraphManager")
//   //   .constructor()
//   //   .function("Initialize", &MppGraphManager::Initialize)
//   //   .function("Send", &MppGraphManager::Send, emscripten::allow_raw_pointers());

  
//   emscripten::register_vector<Landmark>("VectorLandmark>");
// }
int main() {}
