
## 启动facemesh_cpu的命令
GLOG_logtostderr=1 GLOG_v=2 bazel run --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/face_mesh:face_mesh_cpu --sandbox_debug 

wasm:
GLOG_logtostderr=1 bazel build --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/face_mesh:wasm

## 启动hello_world的命令
GLOG_logtostderr=1 bazel run --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/hello_world:hello_world --sandbox_debug

wasm:
GLOG_logtostderr=1 bazel build --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/hello_world:hello_world_wasm