load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "obstacle_detection",
    srcs = ["obstacle_detection.cc"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/lib/obstacle/detector/yolo:yolo_obstacle_detector",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/common/io:io_util",
        "@boost",
        "@gtest",
        "@opencv",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
