/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gtest/gtest.h"

#include "autoware_msgs/DetectedObjectArray.h"
#include "pcl_ros/point_cloud.h"
#include <tvm_vendor/dlpack/dlpackcpp.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <tvm_utility/pipeline.h>

#include "inference_engine_tvm_config.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

// Name of file containing the human readable names of the classes. One class
// on each line.
#define LABEL_FILENAME "labels.txt"
// Name of file containing the anchor values for the network. Each line is one
// anchor. each anchor has 2 comma separated floating point values.
#define ANCHOR_FILENAME "anchors.csv"
// filename of the image on which to run the inference
#define IMAGE_FILENAME "test_image_0.jpg"

class PreProcessorYoloV2Tiny
    : public tvm_utility::pipeline::PreProcessor<sensor_msgs::PointCloud2>
{
public:
  explicit PreProcessorYoloV2Tiny(tvm_utility::pipeline::InferenceEngineTVMConfig config)
      : network_input_width(config.network_inputs[0].second[1]),
        network_input_height(config.network_inputs[0].second[2]),
        network_input_depth(config.network_inputs[0].second[3]),
        network_datatype_bytes(config.tvm_dtype_bits / 8)
  {
    // allocate input variable
    std::vector<int64_t> shape_x
    {
      1,
      network_input_width,
      network_input_height,
      network_input_depth
    };
    tvm_utility::pipeline::TVMArrayContainer x
    {
      shape_x,
      config.tvm_dtype_code,
      config.tvm_dtype_bits,
      config.tvm_dtype_lanes,
      config.tvm_device_type,
      config.tvm_device_id
    };

    output = x;
  }

  tvm_utility::pipeline::TVMArrayContainerVector
  schedule(const sensor_msgs::PointCloud2 &input)
  {
    // read input image
    auto image = cv::imread(IMAGE_FILENAME, CV_LOAD_IMAGE_COLOR);
    if (!image.data)
    {
      throw std::runtime_error("File " IMAGE_FILENAME " not found");
    }

    // Compute the ratio for resizing and size for padding
    double scale_x =
        static_cast<double>(image.size().width) / network_input_width;
    double scale_y =
        static_cast<double>(image.size().height) / network_input_height;
    double scale = std::max(scale_x, scale_y);

    // perform padding
    if (scale != 1)
    {
      cv::resize(image, image, cv::Size(), 1.0f / scale, 1.0f / scale);
    }

    size_t w_pad = network_input_width - image.size().width;
    size_t h_pad = network_input_height - image.size().height;

    if (w_pad || h_pad)
    {
      cv::copyMakeBorder(image, image, h_pad / 2, (h_pad - h_pad / 2),
                         w_pad / 2, (w_pad - w_pad / 2), cv::BORDER_CONSTANT,
                         cv::Scalar(0, 0, 0));
    }

    // convert pixel values from int8 to float32. convert pixel value range from
    // 0 - 255 to 0 - 1.
    cv::Mat3f image_3f{};
    image.convertTo(image_3f, CV_32FC3, 1 / 255.0f);

    // cv library use BGR as a default color format, the network expects the
    // data in RGB format
    cv::cvtColor(image_3f, image_3f, CV_BGR2RGB);

    TVMArrayCopyFromBytes(output.getArray(), image_3f.data,
                          network_input_width * network_input_height *
                              network_input_depth * network_datatype_bytes);

    return {output};
  }

private:
  int64_t network_input_width;
  int64_t network_input_height;
  int64_t network_input_depth;
  int64_t network_datatype_bytes;
  tvm_utility::pipeline::TVMArrayContainer output;
};

class PostProcessorYoloV2Tiny
    : public tvm_utility::pipeline::PostProcessor<std::vector<float>>
{
public:
  PostProcessorYoloV2Tiny(
      tvm_utility::pipeline::InferenceEngineTVMConfig config)
      : network_output_width(config.network_outputs[0].second[1]),
        network_output_height(config.network_outputs[0].second[2]),
        network_output_depth(config.network_outputs[0].second[3])
  {
    // parse human readable names for the classes
    std::ifstream label_file{LABEL_FILENAME};
    if (!label_file.good())
    {
      throw std::runtime_error("unable to open label file:" LABEL_FILENAME);
    }
    std::string line{};
    while (std::getline(label_file, line))
    {
      labels.push_back(line);
    }

    // Get anchor values for this network from the anchor file
    std::ifstream anchor_file{ANCHOR_FILENAME};
    if (!anchor_file.good())
    {
      throw std::runtime_error("unable to open anchor file:" ANCHOR_FILENAME);
    }
    std::string first{};
    std::string second{};
    while (std::getline(anchor_file, line))
    {
      std::stringstream line_stream(line);
      std::getline(line_stream, first, ',');
      std::getline(line_stream, second, ',');
      anchors.push_back(
          std::make_pair(std::atof(first.c_str()), std::atof(second.c_str())));
    }
  }

  // sigmoid function
  float sigmoid(float x) { return static_cast<float>(1.0 / (1.0 + std::exp(-x))); }

  std::vector<float>
  schedule(const tvm_utility::pipeline::TVMArrayContainerVector &input)
  {
    auto l_h = network_output_width;   // layer height
    auto l_w = network_output_height;  // layer width
    auto n_classes = labels.size();    // total number of classes
    auto n_anchors = anchors.size();   // total number of anchors
    const uint32_t n_coords = 4;       // number of coordinates in a single anchor box

    // assert data is stored row-majored in input and the dtype is float
    assert(input[0].getArray()->strides == nullptr);
    assert(input[0].getArray()->dtype.bits == sizeof(float) * 8);

    // get a pointer to the output data
    float *data_ptr =
      reinterpret_cast<float *>(reinterpret_cast<uint8_t *>(input[0].getArray()->data) +
                                input[0].getArray()->byte_offset);

    // utility function to return data from y given index
    auto get_output_data = [this, data_ptr, n_classes, n_anchors,
                            n_coords](size_t row_i, size_t col_j,
                                      size_t anchor_k, size_t offset)
    {
      auto box_index =
          (row_i * network_output_height + col_j) * network_output_depth;
      auto index = box_index + anchor_k * (n_classes + n_coords + 1);
      return data_ptr[index + offset];
    };

    // vector used to check if the result is accurate,
    // this is also the output of this (schedule) function
    std::vector<float> scores_above_threshold{};

    // Parse results into detections. Loop over each detection cell in the model
    // output
    for (size_t i = 0; i < l_w; i++)
    {
      for (size_t j = 0; j < l_h; j++)
      {
        for (size_t anchor_k = 0; anchor_k < n_anchors; anchor_k++)
        {
          float anchor_w = anchors[anchor_k].first;
          float anchor_h = anchors[anchor_k].second;

          // Compute property indices
          auto box_x = get_output_data(i, j, anchor_k, 0);
          auto box_y = get_output_data(i, j, anchor_k, 1);
          auto box_w = get_output_data(i, j, anchor_k, 2);
          auto box_h = get_output_data(i, j, anchor_k, 3);
          auto box_p = get_output_data(i, j, anchor_k, 4);

          // Transform log-space predicted coordinates to absolute space +
          // offset Transform bounding box position from offset to absolute
          // (ratio)
          auto x = (sigmoid(box_x) + j) / l_w;
          auto y = (sigmoid(box_y) + i) / l_h;

          // Transform bounding box height and width from log to absolute space
          auto w = anchor_w * exp(box_w) / l_w;
          auto h = anchor_h * exp(box_h) / l_h;

          // Decode the confidence of detection in this anchor box
          auto p_0 = sigmoid(box_p);

          // find maximum probability of all classes
          float max_p = 0.0f;
          int max_ind = -1;
          for (int i_class = 0; i_class < n_classes; i_class++)
          {
            auto class_p = get_output_data(i, j, anchor_k, 5 + i_class);
            if (max_p < class_p)
            {
              max_p = class_p;
              max_ind = i_class;
            }
          }

          // decode and copy class probabilities
          std::vector<float> class_probabilities{};
          float p_total = 0;
          for (size_t i_class = 0; i_class < n_classes; i_class++)
          {
            auto class_p = get_output_data(i, j, anchor_k, 5 + i_class);
            class_probabilities.push_back(std::exp(class_p - max_p));
            p_total += class_probabilities[i_class];
          }

          // Find the most likely score
          auto max_score = class_probabilities[max_ind] * p_0 / p_total;

          // draw all detections with high scores
          if (max_score > 0.3)
          {
            scores_above_threshold.push_back(max_score);
          }
        }
      }
    }

    return scores_above_threshold;
  }

private:
  int64_t network_output_width;
  int64_t network_output_height;
  int64_t network_output_depth;
  std::vector<std::string> labels{};
  std::vector<std::pair<float, float>> anchors{};
};

// bring config into scope
namespace model_config = model_zoo::perception::camera_obstacle_detection::
    yolo_v2_tiny::tensorflow_fp32_coco;
using tvm_utility::pipeline::InferenceEngineTVM;
using tvm_utility::pipeline::Pipeline;

TEST(PipelineExamples, SimplePipeline)
{
  // instantiate the pipeline
  Pipeline<PreProcessorYoloV2Tiny, InferenceEngineTVM, PostProcessorYoloV2Tiny>
      pipeline(PreProcessorYoloV2Tiny{model_config::config},
               InferenceEngineTVM{model_config::config},
               PostProcessorYoloV2Tiny{model_config::config});  // NOLINT

  // push data input the pipeline and get the output
  sensor_msgs::PointCloud2 msg{};
  auto output = pipeline.schedule(msg);

  // define reference vector containing expected values
  std::vector<float> expected_output
  {
    0.360896,
    0.763201,
    0.402838,
    0.302775,
    0.612508,
    0.583656,
    0.422576,
    0.452684,
    0.372608,
    0.806072,
    0.572778
  };

  // test: check if the generated output is equal to the reference
  EXPECT_EQ(expected_output.size(), output.size()) << "Unexpected output size";
  for (auto i = 0; i < output.size(); ++i)
  {
    EXPECT_NEAR(expected_output[i], output[i], 0.0001) << "at index: " << i;
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
