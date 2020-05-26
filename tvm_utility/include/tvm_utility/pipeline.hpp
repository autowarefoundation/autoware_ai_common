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

#include <dlpack/dlpack.h>
#include <string>
#include <tvm/runtime/c_runtime_api.h>
#include <vector>

#pragma once

namespace tvm_utility {
namespace pipeline {

class TVMArrayContainer {
public:
  TVMArrayContainer() = default;

  TVMArrayContainer(std::vector<int64_t> shape, DLDataTypeCode dtype_code,
                    uint32_t dtype_bits, uint32_t dtype_lanes,
                    DLDeviceType device_type, uint32_t device_id) {

    TVMArray *x{};
    TVMArrayAlloc(&shape[0], shape.size(), dtype_code, dtype_bits, dtype_lanes,
                  device_type, device_id, &x);
    handle_ = std::make_shared<TVMArray *>(x);
  }

private:
  std::shared_ptr<TVMArray *> handle_{nullptr, [](TVMArray *ptr) {
                                        if (ptr)
                                          TVMArrayFree(ptr);
                                      }};
};

using TVMArrayContainerVector = std::vector<TVMArrayContainer>;

    /**
     * @class PipelineStage
     * @brief Base class for all types of pipeline stages.
     *
     * @tparam InputType The datatype of the input of the pipeline stage.
     * @tparam OutputType The datatype of the output from the pipeline stage.
     */
    template <class InputType, class OutputType>
    class PipelineStage {
public:
  /**
   * @brief Execute the pipeline stage
   *
   * @param input The data to push into the pipeline stage. The pipeline stage
   * should not modify the input data.
   * @return The output of the pipeline
   */
  virtual OutputType schedule(const InputType &input) = 0;
  InputType input_type_indicator_;
  OutputType output_type_indicator_;
};

/**
 * @class PreProcessor
 * @brief Pre processor of the inference pipeline. In charge of converting data
 * from InputType into TVMArrayContainer format. Any necessary pre processing
 * of the data, such as image resizing or padding, should also be done in this
 * stage .
 *
 * @tparam InputType The data type of the input to the pre-processing pipeline
 * stage. Usually a ROS message type.
 */
template <class InputType>
class PreProcessor : public PipelineStage<InputType, TVMArrayContainerVector> {};

/**
 * @class InferenceEngine
 * @brief Pipeline stage in charge of machine learning inference.
 */
class InferenceEngine
    : public PipelineStage<TVMArrayContainerVector, TVMArrayContainerVector> {};

/**
 * @class PostProcessor
 * @brief The post processing stage of the inference pipeline. In charge of
 * converting the tensor data from the inference stage into detections in
 * OutputType, usually a ROS message format. Thing such as decoding bounding
 * boxes, non-maximum-supperssion and minimum score filtering should be done in
 * this stage.
 *
 * @tparam OutputType The data type of the output of the inference pipeline.
 * Usually a ROS message type.
 */
template <class OutputType>
class PostProcessor : public PipelineStage<TVMArrayContainerVector, OutputType> {
};

/**
 * @class Pipeline
 * @brief Inference Pipeline. Consists of 3 stages: preprocessor, inference
 * stage and postprocessor.
 */
template <class PreProcessorType, class InferenceEngineType,
          class PostProcessorType>
class Pipeline {
  using InputType =
      decltype(std::declval<PreProcessorType>().input_type_indicator_);
  using OutputType =
      decltype(std::declval<PostProcessorType>().output_type_indicator_);

public:
  /**
   * @brief Construct a new Pipeline object
   *
   * @param pre_processor a PreProcessor object
   * @param post_processor a PostProcessor object
   * @param inference_engine a InferenceEngine object
   */
  Pipeline(PreProcessorType pre_processor, InferenceEngineType inference_engine,
           PostProcessorType post_processor)
      : pre_processor_(pre_processor), post_processor_(post_processor),
        inference_engine_(inference_engine) {}

  /**
   * @brief run the pipeline. Return asynchronously in a callback.
   *
   * @param input The data to push into the pipeline
   * @return The pipeline output
   */
  OutputType schedule(const InputType &input) {
    auto input_tensor = pre_processor_.schedule(input);
    auto output_tensor = inference_engine_.schedule(input_tensor);
    return post_processor_.schedule(output_tensor);
  };

private:
  PreProcessorType pre_processor_{};
  InferenceEngineType inference_engine_{};
  PostProcessorType post_processor_{};
};

// each node should be specificed with a string name and a shape
using NetworkNode = std::pair<std::string, std::vector<int64_t>>;
typedef struct {
  // network files
  std::string network_module_path;
  std::string network_graph_path;
  std::string network_params_path;

  // network data type configurations
  DLDataTypeCode tvm_dtype_code;
  uint32_t tvm_dtype_bits;
  uint32_t tvm_dtype_lanes;

  // inference hardware configuration
  DLDeviceType tvm_device_type;
  uint32_t tvm_device_id;

  // network inputs
  std::vector<NetworkNode> network_inputs;

  // network outputs
  std::vector<NetworkNode> network_outputs;
} InferenceEngineTVMConfig;

class InferenceEngineTVM : public InferenceEngine {
public:
  InferenceEngineTVM(InferenceEngineTVMConfig config) : config_(config){};
  TVMArrayContainerVector schedule(const TVMArrayContainerVector &input) {
    // call tvm inference engine with the given input
    // tvm_inf_func(input);

    // get output
    TVMArrayContainer output;
    // tvm_output_func(output);

    return {output};
  };

private:
  InferenceEngineTVMConfig config_;
};

} // namespace pipeline
} // namespace tvm_utility
