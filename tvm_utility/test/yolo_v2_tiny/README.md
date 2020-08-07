# Example Pipeline

This is an example implementation of an inference pipeline using the pipeline
framework. This example pipeline executes the
[YOLO V2 Tiny](https://pjreddie.com/darknet/yolov2/) model and decodes its
output.

## Compiling the Example

1. Download an example image to be used as test input. this image needs to be
   saved in the `test/yolo_v2_tiny/` folder

    ```sh
    $ curl https://raw.githubusercontent.com/pjreddie/darknet/master/data/dog.jpg \
      > test/yolo_v2_tiny/test_image_0.jpg
    ```

2. Compile the neural network model using TVM. A CLI utility program and
   instructions on how to compile can be found in `/modelzoo/scripts/tvm_cli/`
   The files produced by the tvm_cli must then be placed in
   `test/yolo_v2_tiny/`. These files are:

   - inference_engine_tvm_config.hpp
   - deploy_lib.so
   - deploy_graph.json
   - deploy_param.params

3. Start autoware docker environment.

4. Build and test inside the container.

    ```sh
    $ colcon build --packages-up-to tvm_utility
    $ colcon test --packages-select tvm_utility
    ```
