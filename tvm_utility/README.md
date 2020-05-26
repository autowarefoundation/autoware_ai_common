# TVM Utility Library

A set of c++ utilities to help build a TVM based machine learning inference
pipeline. The library contains a pipeline class which helps building the
pipeline and a number of utility functions that are common in machine learning.

## The Pipeline Class

The Pipeline Class is a standardized way to write a inference pipeline. The
pipeline class contains 3 different stages: the pre-processor, the inference
engine and the post processor. The TVM implementation of a inference engine
stage is provided. The pre and post processor need to be implemented by the user
before instantiating the pipeline. You can see example usage in
[example_pipeline](test/example_pipeline_test).

Each stage in the pipeline has a `schedule` function which takes input data as a
parameter and return the output data. Once the pipeline object is created,
`pipeline.schedule` is called to run the pipeline.

```cpp
int main() {
  nh_.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, [pipeline, pub]() {
    pub.publish(pipeline.schedule(msg));
  });
}
```

## The Utility Functions

A set of utility functions common in machine learning that can be used in
building the pipeline.

## Error

`std::runtime_error` should be thrown whenever error is encountered. It should
be populated with an appropriate text error description.
