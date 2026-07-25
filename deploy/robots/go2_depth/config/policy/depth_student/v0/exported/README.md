# Depth Student ONNX

Place the exported depth-student ONNX policy here:

```text
policy.onnx
```

Expected interface:

```text
inputs:
  obs        float32[1, 753]
  depth      float32[1, 58, 87]
  hidden_in  float32[1, 1, 512]

outputs:
  actions    float32[1, 12]
  hidden_out float32[1, 1, 512]
```
