## Run with Bazel

```shell
bazel run -c opt spawn_model
```

### On MacOS:
```shell
bazel run --macos_minimum_os=15.5 --features=-layering_check -c opt spawn_model
```
