# Build Systems and Tooling

## Bazel Build System

### Project Structure

```
project/
├── MODULE.bazel          # bzlmod dependency management (modern)
├── WORKSPACE             # Legacy dependency management
├── BUILD.bazel           # Root BUILD file
├── .bazelrc              # Build configuration and flags
├── .bazeliskrc           # Bazelisk version pinning
├── build_rules/          # Custom macros and rules (.bzl files)
│   ├── proto/
│   │   └── build_defs.bzl
│   ├── cc_image/
│   │   └── build_defs.bzl
│   └── ...
├── third_party/          # Third-party BUILD files for external deps
│   ├── absl/
│   │   └── status/BUILD  # Custom test matchers
│   └── protobuf_matchers/
│       └── BUILD
├── platforms/            # Custom platform definitions
│   └── BUILD
├── applications/         # Application source code
│   ├── lib/              # Shared libraries
│   │   ├── grpc/BUILD
│   │   └── zmq/BUILD
│   └── service/            # service-specific code
│       ├── BUILD
│       └── services/
└── tools/                # Developer tooling
```

### MODULE.bazel (bzlmod - Modern Dependency Management)

```python
module(
    name = "my_project",
)

# -- Dependency declarations -- #
bazel_dep(name = "abseil-cpp", version = "20240116.1")
bazel_dep(name = "googletest", version = "1.14.0.bcr.1", repo_name = "gtests")
bazel_dep(name = "google_benchmark", version = "1.9.0")
bazel_dep(name = "protobuf", version = "0", repo_name = "com_google_protobuf")
bazel_dep(name = "rules_cc", version = "0.2.14")
bazel_dep(name = "rules_proto", version = "6.0.0")
bazel_dep(name = "rules_foreign_cc", version = "0.11.1")
bazel_dep(name = "rules_pkg", version = "1.0.1")
bazel_dep(name = "rules_oci", version = "2.0.1")
bazel_dep(name = "bazel_skylib", version = "1.8.2")
bazel_dep(name = "platforms", version = "1.0.0")
bazel_dep(name = "buildifier_prebuilt", version = "6.4.0")
bazel_dep(name = "re2", version = "2023-09-01")

# Override with patched version
archive_override(
    module_name = "protobuf",
    integrity = "sha256-...",
    patch_strip = 1,
    patches = [
        "//:protobuf_add_module.patch",
        "//:protobuf_fix_rules_pkg_providers.patch",
    ],
    strip_prefix = "protobuf-3.20.1",
    urls = ["https://github.com/protocolbuffers/protobuf/archive/v3.20.1.tar.gz"],
)

# LLVM toolchain setup
bazel_dep(name = "toolchains_llvm", version = "1.4.0")

llvm = use_extension("@toolchains_llvm//toolchain/extensions:llvm.bzl", "llvm")
llvm.toolchain(
    llvm_version = "19.1.3",
)
use_repo(llvm, "llvm_toolchain")
use_repo(llvm, "llvm_toolchain_llvm")

# External archive dependencies
http_archive = use_repo_rule("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "openssl_src",
    build_file = "@//:bazel_deps/third_party/BUILD.openssl.bazel",
    integrity = "sha256-...",
    strip_prefix = "openssl-3.4.0",
    urls = ["https://github.com/openssl/openssl/releases/download/openssl-3.4.0/openssl-3.4.0.tar.gz"],
)
```

### WORKSPACE (Legacy Dependency Management)

```python
workspace(name = "my_project")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# GCC Toolchain
http_archive(
    name = "gcc_toolchain",
    patch_args = ["-p1"],
    patches = ["//:gcc_toolchain_skip_aspect_bazel_lib_dep.patch"],
    sha256 = "...",
    strip_prefix = "gcc-toolchain-0.6.0",
    urls = ["https://github.com/f0rmiga/gcc-toolchain/archive/refs/tags/0.6.0.tar.gz"],
)

load("@gcc_toolchain//toolchain:repositories.bzl", "gcc_toolchain_dependencies")
gcc_toolchain_dependencies()

load("@gcc_toolchain//toolchain:defs.bzl", "ARCHS", "gcc_register_toolchain")

# Sysroot for hermetic builds
http_archive(
    name = "sysroot_x86_64",
    build_file = "//third_party:sysroot.BUILD.bazel",
    sha256 = "...",
    url = "https://storage.googleapis.com/bazel-deps/sysroots/sysroot-base-x86_64.tar.xz",
)

load("//third_party/sysroot:flags.bzl", "includes")

gcc_register_toolchain(
    name = "gcc_toolchain_x86_64",
    gcc_version = "10.3.0",
    includes = includes("x86_64", "10.3.0"),
    sysroot = "@sysroot_x86_64//:sysroot",
    target_arch = ARCHS.x86_64,
)

# Hedron's Compile Commands Extractor for IDE support
http_archive(
    name = "hedron_compile_commands",
    sha256 = "...",
    strip_prefix = "bazel-compile-commands-extractor-...",
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/....tar.gz",
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")
hedron_compile_commands_setup()
```

### BUILD Files - cc_library

```python
load("@rules_cc//cc:defs.bzl", "cc_library")

# Header-only library
cc_library(
    name = "operation_result",
    hdrs = ["operation_result.h"],
    visibility = ["//myproject/services/grpc_bridge:__subpackages__"],
    deps = [
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/time",
    ],
)

# Standard library with srcs and hdrs
cc_library(
    name = "system_log_consumer",
    srcs = ["system_log_consumer.cpp"],
    hdrs = ["system_log_consumer.h"],
    visibility = [
        "//myproject/services/firmware_log_handler:__pkg__",
    ],
    deps = [
        "//myproject/firmware_log_handler:log_consumer",
        "@abseil-cpp//absl/log:log",
        "@abseil-cpp//absl/types:span",
    ],
)

# Library with custom include path
cc_library(
    name = "lib",
    srcs = [
        "src/player.cpp",
        "src/player/async.cpp",
        "src/player/window.cpp",
    ],
    hdrs = [
        "include/qdisplay/player.h",
        "include/qdisplay/player/async.h",
        "include/qdisplay/player/display.h",
    ],
    strip_include_prefix = "include",
    deps = [
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/types:span",
    ],
)

# Mock library for testing (testonly = True)
cc_library(
    name = "mock_metric",
    testonly = True,
    hdrs = ["tests/mock_metric.h"],
    deps = [
        "@abseil-cpp//absl/time",
        "@gtests//:gtest",
    ],
)

# Test utility library with broader visibility
cc_library(
    name = "client_lib_mock_cpp",
    testonly = True,
    srcs = ["test_context.cpp"],
    hdrs = [
        "mock_context.h",
        "mock_publisher.h",
        "mock_subscriber.h",
        "test_context.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":client_lib_interface_cpp",
        "@abseil-cpp//absl/hash",
        "@abseil-cpp//absl/log",
        "@gtests//:gtest",
    ],
)

# Third-party test helper with strip_include_prefix
cc_library(
    name = "status_matchers",
    testonly = True,
    srcs = [
        "internal/status_matchers.cc",
        "internal/status_matchers.h",
    ],
    hdrs = ["status_matchers.h"],
    strip_include_prefix = "/third_party",
    visibility = ["//visibility:public"],
    deps = [
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@gtests//:gtest",
    ],
)
```

### BUILD Files - cc_binary

```python
load("@rules_cc//cc:defs.bzl", "cc_binary")

# Standard service binary with absl flags/logging
cc_binary(
    name = "grpc_bridge_service",
    srcs = ["grpc_bridge_server.cpp"],
    deps = [
        "//myproject/grpc_bridge:manager",
        "//myproject/grpc_bridge/internal:server",
        "@abseil-cpp//absl/flags:parse",
        "@abseil-cpp//absl/log:flags",
        "@abseil-cpp//absl/log:initialize",
        "@abseil-cpp//absl/status:status",
    ],
)

# Binary with local library dependency
cc_binary(
    name = "player",
    srcs = ["src/main.cpp"],
    deps = [
        ":lib",
        "@abseil-cpp//absl/flags:parse",
        "@abseil-cpp//absl/log:flags",
        "@abseil-cpp//absl/log:initialize",
    ],
)
```

### BUILD Files - cc_test (GoogleTest)

```python
load("@rules_cc//cc:defs.bzl", "cc_test")

# Small unit test (fastest, most common)
cc_test(
    name = "socket_tests",
    size = "small",
    srcs = ["tests/socket_test.cpp"],
    deps = [
        ":zmqcpp_exception_free",
        "//third_party/absl/status:status_matchers",
        "@gtests//:gtest_main",
    ],
)

# Medium test (for integration-level tests)
cc_test(
    name = "server_cpp_test",
    size = "medium",
    srcs = ["server_test.cpp"],
    deps = [
        ":server_cpp",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/synchronization",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_grpc_grpc//:grpc++_test",
        "@gtests//:gtest_main",
    ],
)

# Test with data dependencies and args
cc_test(
    name = "test",
    srcs = [
        "tests/async.cpp",
        "tests/cli.cpp",
        "tests/flags.cpp",
        "tests/flags.h",
        "tests/player.cpp",
    ],
    args = [
        "--cli_test_executable=$(location :player)",
    ],
    data = [":player"],
    includes = ["tests"],
    deps = [
        ":lib",
        "@abseil-cpp//absl/flags:flag",
        "@abseil-cpp//absl/flags:parse",
        "@gtests//:gtest_main",
    ],
)

# Test with protobuf matchers
cc_test(
    name = "utils_cpp_test",
    size = "small",
    srcs = ["tests/utils_test.cpp"],
    deps = [
        ":client_lib_mock_cpp",
        ":utils_cpp",
        "//myproject/app_data_hub/internal:broker_commands_cc_proto",
        "//third_party/absl/status:status_matchers",
        "//third_party/protobuf_matchers",
        "@gtests//:gtest_main",
    ],
)
```

### Proto Compilation

```python
load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")
load("@io_bazel_rules_go//proto:def.bzl", "go_proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("//build_rules/proto:build_defs.bzl", "proto_lint_test")

# Step 1: Define proto_library (language-agnostic base)
proto_library(
    name = "broker_commands_proto",
    srcs = ["broker_commands.proto"],
    visibility = ["//visibility:private"],
    deps = [":common_proto"],
)

# Step 2: Generate C++ bindings
cc_proto_library(
    name = "broker_commands_cc_proto",
    deps = [":broker_commands_proto"],
)

# Step 3: Generate Go bindings
go_proto_library(
    name = "broker_commands_go_proto",
    importpath = "github.com/myorg/myproject/internal/broker_commands_proto",
    protos = [":broker_commands_proto"],
    deps = [":common_go_proto"],
)

# Step 4: Generate Python bindings
py_proto_library(
    name = "broker_commands_py_proto",
    srcs = ["broker_commands.proto"],
    deps = [":common_py_proto"],
)

# Step 5: Lint proto files
proto_lint_test(
    name = "broker_commands_proto_lint",
    protos = [":broker_commands_proto"],
)
```

### Custom Bazel Macros (.bzl)

```python
# build_rules/proto/build_defs.bzl
"""Custom proto lint test macro with project defaults."""

load("@rules_proto_grpc//buf:defs.bzl", "buf_proto_lint_test")

def proto_lint_test(
        name,
        protos,
        enum_zero_value_suffix = "_UNKNOWN",
        except_rules = None):
    """Provides default values to a buf protolint test target.

    Args:
        name (str): Name of target
        protos (list): proto libraries to include in test
        enum_zero_value_suffix (string): Optional enum zero value suffix
        except_rules (list): Optional list of rules to ignore
    """
    except_rules = except_rules or []
    buf_proto_lint_test(
        name = name,
        enum_zero_value_suffix = enum_zero_value_suffix,
        except_rules = [
            "PACKAGE_VERSION_SUFFIX",
            "PACKAGE_DIRECTORY_MATCH",
        ] + except_rules,
        protos = protos,
        use_rules = [
            "DEFAULT",
            "COMMENTS",
        ],
    )
```

```python
# build_rules/cc_image/build_defs.bzl
"""Quality of life macro for building cc container images."""

load("@io_bazel_rules_docker//cc:image.bzl", io_bazel_cc_image = "cc_image")
load("//build_rules/base_image:build_defs.bzl", "base_image")

def cc_image(
        name,
        base = None,
        deps = None,
        layers = None,
        binary = None,
        **kwargs):
    """Adds standard tooling to cc image builds.

    Args:
        name (string): The name of the resulting image target
        base (Label): The base image to use for builds
        deps (list): List of labels referring to library targets
        layers (list): List of labels referring to container_layer targets
        binary (Label): cc_binary target to run
        **kwargs (dict): Passthrough values to cc_image
    """
    layers = layers or []
    deps = deps or []
    if base == None:
        base = select({
            "//:is_aarch64": "@noble_aarch64//image",
            "//conditions:default": "@distroless-cc//image",
        })
    user_image_name = name + "_base_image"
    base_image(name = user_image_name, base = base, layers = layers)

    io_bazel_cc_image(
        name = name,
        base = user_image_name,
        binary = binary,
        deps = deps,
        **kwargs
    )
```

### .bazelrc Configuration

```bash
# Common settings (apply to all commands)
common \
  --action_env=BAZEL_CXXOPTS="-std=c++17" \
  --action_env=BAZEL_DO_NOT_DETECT_CPP_TOOLCHAIN=1 \
  --color=yes \
  --cxxopt=-std=c++17 \
  --host_cxxopt=-std=c++17 \
  --enable_bzlmod \
  --enable_platform_specific_config \
  --incompatible_enable_cc_toolchain_resolution

# Build settings
build \
  --reuse_sandbox_directories \
  --incompatible_strict_action_env

# Test settings
test \
  --incompatible_exclusive_test_sandboxed \
  --sandbox_writable_path=/opt/myproject \
  --test_output=errors

# Integration test config
test:integration \
  --test_tag_filters=integration

# Build profiling
build:profile \
  --experimental_collect_system_network_usage \
  --generate_json_trace_profile \
  --profile=bazel.profile.gz \
  --experimental_profile_include_target_label

# CI configuration with remote cache
build:CI \
  --google_default_credentials \
  --test_output=errors \
  --show_progress_rate_limit=60 \
  --remote_cache=https://storage.googleapis.com/bazel-ci-cache \
  --keep_going \
  --show_timestamps \
  --remote_upload_local_results \
  --experimental_remote_cache_compression \
  --remote_download_minimal \
  --sandbox_base=/dev/shm \
  --test_summary=terse \
  --test_env=RUNNING_IN_CI=true

# Cross-compilation for aarch64
build:aarch64 \
  --cpu=aarch64 \
  --action_env=BAZEL_DO_NOT_DETECT_CPP_TOOLCHAIN= \
  --linkopt=-mno-fix-cortex-a53-843419

build:aarch64_musl \
  --config=aarch64 \
  --platforms=//platforms:aarch64-unknown-linux-musl

# Optimization
build:optimize \
  --compilation_mode=opt

# Debug with symbols
common:opt_with_symbols \
  --config=optimize \
  --cxxopt="-g" \
  --cxxopt="-fno-omit-frame-pointer"

# Developer-friendly resource limits
build:nice-dev \
  --local_resources=memory=HOST_RAM*.75 \
  --local_resources=cpu=HOST_CPUS*.75 \
  --jobs=HOST_CPUS*.75 \
  --disk_cache=~/.cache/bazel_disk_cache \
  --experimental_disk_cache_gc_max_size=250G \
  --experimental_disk_cache_gc_max_age=30d \
  --watchfs

# Verbose failure debugging
build:verbose_failures \
  --verbose_failures \
  --subcommands=pretty_print

build:very_verbose_failures \
  --verbose_failures \
  --test_output=all \
  --sandbox_debug

# Coverage
coverage \
  --combined_report=lcov

# User-local overrides
try-import %workspace%/.bazelrc.user
```

### Platform and Toolchain Configuration

```python
# platforms/BUILD
constraint_setting(
    name = "linker",
    default_constraint_value = ":unknown",
    visibility = ["//visibility:public"],
)

constraint_value(
    name = "musl",
    constraint_setting = ":linker",
    visibility = ["//visibility:public"],
)

constraint_value(
    name = "unknown",
    constraint_setting = ":linker",
    visibility = ["//visibility:public"],
)

platform(
    name = "aarch64-unknown-linux-musl",
    constraint_values = [
        ":musl",
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
)
```

```python
# Root BUILD.bazel - config_setting and select()
config_setting(
    name = "is_aarch64",
    values = {"cpu": "aarch64"},
)

platform(
    name = "linux_amd64",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    visibility = ["//visibility:public"],
)

platform(
    name = "linux_arm64",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:aarch64",
        "@//platforms:musl",
    ],
    visibility = ["//visibility:public"],
)

# Using select() in targets
cc_library(
    name = "platform_specific_lib",
    deps = select({
        "//:is_aarch64": ["@noble_aarch64//image"],
        "//conditions:default": ["@distroless-cc//image"],
    }),
)
```

### Package-Level Defaults

```python
# Use package() to set default visibility for all targets in a BUILD file
package(
    default_visibility = [
        "//myproject/display:__subpackages__",
        "//myproject/services/display:__subpackages__",
    ],
)

# Or use package-level defaults in internal BUILD files
package(default_visibility = ["//myproject/app_data_hub:__subpackages__"])
```

### Visibility Patterns

```python
# Public visibility (use sparingly, for shared utility libraries)
visibility = ["//visibility:public"]

# Restrict to specific package only
visibility = ["//myproject/services/firmware_log_handler:__pkg__"]

# Restrict to package and all subpackages
visibility = ["//myproject/grpc_bridge:__subpackages__"]

# Multiple specific packages
visibility = [
    "//system_control/process_manager:__subpackages__",
    "//myproject/adh_bridge:__subpackages__",
    "//myproject/services/adh_bridge:__pkg__",
]
```

### IDE Support (compile_commands.json)

```python
# Root BUILD.bazel or package-level BUILD
load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

# Generate compile_commands.json for all targets (excludes external code)
refresh_compile_commands(
    name = "refresh_compile_commands",
    exclude_external_sources = True,
    targets = {
        "//...:all": "--keep_going",
    },
)

# Per-package compile commands (faster, scoped)
refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = {
        ":all": "",
    },
)
```

```bash
# Generate compile_commands.json
bazel run //:refresh_compile_commands

# Per-package (faster)
bazel run //myproject/service:refresh_compile_commands
```

### Container Image from C++ Binary

```python
load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//build_rules/cc_image:build_defs.bzl", "cc_image")
load("//build_rules/base_image:build_defs.bzl", "base_container_test")

# Build binary
cc_binary(
    name = "grpc_bridge_service",
    srcs = ["grpc_bridge_server.cpp"],
    deps = [...],
)

# Package into container image
cc_image(
    name = "grpc_bridge_image",
    binary = ":grpc_bridge_service",
)

# Validate container structure
base_container_test(
    name = "grpc_bridge_structure_test",
    configs = ["structure_tests/grpc_bridge_service.yaml"],
    image = ":grpc_bridge_image",
    target_compatible_with = ["@platforms//cpu:x86_64"],
)
```

### Buildifier (BUILD File Formatting)

```python
# Root BUILD.bazel
load("@buildifier_prebuilt//:rules.bzl", "buildifier", "buildifier_test")

buildifier_exclude_patterns = [
    "./output/external/**",
]

# Check BUILD file formatting (CI)
buildifier_test(
    name = "buildifier",
    exclude_patterns = buildifier_exclude_patterns,
    lint_mode = "warn",
    mode = "check",
    no_sandbox = True,
    verbose = True,
    workspace = "//:WORKSPACE",
)

# Auto-fix BUILD file formatting
buildifier(
    name = "buildifier-fix",
    exclude_patterns = buildifier_exclude_patterns,
    lint_mode = "fix",
    verbose = True,
)
```

```bash
# Check formatting
bazel test //:buildifier

# Fix formatting
bazel run //:buildifier-fix
```

### Header-Only Libraries

```python
# third_party/eigen.BUILD.bazel — header-only with glob, defines, include_prefix
load("@rules_cc//cc:defs.bzl", "cc_library")

EIGEN_FILES = [
    "Eigen/**",
    "Eigen/*",
    "unsupported/Eigen/CXX11/**",
    "unsupported/Eigen/FFT",
    "unsupported/Eigen/src/FFT/**",
]

EIGEN_MPL2_HEADER_FILES = glob(
    EIGEN_FILES,
    exclude = [
        "Eigen/src/Core/util/NonMPL2.h",
        "Eigen/**/CMakeLists.txt",
    ],
)

cc_library(
    name = "eigen",
    hdrs = EIGEN_MPL2_HEADER_FILES,
    defines = [
        "EIGEN_MPL2_ONLY",
    ],
    includes = ["."],
    include_prefix = "eigen3",  # Users include as "eigen3/Eigen/..."
    visibility = ["//visibility:public"],
)
```

### config_setting and select() for Platform-Specific Compilation

```python
# third_party/civetweb.BUILD.bazel
config_setting(
    name = "osx",
    constraint_values = ["@platforms//os:osx"],
)

config_setting(
    name = "windows",
    constraint_values = ["@platforms//os:windows"],
)

config_setting(
    name = "with_ssl",
    define_values = {"with_civetweb_ssl": "true"},
    visibility = ["//visibility:public"],
)

# Feature-based copts with select()
COPTS = [
    "-DUSE_IPV6",
    "-DNDEBUG",
    "-DNO_CGI",
] + select({
    ":with_ssl": [
        "-DOPENSSL_API_1_1",
        "-DNO_SSL_DL",
    ],
    "@//conditions:default": [
        "-DNO_SSL",
    ],
})

# Platform-specific linkopts
cc_library(
    name = "libcivetweb",
    srcs = ["src/civetweb.c"],
    hdrs = ["include/civetweb.h"],
    copts = COPTS,
    includes = ["include"],
    linkopts = select({
        ":windows": [],
        "//conditions:default": ["-lpthread"],
    }) + select({
        ":osx": [],
        ":windows": [],
        "//conditions:default": ["-lrt"],
    }),
    textual_hdrs = [
        "src/handle_form.inl",
        "src/match.inl",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        ":with_ssl": [
            "@boringssl//:crypto",
            "@boringssl//:ssl",
        ],
        "@//conditions:default": [],
    }),
)
```

### Complete C++ Package Example

```python
# A well-structured BUILD file with library, binary, test, and ROS integration
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//build_rules/ros:defs.bzl", "cc_ros_test", "ros_launch")

# Core library
cc_library(
    name = "can_socket",
    srcs = ["src/can_socket.cpp"],
    hdrs = ["include/can_socket/can_socket.h"],
    includes = ["include"],
    deps = [
        ":can_socket_interface",
        "//hardware/base_msgs:cc_base_msgs",
    ],
)

# Interface/abstract base (header-only pattern)
cc_library(
    name = "can_socket_interface",
    srcs = ["include/can_socket/can_socket_interface.h"],
    hdrs = ["include/can_socket/can_socket_interface.h"],
    includes = ["include"],
)

# Controller library depending on core
cc_library(
    name = "controller",
    srcs = ["src/controller.cpp"],
    hdrs = ["include/can_socket/controller.h"],
    includes = ["include"],
    deps = [
        ":can_socket",
        "@ros_comm//:roscpp_lib",
    ],
)

# Executable entry point
cc_binary(
    name = "can_socket_node",
    srcs = ["src/can_socket_node.cpp"],
    deps = [
        ":can_socket",
        ":controller",
        "//ROS/lib",
        "@ros_comm//:roscpp_lib",
    ],
)

# Launch configuration
ros_launch(
    name = "can_socket_launch",
    data = [":can_socket_node"],
    launch_file = "launch/can_socket.launch",
    visibility = [
        "//myproject/services/peripherals:__pkg__",
    ],
)

# Unit test (manual tag: requires special hardware)
cc_test(
    name = "test_can_socket",
    srcs = ["tests/test_can_socket.cpp"],
    tags = ["manual"],  # Skip when running "bazel test :all"
    deps = [
        ":can_socket",
        "@gtests//:gtest_main",
    ],
)

# ROS integration test
cc_ros_test(
    name = "test_controller",
    srcs = ["tests/test_controller.cpp"],
    deps = [
        ":can_socket_interface",
        ":controller",
        "//hardware/base_msgs:cc_base_msgs",
        "//ROS/lib:ros_testing_utils",
        "@gtests//:gtest",
        "@ros_comm//:roscpp_lib",
    ],
)
```

### Third-Party Dependency Repository Macros

```python
# third_party/cppzmq/cppzmq_repositories.bzl
"""A module defining the third party dependency cppzmq."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def cppzmq_repositories():
    maybe(
        http_archive,
        name = "cppzmq",
        build_file = Label("//third_party/cppzmq:BUILD.cppzmq.bazel"),
        sha256 = "...",
        strip_prefix = "cppzmq-4.10.0",
        urls = [
            "https://github.com/zeromq/cppzmq/archive/refs/tags/v4.10.0.tar.gz",
        ],
    )
```

### Building Native C/C++ Dependencies with rules_foreign_cc

```python
# third_party/libzmq/BUILD.libzmq.bazel
load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "all_srcs",
    srcs = glob(
        include = ["**"],
        exclude = ["*.bazel"],
    ),
)

cmake(
    name = "libzmq",
    cache_entries = {
        "WITH_PERF_TOOL": "OFF",
        "ZMQ_BUILD_TESTS": "OFF",
        "ENABLE_CPACK": "OFF",
        "WITH_LIBSODIUM_STATIC": "ON",
        "CMAKE_DISABLE_FIND_PACKAGE_Sodium": "ON",
        "SODIUM_FOUND": "ON",
        "SODIUM_INCLUDE_DIRS": "$EXT_BUILD_DEPS/libsodium/include",
    },
    lib_source = ":all_srcs",
    out_lib_dir = "lib",
    out_shared_libs = [
        "libzmq.so",
        "libzmq.so.5",
    ],
    out_static_libs = ["libzmq.a"],
    deps = [
        "@libsodium",
    ],
)
```

### Custom cc_library Wrappers

```python
# build_rules/plugin/plugin.bzl
"""Plugin rules — cc_library with alwayslink for dynamic loading."""

load("@aspect_bazel_lib//lib:copy_file.bzl", "copy_file")
load("@bazel_skylib//lib:paths.bzl", "paths")
load("@rules_cc//cc:defs.bzl", "cc_library")

def plugin_library(name, plugin_file, **kwargs):
    """Export plugin library with alwayslink for dynamic symbol loading.

    Args:
        name (str): Name of the plugin
        plugin_file (Label): Plugin descriptor file (*.xml)
        **kwargs (dict): Other arguments for cc_library
    """
    basename = paths.basename(plugin_file).split(".")[0]
    filename = "plugins/{}_bazel.xml".format(basename)

    copy_file(
        name = name + "_rename_xml",
        src = plugin_file,
        out = filename,
        tags = ["manual"],
        visibility = ["//visibility:private"],
    )

    cc_library(
        name = name,
        data = kwargs.pop("data", []) + [filename],
        # Prevent Bazel from removing symbols that are dynamically loaded
        alwayslink = True,
        **kwargs
    )
```

### Custom cc_binary Wrappers

```python
# Custom cc_binary with compile-time defines
load("@rules_cc//cc:defs.bzl", "cc_binary")

def sub_driver_node(name, srcs, deps, **kwargs):
    """cc_binary macro that injects SUB_DRIVER_NODE_NAME as a compile-time define.

    Args:
        name: The name of the node.
        srcs: The source files.
        deps: The dependencies.
        kwargs: Additional arguments for cc_binary.
    """
    user_defines = kwargs.pop("defines", [])
    cc_binary(
        name = name,
        srcs = srcs,
        defines = [
            "SUB_DRIVER_NODE_NAME=\\\"{}\\\"".format(name),
        ] + user_defines,
        deps = deps,
        **kwargs
    )
```

### Custom cc_test Wrappers

```python
# build_rules/testing/cc_integration_test.bzl
"""cc_binary test runner with optional launch configuration."""

load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//build_rules/testing:launch_test.bzl", "test_with_auto_launch")

def cc_integration_test(name, srcs, deps, launch_file = None, **kwargs):
    """Creates a cc_binary test runner with optional launch environment.

    Usage:
        cc_integration_test(
            name = "my_test",
            srcs = ["test/my_test.cpp"],
            deps = ["@gtests//:gtest"],
        )

    Args:
        name (str): Name of the target
        srcs (list): Source files for cc_binary
        deps (list): Dependencies for cc_binary
        launch_file (Label): Optional launch config file
        **kwargs (dict): Additional arguments for cc_binary
    """
    size = kwargs.pop("size", "medium")
    timeout = kwargs.pop("timeout", None)
    data = kwargs.pop("data", [])
    tags = kwargs.pop("tags", [])

    binary_name = name + "_runner"
    cc_binary(
        name = binary_name,
        testonly = True,
        srcs = srcs,
        deps = deps,
        tags = ["manual"] + tags,
        visibility = ["//visibility:private"],
        **kwargs
    )

    test_with_auto_launch(
        name = name,
        runner = binary_name,
        launch_file = launch_file,
        size = size,
        timeout = timeout,
        data = data,
        tags = tags,
    )
```

### Sanitizers with Bazel (Recommended .bazelrc Configs)

```bash
# Reference: Envoy proxy's production sanitizer configs

# AddressSanitizer (ASan) + UndefinedBehaviorSanitizer (UBSan)
build:asan --config=sanitizer
build:asan --copt=-fsanitize=address,undefined
build:asan --linkopt=-fsanitize=address,undefined
build:asan --copt=-fno-sanitize=vptr
build:asan --linkopt=-fno-sanitize=vptr
build:asan --copt=-DADDRESS_SANITIZER=1
build:asan --copt=-D__SANITIZE_ADDRESS__
build:asan --test_env=ASAN_OPTIONS=handle_abort=1:allow_addr2line=true:check_initialization_order=true:strict_init_order=true:detect_odr_violation=1
build:asan --test_env=UBSAN_OPTIONS=halt_on_error=true:print_stacktrace=1
build:asan --test_env=ASAN_SYMBOLIZER_PATH

# ThreadSanitizer (TSan)
build:tsan --config=sanitizer
build:tsan --copt=-fsanitize=thread
build:tsan --linkopt=-fsanitize=thread
build:tsan --copt=-DTHREAD_SANITIZER=1
build:tsan --test_env=TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1

# MemorySanitizer (MSan)
build:msan --config=sanitizer
build:msan --copt=-fsanitize=memory
build:msan --linkopt=-fsanitize=memory
build:msan --copt=-fsanitize-memory-track-origins=2
build:msan --test_env=MSAN_SYMBOLIZER_PATH

# Common sanitizer base config
build:sanitizer --copt=-fno-omit-frame-pointer
build:sanitizer --copt=-fno-optimize-sibling-calls
build:sanitizer --copt=-O1
build:sanitizer --copt=-g
build:sanitizer --strip=never
build:sanitizer --define=tcmalloc=disabled
build:sanitizer --define=signal_trace=disabled
```

```bash
# Usage:
bazel test --config=asan //path:test_name
bazel test --config=tsan //path:test_name
bazel test --config=msan //path:test_name
```

### Bazel Query Commands

```bash
# Show all dependencies of a target
bazel query "deps(//path:target)"

# Show reverse dependencies (who depends on this?)
bazel query "rdeps(//..., //path:target)"

# Find all cc_test targets in a package
bazel query "kind(cc_test, //path/...)"

# Show all paths between two targets
bazel query "allpaths(//a:target, //b:target)"

# Configurable query (respects select() and config)
bazel cquery "deps(//path:target)" --config=aarch64

# Action query (show actual compiler commands)
bazel aquery 'mnemonic("CppCompile", deps(//path:target))'

# Find all BUILD files affected by a source file
bazel query "rbuildfiles(path/to/file.cpp)"
```

### Bazel Quick Reference

| Command | Purpose |
|---------|---------|
| `bazel build //path:target` | Build a specific target |
| `bazel build //path/...` | Build all targets in a package recursively |
| `bazel test //path:test_name` | Run a specific test |
| `bazel test //path/...` | Run all tests recursively |
| `bazel test --test_tag_filters=integration //...` | Run integration tests only |
| `bazel test --config=asan //path:test_name` | Test with AddressSanitizer |
| `bazel test --config=tsan //path:test_name` | Test with ThreadSanitizer |
| `bazel run //:refresh_compile_commands` | Generate compile_commands.json |
| `bazel run //:buildifier-fix` | Format BUILD files |
| `bazel run //:gazelle update` | Auto-generate BUILD files |
| `bazel query "deps(//path:target)"` | Show target dependencies |
| `bazel query "rdeps(//..., //path:target)"` | Show reverse dependencies |
| `bazel query "kind(cc_test, //path/...)"` | Find all cc_test targets |
| `bazel aquery 'mnemonic("CppCompile", ...)'` | Show compiler commands |
| `bazel build --config=CI //...` | CI build with remote cache |
| `bazel build --config=aarch64 //path:target` | Cross-compile for ARM64 |
| `bazel build --config=optimize //path:target` | Optimized release build |
| `bazel build --config=opt_with_symbols //path:target` | Release with debug symbols |
| `bazel coverage //path:test_name` | Generate code coverage |

---

## Modern CMake

```cmake
cmake_minimum_required(VERSION 3.20)
project(MyProject VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Create library target
add_library(mylib
    src/mylib.cpp
    include/mylib.h
)

target_include_directories(mylib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(mylib PUBLIC cxx_std_20)

# Create executable
add_executable(myapp src/main.cpp)
target_link_libraries(myapp PRIVATE mylib)

# Dependencies with FetchContent
include(FetchContent)

FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 10.1.1
)
FetchContent_MakeAvailable(fmt)

target_link_libraries(mylib PUBLIC fmt::fmt)

# Testing
enable_testing()
add_subdirectory(tests)

# Install rules
include(GNUInstallDirs)
install(TARGETS mylib myapp
    EXPORT MyProjectTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
```

## Sanitizers

```cmake
# AddressSanitizer (ASan) - memory errors
set(CMAKE_CXX_FLAGS_ASAN
    "-g -O1 -fsanitize=address -fno-omit-frame-pointer"
    CACHE STRING "Flags for ASan build"
)

# UndefinedBehaviorSanitizer (UBSan)
set(CMAKE_CXX_FLAGS_UBSAN
    "-g -O1 -fsanitize=undefined -fno-omit-frame-pointer"
    CACHE STRING "Flags for UBSan build"
)

# ThreadSanitizer (TSan) - data races
set(CMAKE_CXX_FLAGS_TSAN
    "-g -O1 -fsanitize=thread -fno-omit-frame-pointer"
    CACHE STRING "Flags for TSan build"
)

# MemorySanitizer (MSan) - uninitialized reads
set(CMAKE_CXX_FLAGS_MSAN
    "-g -O1 -fsanitize=memory -fno-omit-frame-pointer"
    CACHE STRING "Flags for MSan build"
)

# Usage: cmake -DCMAKE_BUILD_TYPE=ASAN ..
```

## Static Analysis

```yaml
# .clang-tidy configuration
---
Checks: >
  *,
  -fuchsia-*,
  -llvm-*,
  -modernize-use-trailing-return-type,
  -readability-identifier-length

WarningsAsErrors: '*'

CheckOptions:
  - key: readability-identifier-naming.ClassCase
    value: CamelCase
  - key: readability-identifier-naming.FunctionCase
    value: CamelCase
  - key: readability-identifier-naming.VariableCase
    value: lower_case
  - key: readability-identifier-naming.ConstantCase
    value: CamelCase
  - key: readability-identifier-naming.ConstantPrefix
    value: k
  - key: readability-identifier-naming.MemberCase
    value: lower_case
  - key: readability-identifier-naming.MemberSuffix
    value: '_'
  - key: modernize-use-nullptr.NullMacros
    value: 'NULL'
```

```bash
# Run clang-tidy
clang-tidy src/*.cpp -p build/

# Run cppcheck
cppcheck --enable=all --std=c++20 --suppress=missingInclude src/

# Run include-what-you-use
include-what-you-use -std=c++20 src/main.cpp
```

## Testing with Catch2

```cpp
#include "mylib.h"
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Vector operations", "[vector]") {
    std::vector<int> vec{1, 2, 3};

    SECTION("push_back") {
        vec.push_back(4);
        REQUIRE(vec.size() == 4);
        REQUIRE(vec.back() == 4);
    }

    SECTION("pop_back") {
        vec.pop_back();
        REQUIRE(vec.size() == 2);
        REQUIRE(vec.back() == 2);
    }
}

TEST_CASE("Status handling", "[status]") {
    const Status status = RiskyFunction();
    REQUIRE_FALSE(status.ok());
    REQUIRE(status.message() == "error message");
}

TEST_CASE("Floating point", "[math]") {
    REQUIRE_THAT(ComputeValue(),
                 Catch::Matchers::WithinAbs(3.14, 0.01));
}

BENCHMARK("Vector creation") {
    return std::vector<int>(1000);
};

BENCHMARK("Vector fill") {
    std::vector<int> vec(1000);
    for (int i = 0; i < 1000; ++i) {
        vec[i] = i;
    }
    return vec;
};
```

## Testing with GoogleTest

```cpp
#include "calculator.h"
#include <memory>
#include <tuple>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

class CalculatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        calculator_ = std::make_unique<Calculator>();
    }

    std::unique_ptr<Calculator> calculator_;
};

TEST_F(CalculatorTest, Addition) {
    EXPECT_EQ(calculator_->Add(2, 3), 5);
    EXPECT_EQ(calculator_->Add(-1, 1), 0);
}

TEST_F(CalculatorTest, Division) {
    absl::StatusOr<double> quotient = calculator_->Divide(10, 2);
    ASSERT_TRUE(quotient.ok());
    EXPECT_DOUBLE_EQ(*quotient, 5.0);

    absl::StatusOr<double> divide_by_zero = calculator_->Divide(10, 0);
    EXPECT_FALSE(divide_by_zero.ok());
    EXPECT_THAT(divide_by_zero.status().message(), ::testing::HasSubstr("zero"));
}

// Parameterized tests
class AdditionTest : public ::testing::TestWithParam<std::tuple<int, int, int>> {};

TEST_P(AdditionTest, ValidAddition) {
    auto [a, b, expected] = GetParam();
    Calculator calc;
    EXPECT_EQ(calc.Add(a, b), expected);
}

INSTANTIATE_TEST_SUITE_P(
    AdditionSuite,
    AdditionTest,
    ::testing::Values(
        std::make_tuple(1, 2, 3),
        std::make_tuple(-1, -2, -3),
        std::make_tuple(0, 0, 0)
    )
);

// Mock objects
class MockDatabase : public Database {
public:
    MOCK_METHOD(void, Connect, (const std::string&), (override));
    MOCK_METHOD(std::string, Query, (const std::string&), (override));
    MOCK_METHOD(void, Disconnect, (), (override));
};

TEST(ServiceTest, UsesDatabase) {
    MockDatabase mock_db;
    EXPECT_CALL(mock_db, Connect("localhost"))
        .Times(1);
    EXPECT_CALL(mock_db, Query("SELECT *"))
        .WillOnce(::testing::Return("result"));

    Service service(mock_db);
    service.Process();
}
```

## Performance Profiling

```cpp
// Benchmark with Google Benchmark
#include <benchmark/benchmark.h>

static void BM_VectorPush(benchmark::State& state) {
    for (auto _ : state) {
        std::vector<int> vec;
        for (int i = 0; i < state.range(0); ++i) {
            vec.push_back(i);
        }
        benchmark::DoNotOptimize(vec);
    }
}
BENCHMARK(BM_VectorPush)->Range(8, 8<<10);

static void BM_VectorReserve(benchmark::State& state) {
    for (auto _ : state) {
        std::vector<int> vec;
        vec.reserve(state.range(0));
        for (int i = 0; i < state.range(0); ++i) {
            vec.push_back(i);
        }
        benchmark::DoNotOptimize(vec);
    }
}
BENCHMARK(BM_VectorReserve)->Range(8, 8<<10);

BENCHMARK_MAIN();
```

```bash
# Profiling with perf (Linux)
perf record -g ./myapp
perf report

# Profiling with Instruments (macOS)
instruments -t "Time Profiler" ./myapp

# Valgrind callgrind
valgrind --tool=callgrind ./myapp
kcachegrind callgrind.out.*

# Memory profiling
valgrind --tool=massif ./myapp
ms_print massif.out.*
```

## Conan Package Manager

```python
# conanfile.txt
[requires]
fmt/10.1.1
spdlog/1.12.0
catch2/3.4.0

[generators]
CMakeDeps
CMakeToolchain

[options]
fmt:header_only=True
```

```cmake
# CMakeLists.txt with Conan
cmake_minimum_required(VERSION 3.20)
project(MyProject)

find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Catch2 REQUIRED)

add_executable(myapp src/main.cpp)
target_link_libraries(myapp
    PRIVATE
        fmt::fmt
        spdlog::spdlog
)

add_executable(tests test/main.cpp)
target_link_libraries(tests
    PRIVATE
        Catch2::Catch2WithMain
)
```

```bash
# Install dependencies
conan install . --output-folder=build --build=missing
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
cmake --build .
```

## CI/CD with GitHub Actions

```yaml
# .github/workflows/ci.yml
name: CI

on: [push, pull_request]

jobs:
  build:
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [ubuntu-latest, macos-latest, windows-latest]
        compiler: [gcc, clang, msvc]
        build_type: [Debug, Release]

    steps:
    - uses: actions/checkout@v3

    - name: Install dependencies
      run: |
        pip install conan
        conan install . --output-folder=build --build=missing

    - name: Configure
      run: |
        cmake -B build -DCMAKE_BUILD_TYPE=${{ matrix.build_type }}

    - name: Build
      run: cmake --build build --config ${{ matrix.build_type }}

    - name: Test
      run: ctest --test-dir build -C ${{ matrix.build_type }}

  sanitizers:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        sanitizer: [asan, ubsan, tsan]

    steps:
    - uses: actions/checkout@v3

    - name: Build with sanitizer
      run: |
        cmake -B build -DCMAKE_BUILD_TYPE=${{ matrix.sanitizer }}
        cmake --build build

    - name: Run tests
      run: ctest --test-dir build

  static-analysis:
    runs-on: ubuntu-latest

    steps:
    - uses: actions/checkout@v3

    - name: Run clang-tidy
      run: |
        cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        clang-tidy src/*.cpp -p build/

    - name: Run cppcheck
      run: cppcheck --enable=all --error-exitcode=1 src/
```

## Quick Reference

### CMake

| Tool | Purpose | Command |
|------|---------|---------|
| CMake | Build system | `cmake -B build && cmake --build build` |
| Conan | Package manager | `conan install . --build=missing` |
| ASan | Memory errors | `-fsanitize=address` |
| UBSan | Undefined behavior | `-fsanitize=undefined` |
| TSan | Data races | `-fsanitize=thread` |
| clang-tidy | Static analysis | `clang-tidy src/*.cpp` |
| cppcheck | Static analysis | `cppcheck --enable=all src/` |
| Catch2 | Unit testing | `TEST_CASE("name") { REQUIRE(...); }` |
| GoogleTest | Unit testing | `TEST(Suite, Name) { EXPECT_EQ(...); }` |
| Google Benchmark | Performance | `BENCHMARK(func)->Range(...)` |
| Valgrind | Memory profiler | `valgrind --tool=memcheck ./app` |

### Bazel

| Tool / Rule | Purpose | Command / Usage |
|-------------|---------|-----------------|
| Bazel | Build system | `bazel build //path/to:target` |
| `cc_library` | C++ library | `cc_library(name, srcs, hdrs, deps)` |
| `cc_binary` | C++ executable | `cc_binary(name, srcs, deps)` |
| `cc_test` | C++ test | `cc_test(name, srcs, deps)` |
| `proto_library` | Protobuf defs | `proto_library(name, srcs, deps)` |
| `cc_proto_library` | C++ proto stubs | `cc_proto_library(name, deps = [":proto"])` |
| `bazel test` | Run tests | `bazel test //path/to:target --test_output=errors` |
| `bazel query` | Dependency graph | `bazel query 'deps(//path/to:target)'` |
| `bazel run` | Run binary | `bazel run //path/to:target -- args` |
| `bazel cquery` | Configured query | `bazel cquery 'deps(//path/to:target)' --output=files` |
| Buildifier | BUILD formatter | `buildifier -r .` |
| ASan (Bazel) | Memory errors | `bazel test --config=asan //...` |
| TSan (Bazel) | Data races | `bazel test --config=tsan //...` |
| UBSan (Bazel) | Undefined behavior | `bazel test --config=ubsan //...` |
| Hedron | IDE support | Generates `compile_commands.json` via `refresh_compile_commands` |
| `rules_foreign_cc` | Build native C deps | `cmake(name, lib_source, ...)` in BUILD |
