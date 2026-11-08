# piper_cmake

The CMake defaults every C++ package of the stack shares: one function that sets the C++ standard
and the warnings on a target.

## Use

In a package's `package.xml`:

```xml
<buildtool_depend>piper_cmake</buildtool_depend>
```

and in its `CMakeLists.txt`, for each library, executable and test:

```cmake
find_package(piper_cmake REQUIRED)
add_library(my_lib src/my_lib.cpp)
piper_target_defaults(my_lib)
```

`piper_target_defaults(<target>)` makes the target C++23 (a public compile feature, so dependents
get it too) and compiles its C++ sources with `-Wall -Wextra -Wpedantic -Wconversion -Wshadow`.

It is an ament package rather than a `cmake/` directory so that colcon finds it from any package
of the workspace.

## Tests

None of its own: every package that uses it builds with its warnings.
