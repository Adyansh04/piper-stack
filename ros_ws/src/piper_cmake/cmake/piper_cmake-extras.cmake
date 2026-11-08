# piper_target_defaults(<target>): the C++ standard and warnings every C++ target of the stack
# shares. Warnings are fixed, not silenced; CUDA sources keep nvcc's own flags.
function(piper_target_defaults target)
  target_compile_features(${target} PUBLIC cxx_std_23)
  target_compile_options(${target} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow>)
endfunction()
