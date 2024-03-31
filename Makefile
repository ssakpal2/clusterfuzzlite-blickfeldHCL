# Copyright 2017 Google Inc. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");

# Simple example of a build file that nicely integrates a fuzz target
# with the rest of the project.
#
# We use 'make' as the build system, but these ideas are applicable
# to any other build system

# By default, use our own standalone_fuzz_target_runner.
# This runner does no fuzzing, but simply executes the inputs
# provided via parameters.
# Run e.g. "make all LIB_FUZZING_ENGINE=/path/to/libFuzzer.a"
# to link the fuzzer(s) against a real fuzzing engine.
#
# OSS-Fuzz will define its own value for LIB_FUZZING_ENGINE.


# Values for CC, CFLAGS, CXX, CXXFLAGS are provided by OSS-Fuzz.
# Outside of OSS-Fuzz use the ones you prefer or rely on the default values.
# Do not use the -fsanitize=* flags by default.
# OSS-Fuzz will use different -fsanitize=* flags for different builds (asan, ubsan, msan, ...)

# You may add extra compiler flags like this:
CXX = g++
CXXFLAGS += -std=c++11 -Wall -Wextra -fsanitize=fuzzer

all: Libfuzzer 

clean:
	rm -fv *.o Libfuzzer

# Continuos integration system should run "make clean && make check"
check: all
	./Libfuzzer

# Fuzz target
Libfuzzer: Libfuzzer.o $(LIB_FUZZING_ENGINE)
	$(CXX) $(CXXFLAGS) $^ -o $@

	
# Object files
Libfuzzer.o: Libfuzzer.cpp Blickfeld_features.hpp Blickfeld_functions.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


