# Copyright 2017 Google Inc.
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
LIB_FUZZING_ENGINE ?= standalone_fuzz_target_runner.o
$(info $$LIB_FUZZING_ENGINE is [${LIB_FUZZING_ENGINE}])

# Values for CC, CFLAGS, CXX, CXXFLAGS are provided by OSS-Fuzz.
# Outside of OSS-Fuzz use the ones you prefer or rely on the default values.
# Do not use the -fsanitize=* flags by default.
# OSS-Fuzz will use different -fsanitize=* flags for different builds (asan, ubsan, msan, ...)

# You may add extra compiler flags like this:
CXXFLAGS += -std=c++11

# Paths to external libraries (adjust paths as necessary)
PCL_INCLUDE_DIRS = /path/to/pcl/include
PCL_LIBRARY_DIRS = /path/to/pcl/lib
BOOST_LIBRARY_DIRS = /path/to/boost/lib

INCLUDE_DIRS = -I${PCL_INCLUDE_DIRS}
LIBRARY_DIRS = -L${PCL_LIBRARY_DIRS} -L${BOOST_LIBRARY_DIRS}

# Libraries
LIBS = -lpcl_common -lboost_system -lprotobuf -lblickfeld

# Targets
all: Lib_fuzzer

clean:
	rm -fv *.a *.o  *_fuzzer *_seed_corpus.zip crash-* *.zip

# Continuous integration system should run "make clean && make check"
check: all
	./Lib_fuzzer do_stuff_test_data/*

Lib_fuzzer: Lib_fuzzer.o blickfeld_function.a standalone_fuzz_target_runner.o
	${CXX} ${CXXFLAGS} Lib_fuzzer.o blickfeld_function.a ${LIB_FUZZING_ENGINE} ${LIBRARY_DIRS} ${LIBS} -o $@
	zip -q -r do_stuff_fuzzer_seed_corpus.zip . -i do_stuff_test_data

Lib_fuzzer.o: Lib_fuzzer.cpp
	${CXX} ${CXXFLAGS} ${INCLUDE_DIRS} -c Lib_fuzzer.cpp -o Lib_fuzzer.o

blickfeld_function.a: lidar_provider.o
	ar ruv blickfeld_function.a lidar_provider.o

lidar_provider.o: lidar_provider.cpp blickfeld_features.hpp blickfeld_functions.hpp
	${CXX} ${CXXFLAGS} ${INCLUDE_DIRS} -c lidar_provider.cpp -o lidar_provider.o

standalone_fuzz_target_runner.o: standalone_fuzz_target_runner.cpp
	${CXX} ${CXXFLAGS} ${INCLUDE_DIRS} -c standalone_fuzz_target_runner.cpp -o standalone_fuzz_target_runner.o

.PHONY: all clean check
