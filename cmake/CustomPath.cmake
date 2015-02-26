# This file assumes that the two custom libraries this
# depends on are located in a directory above the source
# directory, and are called "sdf_builder" and "robogen"
# TODO Change for more flexibility later
# TODO Windows support if required
set(TOL_DEP_DIR "${CMAKE_SOURCE_DIR}/../../")
message("Dependency base directory: ${TOL_DEP_DIR}")

set(SDFBUILDER_INCLUDE_PATH "${TOL_DEP_DIR}/sdf_builder/src")
set(SDFBUILDER_LIBRARIES "${TOL_DEP_DIR}/sdf_builder/build/libsdfbuilder.a")
