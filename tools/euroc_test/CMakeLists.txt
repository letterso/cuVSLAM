# Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA software released under the NVIDIA Open Software License is intended to be used permissively and enable the
# further development of AI technologies. Subject to the terms of this License, NVIDIA confirms that you are free to
# commercially use, modify, and distribute the software with NVIDIA hardware. NVIDIA does not claim ownership to any
# outputs generated using the software or derivative works thereof. By using, reproducing, modifying, distributing,
# performing or displaying any portion or element of the software or derivative works thereof, you agree to be bound by
# this License.

set(MODULE_NAME "euroc_test")

set(SOURCES
    euroc_test.cpp
)

set(LIBS
    cuvslam
    utils
    gflags::gflags
)

setup_app(MODULE_NAME ${MODULE_NAME}
          SOURCES ${SOURCES}
          LIBRARIES ${LIBS})
