# Copyright (c) 2019-present Samsung Electronics Co., Ltd
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
#  USA

IF (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    SET (COMPILER_PREFIX aarch64-linux-gnu)
    SET (CMAKE_C_COMPILER ${COMPILER_PREFIX}-gcc)
    SET (CMAKE_CXX_COMPILER ${COMPILER_PREFIX}-g++)
    SET (LINK ${COMPILER_PREFIX}-g++)
    SET (LD ${COMPILER_PREFIX}-ld)
    SET (AR ${COMPILER_PREFIX}-ar)
    SET (GC_CONFIGURE_HOST --host=aarch64-linux-gnu)
ELSE()
    SET (GC_CONFIGURE_HOST)
ENDIF()

SET (ESCARGOT_CXXFLAGS -fno-rtti)

SET (ESCARGOT_DEFINITIONS -DESCARGOT_64=1 -DENABLE_INTL)

SET (ESCARGOT_CXXFLAGS_DEBUG)
SET (ESCARGOT_CXXFLAGS_RELEASE)

SET (ESCARGOT_LDFLAGS -lpthread -lrt -Wl,--gc-sections)

SET (ESCARGOT_LIBRARIES)
SET (ESCARGOT_INCDIRS)

SET (GC_CFLAGS_ARCH)
SET (GC_LDFLAGS_ARCH)
