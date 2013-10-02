# Find the xerces-c includes and library
#
#  XERCESC_INCLUDE_DIR - Where to find xercesc include sub-directory.
#  XERCESC_LIBRARIES   - List of libraries when using Xerces-C.
#  XERCESC_FOUND       - True if Xerces-C found.

find_path(XERCESC_INCLUDE_DIR xercesc/util/XercesVersion.hpp)
find_library(XERCESC_LIBRARY xerces-c)

# Set found value
set (XERCESC_FOUND FALSE)
if (XERCESC_INCLUDE_DIR)
  if (XERCESC_LIBRARY)
    set (XERCESC_FOUND TRUE)
  endif (XERCESC_LIBRARY)
endif (XERCESC_INCLUDE_DIR)

if(XERCESC_FOUND)
  set(XERCESC_LIBRARIES ${XERCESC_LIBRARY})
  set(XERCESC_INCLUDE_DIRS ${XERCESC_INCLUDE_DIR})
endif(XERCESC_FOUND)

mark_as_advanced(XERCESC_LIBRARY XERCESC_INCLUDE_DIR)
