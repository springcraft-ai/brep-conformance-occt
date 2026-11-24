# SPDX-License-Identifier: MIT OR Apache-2.0
# On macOS, points the adapter at the package's own copies of its libraries: each library the
# package names by the folder it was built in, and the libc++ the package's libraries load.
# usage: cmake -DBINARY=<adapter> -DLIBRARY_DIR=<the package's library folder> -P relink.cmake
execute_process(COMMAND otool -L "${BINARY}" OUTPUT_VARIABLE listing COMMAND_ERROR_IS_FATAL ANY)
string(REPLACE "\n" ";" lines "${listing}")
set(changes)
foreach(line IN LISTS lines)
    if(line MATCHES "^\t([^ ]+) \\(")
        set(dependency "${CMAKE_MATCH_1}")
        get_filename_component(name "${dependency}" NAME)
        if(dependency MATCHES "^/DLC/" AND EXISTS "${LIBRARY_DIR}/${name}")
            list(APPEND changes -change "${dependency}" "${LIBRARY_DIR}/${name}")
        elseif(name MATCHES "^libc\\+\\+\\.1" AND EXISTS "${LIBRARY_DIR}/libc++.1.0.dylib")
            list(APPEND changes -change "${dependency}" "${LIBRARY_DIR}/libc++.1.0.dylib")
        endif()
    endif()
endforeach()
if(changes)
    execute_process(COMMAND install_name_tool ${changes} "${BINARY}" COMMAND_ERROR_IS_FATAL ANY)
    # The edit breaks the linker's ad hoc signature, without which macOS kills the program.
    execute_process(COMMAND codesign --force --sign - "${BINARY}" COMMAND_ERROR_IS_FATAL ANY)
endif()
