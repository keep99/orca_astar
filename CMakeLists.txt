cmake_minimum_required(VERSION 3.15)
project(collision_avoidance)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(third_party/RVO2-2.0.2)
add_subdirectory(third_party/imgui-1.74)

# one
add_executable(collision_avoidance main.cpp astar.cpp blockallocator.cpp)
target_link_libraries(collision_avoidance PRIVATE RVO imgui)

#two
add_executable(Astar_ORCA astar_orca.cpp astar.cpp blockallocator.cpp)
target_link_libraries(Astar_ORCA PRIVATE RVO imgui)

#three
add_executable(BIGAGENT circle.cpp astar.cpp blockallocator.cpp)
target_link_libraries(BIGAGENT PRIVATE RVO imgui)

if (EMSCRIPTEN)
  set_target_properties(collision_avoidance PROPERTIES
    COMPILE_FLAGS_DEBUG "-g4"
    COMPILE_FLAGS "-s USE_SDL=2 -s ASSERTIONS=1 -s ALLOW_MEMORY_GROWTH=1 --emrun"
    LINK_FLAGS_DEBUG "-g4"
    LINK_FLAGS "-s USE_SDL=2 -s ASSERTIONS=1 -s ALLOW_MEMORY_GROWTH=1 --emrun"
    SUFFIX ".html"
  )
else()
  find_package(SDL2 REQUIRED)
  string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
  target_link_libraries(collision_avoidance PRIVATE ${SDL2_LIBRARIES})
endif()

if (EMSCRIPTEN)
  set_target_properties(Astar_ORCA PROPERTIES
    COMPILE_FLAGS_DEBUG "-g4"
    COMPILE_FLAGS "-s USE_SDL=2 -s ASSERTIONS=1 -s ALLOW_MEMORY_GROWTH=1 --emrun"
    LINK_FLAGS_DEBUG "-g4"
    LINK_FLAGS "-s USE_SDL=2 -s ASSERTIONS=1 -s ALLOW_MEMORY_GROWTH=1 --emrun"
    SUFFIX ".html"
  )
else()
  find_package(SDL2 REQUIRED)
  string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
  target_link_libraries(Astar_ORCA PRIVATE ${SDL2_LIBRARIES})
endif()

if (EMSCRIPTEN)
  set_target_properties(BIGAGENT PROPERTIES
    COMPILE_FLAGS_DEBUG "-g4"
    COMPILE_FLAGS "-s USE_SDL=2 -s ASSERTIONS=1 -s ALLOW_MEMORY_GROWTH=1 --emrun"
    LINK_FLAGS_DEBUG "-g4"
    LINK_FLAGS "-s USE_SDL=2 -s ASSERTIONS=1 -s ALLOW_MEMORY_GROWTH=1 --emrun"
    SUFFIX ".html"
  )
else()
  find_package(SDL2 REQUIRED)
  string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
  target_link_libraries(BIGAGENT PRIVATE ${SDL2_LIBRARIES})
endif()
