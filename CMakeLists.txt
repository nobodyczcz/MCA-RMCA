# CMakeList.txt : CMake project for CBSH-rect-cmake, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
project(MAPD)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
elseif()
	set(CMAKE_CXX_FLAGS "-Wall -Wextra")
	set(CMAKE_CXX_FLAGS_DEBUG "-g")
	set(CMAKE_CXX_FLAGS_RELEASE "-O1")
endif()

set (CMAKE_CXX_STANDARD 14)
#SET(BUILD_SHARED_LIBS OFF)
#SET(CMAKE_EXE_LINKER_FLAGS "-static")
INCLUDE_DIRECTORIES("MAPF/src")

add_executable (MAPD
		MAPD/main.cpp
		MAPD/Agent.cpp
		MAPD/Agent.h
		MAPD/Task.cpp
		MAPD/Task.h
		MAPD/TaskAssignment.h
		MAPD/basic.h
		MAPD/Assignment.h
		MAPD/TaskAssignmentPP.cpp
		MAPD/TaskAssignmentPP.h
		MAPD/TaskAssignmentRegret.h
		MAPD/TaskAssignmentRegretTask.h
		MAPD/TaskAssignmentRegretTask.cpp
		MAPD/TaskAssignmentPPTask.h
		MAPD/TaskAssignmentPPTask.cpp
		MAPD/TaskAssignmentRegret.cpp
		MAPD/TaskAssignmentTaskHeap.h
		MAPD/TaskAssignmentTaskHeap.cpp
		MAPD/TaskAssignmentTaskHeapRegret.h
		MAPD/TaskAssignmentTaskHeapRegret.cpp
		MAPF/src/agents_loader.h
		MAPF/src/compute_heuristic.h
		MAPF/src/ICBSNode.h
		MAPF/src/ICBSSearch.h
		MAPF/src/LLNode.h
		MAPF/src/map_loader.h
		MAPF/src/MDD.h
		MAPF/src/RectangleReasoning.h
		MAPF/src/SingleAgentICBS.h
		MAPF/src/agents_loader.cpp
		MAPF/src/compute_heuristic.cpp
		MAPF/src/ICBSNode.cpp
		MAPF/src/ICBSSearch.cpp
		MAPF/src/LLNode.cpp
		MAPF/src/map_loader.cpp
		MAPF/src/MDD.cpp
		MAPF/src/RectangleReasoning.cpp
		MAPF/src/SingleAgentICBS.cpp
		MAPF/src/ReservationTable.h
		MAPF/src/ReservationTable.cpp
		MAPF/src/Conflict.h
		MAPF/src/Conflict.cpp
		MAPF/src/ConstraintTable.cpp
		MAPF/src/ConstraintTable.h
		MAPF/src/CorridorReasoning.cpp
		MAPF/src/CorridorReasoning.h
		MAPF/src/egraph_reader.h
		MAPF/src/common.h
		MAPF/src/common.cpp
		MAPF/src/MDDNode.h
		MAPF/src/MDDNode.cpp
		MAPF/src/ICBSHSearchPairAnalysis.cpp
		MAPF/src/ICBSHSearchPairAnalysis.h
		MAPF/src/SinglePlanning.h
		MAPF/src/SinglePlanning.cpp
		MAPD/TaskAssignment.cpp MAPD/PathPlanning.cpp MAPD/PathPlanning.h
		MAPD/CBSHOnline.h MAPD/CBSHOnline.cpp
		MAPF/src/map_loader_with_cost.h
		MAPF/src/map_loader_with_cost.cpp
		MAPD/online_simu.h MAPD/online_simu.cpp)






if(APPLE)
	add_definitions('-std=c++11')

	message(STATUS ">>>> Found APPLE <<<<")


	find_path(SPARSEHASH_INCLUDE_DIRS sparsehash/dense_hash_map)
	if (NOT SPARSEHASH_INCLUDE_DIRS)
		message(FATAL_ERROR "Could not find <sparsehash/dense_hash_map>.")
	endif ()
	include_directories(SYSTEM ${SPARSEHASH_INCLUDE_DIRS})


	set(Boost_USE_STATIC_LIBS OFF)
	set(Boost_USE_MULTITHREADED ON)
	set(Boost_USE_STATIC_RUNTIME OFF)
	FIND_PACKAGE(Boost REQUIRED COMPONENTS system program_options)
	include_directories(${Boost_INCLUDE_DIRS})
	link_directories(${Boost_LIBRARY_DIRS})

	target_link_libraries(MAPD ${Boost_LIBRARIES})

else()
	file(READ "/etc/issue" ETC_ISSUE)

	find_path(SPARSEHASH_INCLUDE_DIRS sparsehash/dense_hash_map)
	if (NOT SPARSEHASH_INCLUDE_DIRS)
		message(FATAL_ERROR "Could not find <sparsehash/dense_hash_map>.")
	endif ()
	include_directories(SYSTEM ${SPARSEHASH_INCLUDE_DIRS})

	set(Boost_USE_STATIC_LIBS OFF)
	set(Boost_USE_MULTITHREADED ON)  
	set(Boost_USE_STATIC_RUNTIME OFF)
	FIND_PACKAGE(Boost REQUIRED COMPONENTS system program_options)

	if(Boost_FOUND)
		include_directories(${Boost_INCLUDE_DIRS})
		target_link_libraries(MAPD ${Boost_LIBRARIES})
	else()
		 message(FATAL_ERROR "Could not find boost.")
	endif()
endif()
