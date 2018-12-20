set(EXE_TARGET_NAME iCubProprioception)

include(AddDependenciesPrintUtils)


set(${EXE_TARGET_NAME}_SRC
        src/main.cpp
        src/BatchCADSuperimposer.cpp
        src/CADSuperimposer.cpp
        src/ExtCADSuperimposer.cpp
        src/iKinCADSuperimposer.cpp
        src/SkeletonSuperimposer.cpp
        src/SuperimposerHandler.cpp
)

set(${EXE_TARGET_NAME}_HDR
        include/iCubProprioception/BatchCADSuperimposer.h
        include/iCubProprioception/CADSuperimposer.h
        include/iCubProprioception/ExtCADSuperimposer.h
        include/iCubProprioception/iKinCADSuperimposer.h
        include/iCubProprioception/SkeletonSuperimposer.h
        include/iCubProprioception/SuperimposerHandler.h
)

set(${EXE_TARGET_NAME}_THRIFT_HDR
        thrift/idl_rfm.thrift
        thrift/idl_mesh.thrift
)

# Dependencies
find_package(ICUB REQUIRED)

find_package(OpenCV REQUIRED)

find_package(YARP CONFIG REQUIRED
             COMPONENTS
                 OS
                 math
                 sig
                 dev
                 eigen
                 idl_tools
                 gsl
)

find_package(SuperimposeMesh 0.9.4.0 QUIET)
if(NOT SuperimposeMesh_FOUND)
    message(STATUS "Did not found required master release of SuperimposeMesh. Looking for devel version.")
    find_package(SuperimposeMesh 0.10.100 REQUIRED)
endif()

print_dependency(ICUB)
print_dependency(OpenCV)
print_dependency(YARP)
print_dependency(SuperimposeMesh)


list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

option(PROPRIO_USE_ANALOGS "Use analog data from the hand of the robot" OFF)
if(PROPRIO_USE_ANALOGS)
    add_definition(-DUSE_ANALOGS)
endif()

# Executable main
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR}
                                  ${${EXE_TARGET_NAME}_THRIFT_SRC}
)

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                               "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                               ${ICUB_INCLUDE_DIRS}
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                          ctrlLib
                          iKin
                          ${ICUB_LIBRARIES}
                          ${OpenCV_LIBS}
                          SuperimposeMesh::SuperimposeMesh
                          YARP::YARP_OS
                          YARP::YARP_init
                          YARP::YARP_math
                          YARP::YARP_sig
                          YARP::YARP_dev
                          YARP::YARP_eigen
                          YARP::YARP_gsl
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Custom commands
# EXTRA files
set(${EXE_TARGET_NAME}_APP
        app/iCubProprioception-DataDump.xml
        app/iCubProprioception-External.xml
        app/iCubProprioception-iKin.xml
        app/iCubProprioception-Batch.xml
        app/iCubProprioception-Sim.xml
        app/iCubProprioception-Skeleton.xml
)

set(${EXE_TARGET_NAME}_CONFIG
        config/config.ini
)

set(${EXE_TARGET_NAME}_SHADERS_VERT
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_frame.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.vert
)

set(${EXE_TARGET_NAME}_SHADERS_FRAG
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_frame.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.frag
)

set(${EXE_TARGET_NAME}_MESH
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ail0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ail1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ail2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ail3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_forearm.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_indexbase.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_lil0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_lil1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_lil2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_lil3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_littlebase.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ml0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ml1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ml2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ml3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_palm.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ril0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ril1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ril2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ril3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_ringbase.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_arm/r_tl4.obj
)

if(${CMAKE_GENERATOR} MATCHES "Xcode")
    source_group("Thrift IDL" FILES ${${EXE_TARGET_NAME}_THRIFT_HDR} ${${EXE_TARGET_NAME}_THRIFT_SRC})
    add_custom_command(TARGET  ${EXE_TARGET_NAME} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E make_directory                          $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
                       COMMAND ${CMAKE_COMMAND} -E copy ${${EXE_TARGET_NAME}_MESH}         $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
                       COMMAND ${CMAKE_COMMAND} -E copy ${${EXE_TARGET_NAME}_SHADERS_VERT} $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
                       COMMAND ${CMAKE_COMMAND} -E copy ${${EXE_TARGET_NAME}_SHADERS_FRAG} $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
)
endif()

# Choose APP domain. Default is yarp, may be ICUBcontrib or custom made.
set(APP_DOMAIN "yarp" CACHE STRING "Specify the application domain for shared files.")
if(${APP_DOMAIN} STREQUAL "")
    message(FATAL_ERROR "APP_DOMAIN variable shall not be empty.")
endif()

install(FILES ${${EXE_TARGET_NAME}_APP}          DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${APP_DOMAIN}/applications)
install(FILES ${${EXE_TARGET_NAME}_CONFIG}       DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${APP_DOMAIN}/contexts/${EXE_TARGET_NAME})
install(FILES ${${EXE_TARGET_NAME}_SHADERS_VERT} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${APP_DOMAIN}/contexts/${EXE_TARGET_NAME}/shader)
install(FILES ${${EXE_TARGET_NAME}_SHADERS_FRAG} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${APP_DOMAIN}/contexts/${EXE_TARGET_NAME}/shader)
install(FILES ${${EXE_TARGET_NAME}_MESH}         DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${APP_DOMAIN}/contexts/${EXE_TARGET_NAME}/mesh)
