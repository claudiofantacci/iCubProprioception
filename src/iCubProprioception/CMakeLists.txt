set(EXE_TARGET_NAME iCubProprioception)

set(${EXE_TARGET_NAME}_SRC
        src/main.cpp
        src/BatchCADSuperimposer.cpp
        src/CADSuperimposer.cpp
        src/ExtCADSuperimposer.cpp
        src/iKinCADSuperimposer.cpp
        src/SkeletonSuperimposer.cpp
        src/SuperimposerHandler.cpp)

set(${EXE_TARGET_NAME}_HDR
        include/iCubProprioception/common.h
        include/iCubProprioception/BatchCADSuperimposer.h
        include/iCubProprioception/CADSuperimposer.h
        include/iCubProprioception/ExtCADSuperimposer.h
        include/iCubProprioception/iKinCADSuperimposer.h
        include/iCubProprioception/SkeletonSuperimposer.h
        include/iCubProprioception/SuperimposerHandler.h)

set(${EXE_TARGET_NAME}_THRIFT_HDR
        thrift/idl_rfm.thrift
        thrift/idl_mesh.thrift)

# Dependencies
find_package(glew            REQUIRED)
find_package(glfw3           REQUIRED)
find_package(ICUB            REQUIRED)
find_package(OpenCV          REQUIRED)
find_package(OpenGL          REQUIRED)
find_package(SuperimposeMesh REQUIRED)
find_package(YARP            REQUIRED)

list(APPEND CMAKE_MODULE_PATH   ${YARP_MODULE_PATH}
                                ${ICUB_MODULE_PATH})

include(YarpInstallationHelpers)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

# Executable main
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC}     ${${EXE_TARGET_NAME}_HDR}
                                  ${${EXE_TARGET_NAME}_THRIFT_SRC})

target_include_directories(${EXE_TARGET_NAME} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      ${GLEW_INCLUDE_DIRS}
                                                      ${YARP_INCLUDE_DIRS}
                                                      ${ICUB_INCLUDE_DIRS}
                                                      ${OPENGL_INCLUDE_DIR}
                                                      ${ASSIMP_INCLUDE_DIRS})

target_link_libraries(${EXE_TARGET_NAME} PRIVATE SuperimposeMesh::SuperimposeMesh
                                                 ${YARP_LIBRARIES}
                                                 ${ICUB_LIBRARIES}
                                                 ctrlLib
                                                 iKin
                                                 ${OpenCV_LIBS}
                                                 ${OPENGL_LIBRARIES}
                                                 ${GLEW_LIBRARIES}
                                                 glfw)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Custom commands
# EXTRA files
set(${EXE_TARGET_NAME}_APP
        app/iCubProprioception-DataDump.xml
        app/iCubProprioception-External.xml
        app/iCubProprioception-iKin.xml
        app/iCubProprioception-BATCH.xml
        app/iCubProprioception-SIM.xml
        app/iCubProprioception-Skeleton.xml)

set(${EXE_TARGET_NAME}_CONFIG
        config/config.ini)

set(${EXE_TARGET_NAME}_SHADERS_VERT
        shader/shader_model.vert
        shader/shader_background.vert)

set(${EXE_TARGET_NAME}_SHADERS_FRAG
        shader/shader_model.frag
        shader/shader_background.frag)

set(${EXE_TARGET_NAME}_MESH
        mesh/icub_right_arm/r_palm.obj
        mesh/icub_right_arm/r_ail0.obj
        mesh/icub_right_arm/r_ail1.obj
        mesh/icub_right_arm/r_ail2.obj
        mesh/icub_right_arm/r_ail3.obj
        mesh/icub_right_arm/r_forearm.obj
        mesh/icub_right_arm/r_indexbase.obj
        mesh/icub_right_arm/r_ml0.obj
        mesh/icub_right_arm/r_ml1.obj
        mesh/icub_right_arm/r_ml2.obj
        mesh/icub_right_arm/r_ml3.obj
        mesh/icub_right_arm/r_tl0.obj
        mesh/icub_right_arm/r_tl1.obj
        mesh/icub_right_arm/r_tl2.obj
        mesh/icub_right_arm/r_tl3.obj
        mesh/icub_right_arm/r_tl4.obj)

if(${CMAKE_GENERATOR} MATCHES "Xcode")
    source_group("Thrift IDL" FILES ${${EXE_TARGET_NAME}_THRIFT_HDR} ${${EXE_TARGET_NAME}_THRIFT_SRC})
    add_custom_command(TARGET  ${EXE_TARGET_NAME} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E make_directory                          $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
                       COMMAND ${CMAKE_COMMAND} -E copy ${${EXE_TARGET_NAME}_MESH}         $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
                       COMMAND ${CMAKE_COMMAND} -E copy ${${EXE_TARGET_NAME}_SHADERS_VERT} $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
                       COMMAND ${CMAKE_COMMAND} -E copy ${${EXE_TARGET_NAME}_SHADERS_FRAG} $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>)
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
