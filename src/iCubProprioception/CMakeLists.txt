set(EXE_TARGET_NAME iCubProprioception)

set(${EXE_TARGET_NAME}_SRC
        src/main.cpp
        src/ExtCADSuperimposer.cpp
        src/iKinCADSuperimposer.cpp
        src/SkeletonSuperimposer.cpp
        src/SuperimposerHandler.cpp)

set(${EXE_TARGET_NAME}_HDR
        include/iCubProprioception/common.h
        include/iCubProprioception/ExtCADSuperimposer.h
        include/iCubProprioception/iKinCADSuperimposer.h
        include/iCubProprioception/SkeletonSuperimposer.h
        include/iCubProprioception/SuperimposerHandler.h)

set(${EXE_TARGET_NAME}_THRIFT_HDR
        thrift/idl_rfm.thrift
        thrift/idl_mesh.thrift)

# Dependencies
find_package(GLEW        REQUIRED)
find_package(GLFW3       REQUIRED)
find_package(ICUB        REQUIRED)
find_package(OpenCV      REQUIRED)
find_package(OpenGL      REQUIRED)
find_package(SuperImpose REQUIRED)
find_package(YARP        REQUIRED)

list(APPEND CMAKE_MODULE_PATH   ${YARP_MODULE_PATH}
                                ${ICUB_MODULE_PATH}
                                ${ICUBCONTRIB_MODULE_PATH})

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

target_link_libraries(${EXE_TARGET_NAME} PRIVATE SuperImpose::SuperImpose
                                                 ${YARP_LIBRARIES}
                                                 ${ICUB_LIBRARIES}
                                                 ctrlLib
                                                 iKin
                                                 ${OpenCV_LIBS}
                                                 ${OPENGL_LIBRARIES}
                                                 ${GLEW_LIBRARIES}
                                                 glfw)

target_compile_features(${EXE_TARGET_NAME} PUBLIC cxx_nullptr cxx_constexpr)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Custom commands
# EXTRA files
set(${EXE_TARGET_NAME}_APP
        ${PROJECT_SOURCE_DIR}/app/iCubProprioception.xml.template
        ${PROJECT_SOURCE_DIR}/app/iCubProprioception-DataDump.xml.template)

set(${EXE_TARGET_NAME}_CONFIG
        ${PROJECT_SOURCE_DIR}/config/iCubProprioception-Config.ini)

set(${EXE_TARGET_NAME}_APP_SIM
        ${PROJECT_SOURCE_DIR}/app/iCubProprioception-SIM.xml
        ${PROJECT_SOURCE_DIR}/app/BackgroundApps-SIM.xml)

set(${EXE_TARGET_NAME}_CONFIG_SIM
        ${PROJECT_SOURCE_DIR}/config/iCubProprioception-Config-SIM.ini)

set(${EXE_TARGET_NAME}_SHADERS_VERT
        ${PROJECT_SOURCE_DIR}/shader/shader_model.vert
        ${PROJECT_SOURCE_DIR}/shader/shader_background.vert)

set(${EXE_TARGET_NAME}_SHADERS_FRAG
        ${PROJECT_SOURCE_DIR}/shader/shader_model.frag
        ${PROJECT_SOURCE_DIR}/shader/shader_background.frag)

set(${EXE_TARGET_NAME}_MESH
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_palm.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_ail0.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_ail1.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_ail2.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_ail3.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_forearm.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_forearm_full.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_indexbase.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_ml0.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_ml1.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_ml2.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_ml3.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_palm.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_tl0.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_tl1.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_tl2.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_tl3.obj
        ${PROJECT_SOURCE_DIR}/mesh/icub_right_hand/r_tl4.obj)

if(${CMAKE_GENERATOR} MATCHES "Xcode")
    source_group("Thrift IDL" FILES ${${EXE_TARGET_NAME}_THRIFT_HDR} ${${EXE_TARGET_NAME}_THRIFT_SRC})
    add_custom_command(TARGET  ${EXE_TARGET_NAME} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E make_directory                          $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
                       COMMAND ${CMAKE_COMMAND} -E copy ${${EXE_TARGET_NAME}_MESH}         $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
                       COMMAND ${CMAKE_COMMAND} -E copy ${${EXE_TARGET_NAME}_SHADERS_VERT} $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>
                       COMMAND ${CMAKE_COMMAND} -E copy ${${EXE_TARGET_NAME}_SHADERS_FRAG} $<TARGET_FILE_DIR:${EXE_TARGET_NAME}>)
endif()

install(FILES ${${EXE_TARGET_NAME}_APP_SIM}      DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
install(FILES ${${EXE_TARGET_NAME}_CONFIG_SIM}   DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
install(FILES ${${EXE_TARGET_NAME}_APP}          DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
install(FILES ${${EXE_TARGET_NAME}_CONFIG}       DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
install(FILES ${${EXE_TARGET_NAME}_SHADERS_VERT} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME}/shader)
install(FILES ${${EXE_TARGET_NAME}_SHADERS_FRAG} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME}/shader)
install(FILES ${${EXE_TARGET_NAME}_MESH}         DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME}/mesh)
