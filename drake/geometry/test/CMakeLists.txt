# Build tests for geometry

drake_add_cc_test(identifier_test identifier_test.cc)
target_link_libraries(identifier_test
    drakeCommon
    )

drake_add_cc_test(frame_kinematics_set_test frame_kinematics_set_test.cc)
target_link_libraries(frame_kinematics_set_test
    drakeGeometry
    )

drake_add_cc_test(geometry_world_test geometry_world_test.cc)
target_link_libraries(geometry_world_test
    drakeGeometry
    )
