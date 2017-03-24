# Build tests for geometry

drake_add_cc_test(identifier_test identifier_test.cc)
target_link_libraries(identifier_test
    drakeCommon
    )

drake_add_cc_test(geometry_state_test geometry_state_test.cc)
target_link_libraries(geometry_state_test
    drakeGeometry
    )
