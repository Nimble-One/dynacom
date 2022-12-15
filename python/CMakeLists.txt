set(${PY_NAME}_SOURCES contact_6d.cpp dynacom_modules.cpp dyna_com.cpp
                       contact_point.cpp dynacom_modules.cpp)
add_library(${PY_NAME}_py SHARED ${${PY_NAME}_SOURCES})
target_link_libraries(${PY_NAME}_py PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
set_target_properties(
  ${PY_NAME}_py
  PROPERTIES INSTALL_RPATH "$ORIGIN/../../.."
             LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
             OUTPUT_NAME ${PY_NAME}
             PREFIX ""
             SUFFIX ${PYTHON_EXT_SUFFIX})
target_link_boost_python(${PY_NAME}_py PRIVATE)
install(TARGETS ${PY_NAME}_py DESTINATION ${${PY_NAME}_INSTALL_DIR})

python_install_on_site(dynacom __init__.py)
