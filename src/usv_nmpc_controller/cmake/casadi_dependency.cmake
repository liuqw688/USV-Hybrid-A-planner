# 导出的模型库在公共接口中使用 CasADi 类型，依赖信息需传递给下游包。
find_package(casadi QUIET CONFIG)
if(NOT casadi_FOUND AND NOT TARGET casadi_external)
  find_path(CASADI_INCLUDE_DIR NAMES casadi/casadi.hpp
    PATHS /usr/local/include /usr/include /opt/casadi/include)
  find_library(CASADI_LIBRARY NAMES casadi
    PATHS /usr/local/lib /usr/local/lib64 /usr/lib /usr/lib/x86_64-linux-gnu
    /opt/casadi/lib)
  if(NOT CASADI_INCLUDE_DIR OR NOT CASADI_LIBRARY)
    message(FATAL_ERROR "The USV model library requires CasADi headers and library")
  endif()
  add_library(casadi_external UNKNOWN IMPORTED)
  set_target_properties(casadi_external PROPERTIES
    IMPORTED_LOCATION "${CASADI_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CASADI_INCLUDE_DIR}")
endif()
