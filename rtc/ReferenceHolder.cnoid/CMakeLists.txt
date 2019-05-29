####
## for each component
##
set(comp_name creekReferenceHolder)
set(comp_sources ${comp_name}.cpp)

#generate_stub_skel(${comp_name}Service)
#set(comp_sources ${comp_sources} ${comp_name}Service_impl.cpp ${comp_name}ServiceSk.cpp)


add_library(${comp_name} SHARED ${comp_sources})
target_link_libraries(${comp_name} ${OPENRTM_LIBRARIES})
target_link_libraries(${comp_name} ${CHOREONOID_LIBRARIES})
target_link_libraries(${comp_name} lapack blas f2c m)
set_target_properties(${comp_name} PROPERTIES PREFIX "")

if (NO_COMP)
  set(target ${comp_name})
else()
  set(target ${comp_name} ${comp_name}Comp)
  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  target_link_libraries(${comp_name}Comp ${OPENRTM_LIBRARIES})
  target_link_libraries(${comp_name}Comp ${CHOREONOID_LIBRARIES})
  target_link_libraries(${comp_name}Comp boost_filesystem)
  target_link_libraries(${comp_name}Comp m)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3 -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing")

#install(TARGETS ${target}
#install(TARGETS ${comp_name} LIBRARY DESTINATION rtc CONFIGURATIONS)
install(TARGETS ${comp_name} LIBRARY DESTINATION lib CONFIGURATIONS)
#install(FILES connect.conf DESTINATION rtc)
