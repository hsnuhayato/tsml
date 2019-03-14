####
## settings for macro 
##
#set(idl_flags -bcxx -Wba -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp -I${OPENRTM_DIR}/include/rtm/idl)
#set(idl_flags -bcxx -Wba -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp -I${OPENRTM_IDL_DIR})
 set(idl_flags -bcxx -Wba -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp )
set(package_path OpenHRP)
if(NOT QNXNTO)
  set(JAVAC javac)
  set(JAR jar)
  set(IDLJ idlj)
  set(idlj_flags -fclient -fserver -emitAll -td src -d ORBIT2_IDL -d TYPECODE_CORBA_PREFIX)
  set(javac_flags -target 1.8 -d . -sourcepath src)
  set(py_path /home/player/tsml/share/rtm_client)
endif()

####
## macro generate_stub_skel()
## 
macro(generate_stub_skel idl_basename)
  set(idl_file ${CMAKE_CURRENT_SOURCE_DIR}/${idl_basename}.idl)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  if(NOT QNXNTO)
    set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.jar)
    add_custom_command(
      #OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${jarfile} ${CMAKE_CURRENT_BINARY_DIR}/python/${idl_basename}_idl.py
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${jarfile} ${py_path}/${idl_basename}_idl.py
      COMMAND omniidl ${idl_flags} ${idl_file}
      #COMMAND mkdir -p python
      #COMMAND omniidl -bpython -C python -Idir${OPENRTM_IDL_DIR} ${idl_file}
      COMMAND omniidl -bpython -C ${py_path} -Idir${OPENRTM_IDL_DIR} ${idl_file}
      COMMAND ${IDLJ} ${idlj_flags} -Idir${OPENRTM_IDL_DIR} ${idl_file}
      COMMAND mkdir -p bin
      COMMAND ${JAVAC} ${javac_flags} src/*/*.java -d bin/
      COMMAND ${JAR} cf ${jarfile} -C ${CMAKE_CURRENT_BINARY_DIR}/bin .
      DEPENDS ${idl_file}
      )
    install(FILES ${idl_file} DESTINATION share/hrpsys/idl RENAME ${idl_basename}.idl${POSTFIX_FOR_ALTERNATIVES})
    install(FILES ${jarfile} DESTINATION share/hrpsys/jar RENAME ${idl_basename}.jar${POSTFIX_FOR_ALTERNATIVES})
    #install(FILES ${CMAKE_CURRENT_BINARY_DIR}/python/${idl_basename}_idl.py DESTINATION share/hrpsys/python RENAME ${idl_basename}_idl.py${POSTFIX_FOR_ALTERNATIVES})
  else()
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${jarfile} 
      COMMAND omniidl ${idl_flags} ${idl_file}
      DEPENDS ${idl_file}
      )
  endif()
endmacro()

