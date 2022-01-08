cmake_minimum_required(VERSION 2.8.3)

project(ur_dlidar)

find_package(catkin REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)

find_package(Boost REQUIRED) 
 if(Boost_VERSION LESS 106700) 
   set(BOOST_PYTHON_COMPONENT python) 
 else()
   set(BOOST_PYTHON_COMPONENT python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}) 
 endif() 
catkin_package()

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
