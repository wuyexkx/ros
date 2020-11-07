cmake_minimum_required(VERSION 2.8.3)
project(building_retrieve)
add_compile_options(-std=c++11)


IF (WIN32)   # --------------windows----------------
    set(OenCV_path "C:/OpenCV-MinGW-Build-OpenCV-3.4.1-x64") # opencv path
    set(OpenCV_INCLUDE_DIRS ${OenCV_path}/include/) 
    file(GLOB WIN_OpenCV_LIBS LIST_DIRECTORIES false ${OenCV_path}/x64/mingw/bin/*.dll) # .a static link library .dll dynamic
ELSEIF(UNIX) # ---------------linux-----------------
    find_package( OpenCV 3 REQUIRED )
ENDIF()


# include and src ---> static library
aux_source_directory(./dbow_src DBOW3_DIRS) 
add_library(${PROJECT_NAME} ${DBOW3_DIRS}) # default STATIC
# include path
include_directories(${PROJECT_SOURCE_DIR}/dbow_src/ ${OpenCV_INCLUDE_DIRS})

# main.cpp 
add_executable( save_database src/save_database.cpp )
add_executable( img_retrieve src/img_retrieve.cpp )
target_link_libraries( save_database ${PROJECT_NAME} ${OpenCV_LIBS} ${WIN_OpenCV_LIBS} )
target_link_libraries( img_retrieve  ${PROJECT_NAME} ${OpenCV_LIBS} ${WIN_OpenCV_LIBS} )
