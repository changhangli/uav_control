# include/config.cmake
set(CURRENT_DIR  ${PROJECT_SOURCE_DIR}/include/inference_backend/)
install(DIRECTORY  ${CURRENT_DIR}/opencv DESTINATION include/inference_backend)

if(WITH_RKNN)
    install(DIRECTORY ${CURRENT_DIR}/rknn DESTINATION include/inference_backend)
endif()
