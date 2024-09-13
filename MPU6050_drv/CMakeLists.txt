# Đặt tên cho thư viện
add_library(MPU6050_drv
    mpu6050.c
)

# Thêm đường dẫn tới thư mục chứa tệp tiêu đề
target_include_directories(MPU6050_drv PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Liên kết thư viện cần thiết từ Raspberry Pi Pico SDK
target_link_libraries(MPU6050_drv
    pico_stdlib
    hardware_i2c
)

# (Tùy chọn) Nếu có một tệp CMakeLists.txt chính, hãy thêm dòng này để đảm bảo
# thư viện MPU6050_drv được xây dựng khi bạn xây dựng dự án chính
if (NOT TARGET ${PROJECT_NAME})
    add_executable(${PROJECT_NAME} main.cpp) # thay main.c bằng tệp chính của bạn
    target_link_libraries(${PROJECT_NAME} MPU6050_drv)
endif()
