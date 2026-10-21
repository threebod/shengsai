find_package(Qt6 6.11 REQUIRED COMPONENTS Quick QuickControls2)
if(ANDROID)
    find_package(Qt6 REQUIRED COMPONENTS Bluetooth)
else()
    find_package(Qt6 REQUIRED COMPONENTS SerialPort)
endif()
qt_standard_project_setup(REQUIRES 6.8)
qt_add_executable(shengsai_tuner main.cpp Controller.cpp Controller.h)
qt_add_qml_module(shengsai_tuner URI Shengsai VERSION 1.0 QML_FILES Main.qml)
target_link_libraries(shengsai_tuner PRIVATE Qt6::Quick Qt6::QuickControls2)
if(ANDROID)
    target_link_libraries(shengsai_tuner PRIVATE Qt6::Bluetooth)
    set_target_properties(shengsai_tuner PROPERTIES
        QT_ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android"
        QT_ANDROID_PACKAGE_NAME "org.threebod.shengsai"
        QT_ANDROID_TARGET_SDK_VERSION 35
        QT_ANDROID_COMPILE_SDK_VERSION 36
        QT_ANDROID_MIN_SDK_VERSION 28
        QT_ANDROID_SDK_BUILD_TOOLS_REVISION "36.1.0"
        QT_ANDROID_VERSION_NAME "0.1.0-preview.1"
        QT_ANDROID_VERSION_CODE 1)
else()
    target_link_libraries(shengsai_tuner PRIVATE Qt6::SerialPort)
endif()
install(TARGETS shengsai_tuner BUNDLE DESTINATION . RUNTIME DESTINATION . LIBRARY DESTINATION lib)
