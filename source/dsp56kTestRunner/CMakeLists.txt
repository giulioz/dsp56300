cmake_minimum_required(VERSION 3.10)

project(dsp56kTestRunner)

add_executable(dsp56kTestRunner)

target_sources(dsp56kTestRunner PRIVATE testRunner.cpp)

target_link_libraries(dsp56kTestRunner PUBLIC dsp56kEmu)

add_test(NAME dsp56300_unitTests COMMAND dsp56kTestRunner)
set_tests_properties(dsp56300_unitTests PROPERTIES LABELS "UnitTest;dsp56300")

add_executable(dsp56kManualAudit manualAudit.cpp)
target_link_libraries(dsp56kManualAudit PRIVATE dsp56kEmu)
add_test(NAME dsp56300_manualAudit COMMAND dsp56kManualAudit)
set_tests_properties(dsp56300_manualAudit PROPERTIES LABELS "UnitTest;dsp56300")
