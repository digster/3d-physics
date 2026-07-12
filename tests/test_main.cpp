// ===========================================================================
//  test_main.cpp — provides doctest's main(). All actual tests live in the
//  other test_*.cpp files. Keeping the runner in its own tiny translation unit
//  means those files recompile without regenerating the framework each time.
// ===========================================================================
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
