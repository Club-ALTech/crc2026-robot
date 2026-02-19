#include <angles.hpp>
#include "unity.h"

using namespace angles;

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}


void test_wrap() {
    angle<domain::continuous, unit::degrees> a1 {400};
    TEST_ASSERT(a1.normalize().value == 40);
}

int main() {
    UNITY_BEGIN();
    test_wrap();
    UNITY_END();
    return 0;
}