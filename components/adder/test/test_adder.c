// Example

#include "unity.h"
#include "adder.h"

TEST_CASE("add_addsCorrect", "[example, c]") {
    TEST_ASSERT_EQUAL_INT32(2, add(1, 1));
    TEST_ASSERT_EQUAL_INT32(123, add(100, 23));
    TEST_ASSERT_EQUAL_INT32(0, add(-1, 1));
    TEST_ASSERT_EQUAL_INT32(-1, add(-11, 10));
}
