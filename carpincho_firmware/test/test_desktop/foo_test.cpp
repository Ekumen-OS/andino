#include <foo.h>
#include <gtest/gtest.h>

TEST(FooTest, ConstructOk) {
  carpinchobot::Foo foo;
  EXPECT_EQ(0, foo.get());
}

TEST(FooTest, SetValue) {
  carpinchobot::Foo foo;
  foo.set(1);
  EXPECT_EQ(1, foo.get());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (RUN_ALL_TESTS()) {
  }

  // Always return zero-code and allow PlatformIO to parse results.
  return 0;
}
