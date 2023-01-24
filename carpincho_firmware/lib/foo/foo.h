#ifndef CARPINCHOBOT_FOO_H_
#define CARPINCHOBOT_FOO_H_

namespace carpinchobot {

class Foo {
 public:
  Foo() : value_(0) {}
  int get();
  void set(int value);

 private:
  int value_;
};

}  // namespace carpinchobot

#endif  // CARPINCHOBOT_FOO_H_
