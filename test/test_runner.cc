#include <testmv.h>

TEST_REGISTRY;

int main() {
  testsoon::default_reporter rep(std::cout);
  return !testsoon::tests().run(rep);
}
