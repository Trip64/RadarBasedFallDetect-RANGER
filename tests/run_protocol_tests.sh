#!/bin/zsh
set -e

test_dir="${0:A:h}"
c++ -std=c++17 -Wall -Wextra -Werror -I"$test_dir" \
  "$test_dir/ranger_link_protocol_test.cpp" -o /tmp/ranger_link_protocol_test
/tmp/ranger_link_protocol_test
c++ -std=c++17 -Wall -Wextra -Werror \
  "$test_dir/fall_policy_test.cpp" -o /tmp/ranger_fall_policy_test
/tmp/ranger_fall_policy_test
