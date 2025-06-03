#!/bin/bash -e

colcon test --return-code-on-test-failure
colcon test-result --all --verbose
