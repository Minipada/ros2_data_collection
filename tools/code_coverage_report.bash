#!/bin/bash

if [ ! -d build ]; then
  echo "Please run this script from the root of your workspace."
  echo "Expected directory hierarchy is:"
  echo "example_ws"
  echo " - build"
  echo " - - package_a"
  echo " - - package_b"
  exit 1
fi

set -e

LCOVDIR=lcov
PWD=$(pwd)

COVERAGE_REPORT_VIEW="genhtml"

for opt in "$@" ; do
  case "$opt" in
    clean)
      rm -rf install build log $LCOVDIR
      exit 0
      ;;
    codecovio)
      COVERAGE_REPORT_VIEW=codecovio
      ;;
    genhtml)
      COVERAGE_REPORT_VIEW=genhtml
      ;;
    ci)
      COVERAGE_REPORT_VIEW=ci
      ;;
  esac
done

set -o xtrace
mkdir -p ${LCOVDIR}

# Ignore certain packages:
# - interfaces, which are auto generated files
EXCLUDE_PACKAGES=$(
  colcon list \
    --names-only \
    --packages-select-regex \
      ".*_interfaces" \
  | xargs)
INCLUDE_PACKAGES=$(
  colcon list \
    --names-only \
    --packages-ignore \
      "${EXCLUDE_PACKAGES}" \
  | xargs)

# Capture executed code data.
# shellcheck disable=SC2086
fastcov --lcov \
  -d build \
  --exclude test/ ${EXCLUDE_PACKAGES} \
  --include ${INCLUDE_PACKAGES} \
  --process-gcno \
  --validate-sources \
  --dump-statistic \
  --output "${LCOVDIR}/total_coverage.info"

if [ $COVERAGE_REPORT_VIEW = codecovio ]; then
  curl -s https://codecov.io/bash > codecov
  codecov_version=$(grep -o 'VERSION=\"[0-9\.]*\"' codecov | cut -d'"' -f2)
  shasum -a 512 -c <(curl -s "https://raw.githubusercontent.com/codecov/codecov-bash/${codecov_version}/SHA512SUM" | grep -w "codecov")
  bash codecov \
    -f ${LCOVDIR}/total_coverage.info \
    -R src/ros2_data_collection
elif [ $COVERAGE_REPORT_VIEW = genhtml ]; then
  genhtml ${LCOVDIR}/total_coverage.info \
    --output-directory ${LCOVDIR}/html
fi
