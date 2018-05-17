#!/bin/bash
set -e
set -x

CURDIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

git -C $CURDIR fetch --all
$CURDIR/git-ffwd-update
