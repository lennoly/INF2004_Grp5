#!/bin/sh
# Format the PicoCar application and host tests to the BARR-C:2018 layout.
#
#   sh tools/barr_format.sh            reformat in place
#   sh tools/barr_format.sh --check    report files that would change
#
# clang-format (app_program/.clang-format, tests/host/.clang-format) does
# the bulk of the layout; tools/barr_layout.py adds the rules clang-format
# cannot express.  Template-owned files and the hand-aligned car_config.h
# are left alone.  Run from the repository root.
set -e
FILES=$(ls app_program/*.c app_program/*.h tests/host/*.c tests/host/*.h |
        grep -v -e demo_tasks -e usb_console_compat -e car_config.h)
if [ "$1" = "--check" ]; then
    TMP=$(mktemp -d)
    for f in $FILES; do
        mkdir -p "$TMP/$(dirname "$f")"
        cp "$f" "$TMP/$f"
        cp "$(dirname "$f")/.clang-format" "$TMP/$(dirname "$f")/"
    done
    (cd "$TMP" && clang-format -i $FILES && \
     python3 "$OLDPWD/tools/barr_layout.py" $FILES > /dev/null)
    STATUS=0
    for f in $FILES; do
        if ! cmp -s "$f" "$TMP/$f"; then
            echo "needs formatting: $f"
            STATUS=1
        fi
    done
    rm -rf "$TMP"
    exit $STATUS
fi
clang-format -i $FILES
python3 tools/barr_layout.py $FILES
