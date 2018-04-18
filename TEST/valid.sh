function valid () {
if [ $? -eq 0 ]; then
    echo OK
else
    echo @@@@@@@@@@@@@@@ FAIL @@@@@@@@@@@@@
    return
fi
}
