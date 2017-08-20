# Package `led_detection` {#led_detection}

TODO: to write


## Unit tests

Quick command:

    $ rosrun led_detection unittests.py '*' '*'

More in general:

    $ rosrun led_detection unittests.py ![tests] ![algorithms]

where:

- `![tests]` is a comma separated list of algorithms. May use "`*`".
- `![algorithms]` is a  comma separated list of algorithms. May use  "`*`".

For example, this runs all tests on all algorithms:

    $ rosrun led_detection unittests.py '*' '*'

The default algorithm is called "`baseline`", and its tests are invoked using:

    $ rosrun led_detection <script> '*' 'baseline'
