# Package `easy_node` {#easy_node}

`easy_node` is a framework to make it easier to create and document ROS nodes.
It allows a *declarative approach* to declaring subscriptions, publishers, and parameters.

The user can directly describe what are the subscription, the publishers, the parameters in a YAML file. The framework then takes care of calling the necessary boilerplate ROS commands for subscribing, publishing, etc.

In addition, `easy_node` can also create the Mardkown documentation from the YAML file.

Using `easy_node` allows to cut 40%-50% of the code required for programming
a node. For an example, see the package [`line_detector2`](#line_detector2),
which contains a re-implementation of  `line_detector` using the new framework.

## Transition plan

The plan is to first use `easy_node` just for documentation of the nodes; then,
later, convert all the nodes to use it.

## YAML file format

If you have a node with the name `![my node]`, implemented in
the file `![my node].py` you must create
a file by the name `![my node].easy_node.yaml` somewhere in the package.

The YAML file must contain 4 sections, each of which is a dictionary.

This is the smallest example of an empty configuration:

    parameters: {}
    subscriptions: {}
    publishers: {}
    contracts: {}

### Configuring parameters

This is the syntax:

    parameters:
        ![name parameter]:
            type: ![type]
            desc: ![description]
            default: ![default value]

where:

- ![type] is one of `float`, `int`, `bool`, `str`.
- ![description] is a description that will appear in the documentation.
- The optional field `default` gives a default value for the parameter.

For example:

    parameters:
        k_d:
            type: float
            desc: The derivative gain for $\theta$.
            default: 1.02

### Describing publishers and subscriptions

The syntax for describing subscribers is:

    subscriptions:
        ![name subscription]:
            topic: ![topic name]
            type: ![message type]
            desc: ![description]

            queue_size: ![queue size]
            latch: ![latch]

where:

- `![topic name]` is the name of the topic to subscribe.
- `![message type]` is a ROS message type name, such as `sensor_msgs/Joy`.
- `![description]` is a Markdown description string.
- `![queue size]`, `![latch]` are optional parameters for ROS publishing/subscribing functions.

The syntax for describing publishers is similar.

Example:


    subscriptions:
        segment_list:
            topic: ~segment_list
            type: duckietown_msgs/SegmentList
            desc: Line detections
            queue_size: 1

    publishers:
        lane_pose:
            topic: ~lane_pose
            type: duckietown_msgs/LanePose
            desc: Estimated pose
            queue_size: 1


### Describing contracts

This is not implemented yet. The idea is to have a place where we can describe
constraints such as:

- "This topic must publish at least at 30 Hz."
- "Panic if you didn't receive a message for 2 seconds."
- "The maximum latency for this is 0.2 s"

Then, we can implement all these checks once and for all in a proper way,
instead of relying on multiple broken implementations

## The `easy_node` functionality

## Automatic docs generation

Generate the docs for each node using this command:

    $ rosrun easy_node generate_docs.py
