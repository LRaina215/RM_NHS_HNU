# Behavior Tree Engine #
This package is a standalone python based behavior tree engine for robotics.  If you'd like ROS integration, check out the [task_behavior_ros](https://github.com/ToyotaResearchInstitute/task_behavior_ros) package.

There are several good webpages describing behavior trees in general:

[Behavior trees for AI: How they work](http://www.gamasutra.com/blogs/ChrisSimpson/20140717/221339/Behavior_trees_for_AI_How_they_work.php)

[Behavior tree Wikipedia](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control))


## Background ##
Behavior trees (BTs) have been well proven in the game developer community, being used in high profile video games such as Halo, Bioshock, and Spore (to name a few).  Since behavior trees are, by definition, non self-referential in that leaf states should never point back to parent states, it makes it very easiy to decompose complex tasks into simpler ones without having to be concerned about how subtasks will fit into a larger task as a whole.  Additionally, BTs are fairly easy to use and understand, which makes them less error prone and easier to debug.
行为树（BT）已经在游戏开发者社区中得到了很好的证明，被用于备受瞩目的视频游戏，如Halo，Bioshock和Spore（仅举几例）。 由于根据定义，行为树是非自引用的，因为叶状态永远不应该指向父状态，因此很容易将复杂任务分解为更简单的任务，而不必担心子任务将如何适应更大的任务作为一个整体。 此外，BT 相当易于使用和理解，这使得它们不易出错且更易于调试。

## Key concepts ##
There are several key concepts of a behavior tree.  Behavior trees can be composed of three types of nodes: a control flow node (called *'behavior'* here), a state modifying node (called a *'decorator'* here) and an execution node (called a *'node'* here) which does the actual work.  The entire tree gets ticked, or run, on each update cycle and as such it is important that the execution nodes are non-blocking, though this isn't strictly enforced.  Each execution node can return 1 of 3 states: active, success, or fail (though I have added some additional states for book-keeping and execution reasons).  The limited number of return states is a part of the magic that makes BTs modularity work.
行为树有几个关键概念。 行为树可以由三种类型的节点组成：控制流节点（此处称为 *'behavior'*）、状态修改节点（此处称为 *'decorator'*）和执行节点（此处称为 *'node'*），它执行实际工作。 整个树在每个更新周期都会被勾选或运行，因此执行节点是非阻塞的，尽管这不是严格执行的，但这一点很重要。 每个执行节点可以返回 3 种状态中的 1 种：活动、成功或失败（尽管出于记账和执行原因，我添加了一些额外的状态）。 返回状态的数量有限是使BT模块化工作的魔力的一部分。

### Behaviors ###
These are the control flow nodes.  By definition they are nodes who have children and control how and when their children are executed.  In this implementation, these nodes are free to have as many children as desired.  Several different types of control flows are included in this package (in branch.py) and should all be well documented within that file.  I will, however explain some key points using examples here.

The two most common behavior tree control nodes are 'selector' and 'sequencer'. A selector runs each child, in order, until one succeeds.  If the child being executed returns "active", that child will continue to be ticked (assessed/run) until a terminal state is reached.  The 'selector' acts as a type of fail-over which allows for recovery behaviors.
这些是控制流节点。 根据定义，它们是具有子节点并控制其子节点的执行方式和时间。 在此实现中，这些节点可以根据需要自由拥有任意数量的子节点。 此包中包含几种不同类型的控制流（在 branch.py 中），并且都应在该文件中详细记录。 但是，我将在这里使用示例来解释一些关键点。

两个最常见的行为树控制节点是“选择器”和“序列器”。选择器按顺序运行每个子项，直到一个子项成功。 如果正在执行的子项返回“活动”，则该子项将继续被勾选（评估/运行），直到达到最终状态。 “选择器”充当一种允许恢复行为的故障转移类型。

**example)** *Say you want your robot to move forward, but for some reason that direction is blocked, you could try the next-best motion 'move right', and do that until forward motion is possible.*

It is important to note that behaviors are re-evaluated at each tick and that this particular one will from the first child at every tick, so we won't get "stuck" in move right when forward motion is desired.

A sequencer runs each child, in order, until one fails.  If the child being executed returns "active", that child will continue to be ticked until a terminal state is reached. The 'sequencer' acts as a type of guarded queue in which the tree can assert certain conditions are met before continuing.

**example)** *Say you want your robot to roam around the room, but you want to make sure you have enough battery power to make it back to base, you can first check that battery level is ok before sending the next waypoint.*

A failed child will stop the behavior of the current task and report a failure up the tree so that appropriate actions can take place.

As you can imagine, there are many different control-flow schemas that can be created here, including those that are machine-learned (which I am very excited about trying!).

### Decorators ###
These are state-modifying nodes.  By definition they have a singular child and their purpose is to change the reported output state of the execution node so that they are executed in the higher level task as desired (without having to change the actual execution node).  Several different decorators have been defined in this package (in decorator.py) and should all be well documented within that file.  A common example is the "repeat" and "negate" decorators.

The "repeat" decorator will report "active" regardless of its child state at every tick.  In essense this will ensure that the child is always run as it will never report a terminal state to the parent behavior.

The "negate" will switch the output state of its child node from "success" to "fail" or vice-versa.  This effectifly adds a "not" to the output and is very useful for reversing limit checks (is it a success to be under or over this value?) without having to re-create the actual check.

### Nodes ###
These are the things that do the actual work.  In this implementation, it is best that the work be 'light' and non-blocking as it will allow the tree to react to other inputs.  Nodes should be designed to do one thing, and to do it well.  Obviously in robotics there are many long-running/blocking actions that will need to happen, such as planning or perception or even trajectory execution.  These things should still happen, but *outside* the behavior tree evaluation loop.  These should be run in separate threads or processes and the current status of them should be reported back to the tree.  Note that this isn't strictly enforced and you could have blocking nodes if you desire, but it is not encouraged.

### Data ###
Each node has access to a data structure which allows the nodes to exchange information and affect each other's execution.  The current implementation seperates the data available for the node *'NodeData'* so that each node only has access to its own data while allowing for remapping from one node's data to another via a *'Blackboard'*.  At the moment blackboards can be shared at any level of the tree if needed, but need to be cascaded down.  I'm not completely sold on my implementation of this structure, and encourage a healthy debate on the matter.

## Organization ##
This package is intended to be a stand-alone implementation of a python-based behavior tree engine which has limited outside dependencies (specifically, no ROS dependency).  Please see [task_behavior_ros](https://github.com/ToyotaResearchInstitute/task_behavior_ros) for ROS support.

* **tree.py** This holds the core implementation of the behavior tree.  The base structure of a 'behavior', 'decorator' and 'node' is defined here.  Note that 'behavior' and 'decorators' are inherited from a 'node' and therefore possess all of a nodes properties.
* **branch.py** This holds the main control-flow behaviors -- There are many!
* **decorator.py** This holds the main output-modifying decorators -- There are many!
* **node.py** While I, in general, would discourage the inclusion of execution nodes in this package as they should be very specific to the application, I have added a few that I found useful for testing as well as execution (think no-ops).

* **test/** In an effort to make the core stable and awesome I have added somewhat extensive unit tests which can be found [here](https://github.com/ToyotaResearchInstitute/task_behavior_engine/tree/master/test).  If something is not working as expected, I highly encourage you to file an issue, or a PR with a test that recreates the problem.

## Getting Started ##

### Installation ###
ROS Integration is provided by the [task_behavior_ros](https://github.com/ToyotaResearchInstitute/task_behavior_ros) package.  If you are intending on using this package with ROS, ignore these install directions and follow the ones in [task_behavior_ros](https://github.com/ToyotaResearchInstitute/task_behavior_ros).

```bash
# Install required libraries
sudo apt-get update
sudo apt-get install  -y \
    cmake \
    git \
    python-pip
sudo pip install catkin_pkg
```
```bash
# Install the package
git clone https://github.com/ToyotaResearchInstitute/task_behavior_engine.git
cd task_behavior_engine
sudo python setup.py install
```

## Usage ##
It's pretty easy to use this library to create a behavior tree (it's python!)  The minimum amount of work that needs to be done is

1. Create a node
2. Create the update loop

The below example can be found in [example/example.py](https://github.com/ToyotaResearchInstitute/task_behavior_engine/blob/master/example/example.py).
### Create a node ###
First, you will want to either create or use an execution node:

``` python
class Count(Node):
    def __init__(self, name, *args, **kwargs):
        super(Count, self).__init__(name,
            run_cb = self.run,
            configure_cb = self.configure,
            cleanup_cb = self.cleanup,
            *args, **kwargs)

    def configure(self, nodedata):
        self.start = nodedata.get_data('start', 0)
        self.limit = nodedata.get_data('limit', 5)
        nodedata.index = self.start

    def run(self, nodedata):
        if nodedata.index < self.limit:
            nodedata.index += 1
            return NodeStatus(NodeStatus.ACTIVE, "Count " + str(nodedata.index))
        return NodeStatus(NodeStatus.SUCCESS, "Count finished at " + str(nodedata.index))

    def cleanup(self, nodedata):
        nodedata.index = self.start
```

Each behavior tree node inherits from the Node class.  You will want to initialize it with a (human) readable name to aid in debugging.  Next, you will want to initialize the parent class.  Inside of the parent class initialization you can setup your callbacks.  Available callbacks are:

* **run\_cb**: _required_ This callback is called at every tick when the node is active.
* **configure\_cb**: This callback is called before the first execution of the node (before run\_cb). In general it will be used to set up variables before execution.
* **cleanup\_cb**: This callback is called after execution has terminated, meaning the node has reported a SUCCESS, FAIL, or CANCEL condition.
* **cancel\_cb**: This callback is called when a CANCEL condition is triggered.

Now you can start filling out your callbacks to make the node execute in the way that you desire.

```python
    def configure(self, nodedata):
        self.start = nodedata.get_data('start', 0)
        self.limit = nodedata.get_data('limit', 5)
        nodedata.index = self.start
```

Here, the node is configured by looking up the start value in its data, and defaulting to 0 if it has not been set.  This allows other nodes to change the initial value of the count.  It also configures how high to count, defaulting to 5 if it hasn't been set by another node.

```python
    def run(self, nodedata):
        if nodedata.index < self.limit:
            nodedata.index += 1
            return NodeStatus(NodeStatus.ACTIVE, "Count " + str(nodedata.index))
        return NodeStatus(NodeStatus.SUCCESS, "Count finished at " + str(nodedata.index))
```

Next, the run_cb does its work by incrementing the count and returns a NodeStatus, which here ACTIVE while counting, and SUCCESS after the count has been reached.  You may also add a string description of the status to aid in debugging (this is optional, but encouraged).

```python
    def cleanup(self, nodedata):
        nodedata.index = self.start
```

Finally, cleanup resets the index to start.


### Create the update loop ###
The simplist method to update the behavior tree is to simply tick the root node.

```python
if __name__ == '__main__':

    count = Count(name="count_index")
    result = NodeStatus(NodeStatus.ACTIVE)

    while result == NodeStatus.ACTIVE:
        result = count.tick()
        print result
        time.sleep(0.1)
```

### Adding a decorator ###
As explained above, decorators are designed to simply change the output of a node.  So say we want count to always be active, we can just add a Repeat decorator as shown below.

```python
if __name__ == '__main__':

    count = Count(name="count_index")
    repeat = Repeat(name="repeat_count", child=count)
    result = NodeStatus(NodeStatus.ACTIVE)

    for i in range(0, 10):
        result = repeat.tick()
        print result
        time.sleep(0.1)
```

### Using a behavior ###
A behavior has a multitude of children and determines when its children are run. If we want to sequence some counts together, we must remember that a sequencer's children are re-evaluated from the first child on every tick.  This will run count1 until success and then move to count2, where it will re-start count1 and again run it until success before moving on.

```python
if __name__ == '__main__':

    # set up common variable blackboard for all nodes
    b = Blackboard()

    count1 = Count(name="count_1", blackboard=b)
    count2 = Count(name="count_2", blackboard=b)
    count3 = Count(name="count_3", blackboard=b)

    finish_counts = Sequencer("finish_counts", blackboard=b)
    finish_counts.add_child(count1)
    finish_counts.add_child(count2)
    finish_counts.add_child(count3)

    # change the limit for the count
    b.save("limit", 1, count1._id)
    b.save("limit", 2, count2._id)
    b.save("limit", 3, count3._id)

    result = NodeStatus(NodeStatus.ACTIVE)

    while result == NodeStatus.ACTIVE:
        result = finish_counts.tick()
        print result
        time.sleep(0.1)
```
Here is also an example of how to use the blackboard to set node parameters.

## Demo ##
The above is included in the package as an example

```bash
$  ./example/example.py
```
