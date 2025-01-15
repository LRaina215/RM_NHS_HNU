# Copyright 2016 Toyota Research Institute

# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy
# of the License at

#   http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.


import logging
import random

from tree import Behavior
from tree import NodeStatus

logger = logging.getLogger(__name__)


class Selector(Behavior):

    """ A Selector runs each child in order until one succeeds.
        This allows failover behaviors.  If one succeeds,
        execution of this behavior stops and it returns NodeStatus.SUCCESS.
        If no child succeed, Selector returns NodeStatus.FAIL.
        If a child is still ACTIVE, then a NodeStatus.ACTIVE status is returned.
        选择器按顺序运行每个子项，直到一个子项成功。
        这允许故障转移行为。 如果成功了，
        此行为的执行停止，并返回 NodeStatus.SUCCESS。
        如果没有子项成功，选择器将返回 NodeStatus.FAIL。
        如果子项仍处于活动状态，则返回 NodeStatus.ACTIVE 状态。
    """

    def __init__(self, name, *args, **kwargs):
        super(Selector, self).__init__(name, run_cb=self.run, *args, **kwargs)

    def run(self, nodedata):
        logger.debug("Selector.run() " + str(self._children))
        self.reset_children_status()
        for c in self._children:
            logger.debug("Selector.tick_child() " + c._name)
            result = self.tick_child(c)
            if result.status == NodeStatus.ACTIVE or result.status == NodeStatus.PENDING:
                return NodeStatus(NodeStatus.ACTIVE,
                                  str("Executing " + self._name + ":" + c._name))
            if result.status == NodeStatus.SUCCESS:
                return NodeStatus(NodeStatus.SUCCESS,
                                  str("Successfully completed " + self._name + ":" + c._name))

        return NodeStatus(NodeStatus.FAIL, str("All children failed in " + self._name))


class Sequencer(Behavior):

    """ A Sequencer runs through each child unless one fails.
        If one fails, the executiong of this behavior stops and it returns NodeStatus.FAIL.
        This behavior returns NodeStatus.ACTIVE until all children have finished
        or one failed.  It returns NodeStatus.SUCCESS if all children succeed.
        Sequencer 贯穿每个子项，除非其中一个子项失败。
        如果失败，此行为的执行将停止，并返回 NodeStatus.FAIL。
        此行为返回 NodeStatus.ACTIVE，直到所有子项都已完成
        或者一个失败了。 如果所有子项都成功，则返回 NodeStatus.SUCCESS。
    """

    def __init__(self, name, *args, **kwargs):
        super(Sequencer, self).__init__(name, run_cb=self.run, *args, **kwargs)

    def run(self, nodedata):
        logger.debug("Sequencer.run() " + str(self._children))
        self.reset_children_status()
        for c in self._children:
            logger.debug("Sequencer.tick_child() " + c._name)
            result = self.tick_child(c)
            if result.status == NodeStatus.ACTIVE or result.status == NodeStatus.PENDING:
                return NodeStatus(NodeStatus.ACTIVE,
                                  str("Executing " + self._name + ":" + c._name))
            if not result.status == NodeStatus.SUCCESS:
                return NodeStatus(NodeStatus.FAIL,
                                  str("Failed to complete " + self._name + ":" + c._name))

        return NodeStatus(NodeStatus.SUCCESS, str("All children succeeded in " + self._name))


class Runner(Behavior):

    """ A Runner runs through all children in order regardless of success/fail.
        It returns NodeStatus.ACTIVE while children are running.
        It always returns Status.SUCCESS after all children have completed.
        运行器按顺序运行所有子项，无论成功/失败如何。
        它在子项运行时返回 NodeStatus.ACTIVE。
        它总是在所有子项完成后返回 Status.SUCCESS。
    """

    def __init__(self, name, *args, **kwargs):
        super(Runner, self).__init__(name, run_cb=self.run, *args, **kwargs)

    def run(self, nodedata):
        logger.debug("Runner.run() " + str(self._children))
        self.reset_children_status()
        for c in self._children:
            logger.debug("Runner.tick_child() " + c._name)
            result = self.tick_child(c)
            if result.status == NodeStatus.ACTIVE or result.status == NodeStatus.PENDING:
                return NodeStatus(NodeStatus.ACTIVE,
                                  str("Executing " + self._name + ":" + c._name))
        return NodeStatus(NodeStatus.SUCCESS, str("All children finished in " + self._name))


class Any(Behavior):

    """ An Any runs every child at each timestep (in order).
        If any child succeeds, all running children are cancelled and an Any returns NodeStatus.SUCCESS.
        If all children fail, it returns NodeStatus.FAIL.
        Otherwise, an Any returns NodeStatus.ACtiVE.
        Any 在每个时间步（按顺序）运行每个子项。
        如果任何子项成功，则取消所有正在运行的子项，并且 Any 返回 NodeStatus.SUCCESS。
        如果所有子项都失败，则返回 NodeStatus.FAIL。
        否则，Any 返回 NodeStatus.ACtiVE。
    """

    def __init__(self, name, *args, **kwargs):
        super(Any, self).__init__(name,
                                  configure_cb=self.configure,
                                  run_cb=self.run,
                                  *args, **kwargs)

    def configure(self, nodedata):
        logger.debug("Any.configure() " + str(self._children))
        self._open_nodes = []
        for child in self._children:
            self._open_nodes.append(child._id)

    def run(self, nodedata):
        logger.debug("Any.run() " + str(self._children))
        active = False
        for c in self._children:
            if c._id in self._open_nodes:
                logger.debug("Any.tick_child() " + c._name)
                result = self.tick_child(c)
                if result.status == NodeStatus.SUCCESS:
                    return NodeStatus(NodeStatus.SUCCESS, str("Found SUCCESS in " + self._name + ":" + c._name))
                if result.status == NodeStatus.ACTIVE or result.status == NodeStatus.PENDING:
                    active = True
        if active:
            return NodeStatus(NodeStatus.ACTIVE, str("Executing " + self._name))

        return NodeStatus(NodeStatus.FAIL, str("Failed to complete " + self._name + ". All children failed."))


class All(Behavior):

    """ An All runs every child at each timestep (in order).
        If one fails, currently running children are canceled and an All returns NodeStatus.FAIL.
        If all succeed, it returns NodeStatus.SUCCESS.
        Otherwise, an All returns NodeStatus.ACtiVE.
        All 在每个时间步（按顺序）运行每个子项。
        如果一个失败，则取消当前正在运行的子项，并且 All 返回 NodeStatus.FAIL。
        如果全部成功，则返回 NodeStatus.SUCCESS。
        否则，All 返回 NodeStatus.ACtiVE。
    """

    def __init__(self, name, *args, **kwargs):
        super(All, self).__init__(name,
                                  configure_cb=self.configure,
                                  run_cb=self.run,
                                  *args, **kwargs)

    def configure(self, nodedata):
        logger.debug("All.configure() " + str(self._children))
        self._open_nodes = []
        for child in self._children:
            self._open_nodes.append(child._id)

    def run(self, nodedata):
        logger.debug("All.run() " + str(self._children))
        active = False
        for c in self._children:
            if c._id in self._open_nodes:
                logger.debug("All.tick_child() " + c._name)
                result = self.tick_child(c)
                if result.status == NodeStatus.FAIL:
                    return NodeStatus(NodeStatus.FAIL, str("Found FAIL in " + self._name + ":" + c._name))
                if result.status == NodeStatus.ACTIVE or result.status == NodeStatus.PENDING:
                    active = True
        if active:
            return NodeStatus(NodeStatus.ACTIVE, str("Executing " + self._name))

        return NodeStatus(NodeStatus.SUCCESS, str("All succeeded in " + self._name))


class Random(Behavior):

    """ A Random chooses one child at random to execute.
        A Random returns the result of that child.
        随机随机选择一个子项执行。
        随机返回该子项的结果。
    """

    def __init__(self, name, *args, **kwargs):
        super(Random, self).__init__(name,
                                     run_cb=self.run,
                                     configure_cb=self.configure,
                                     *args, **kwargs)
        self.child = None

    def configure(self, nodedata):
        if len(self._children) == 0:
            logger.info("No children set")
            self.child = None
            return
        logger.debug("Random.config() " + str(self._children))
        self.child = random.choice(self._children)
        logger.info("Selected random child: " + self.child._name)

    def run(self, nodedata):
        if self.child is None:
            logger.debug("Random.run() empty")
            return NodeStatus(NodeStatus.SUCCESS, "No child selected")
        logger.debug("Random.run() " + str(self.child._name))
        self.reset_children_status()
        return self.tick_child(self.child)


class Progressor(Behavior):

    """ A Progressor keeps track of which child is currently executing,
        and executes all children in order, without re-evaluating
        previously executed children.
        This behavior returns Status.ACTIVE until a child has failed
        or all have succeed.
        If a single child fails, the behavior fails.
        If all children succeed, the behavior succeeds.
        进度器跟踪当前正在执行的子项，
        并按顺序执行所有子项，而无需重新评估
        以前被执行的子项。
        此行为返回状态.活动，直到子项失败
        或者都成功了。
        如果单个子项失败，则行为将失败。
        如果所有子项都成功了，行为就会成功。
    """

    def __init__(self, name, *args, **kwargs):
        super(Progressor, self).__init__(name,
                                         configure_cb=self.configure,
                                         run_cb=self.run,
                                         *args, **kwargs)

    def configure(self, nodedata):
        logger.debug("Progressor.config()" + str(self._children))
        self.index = 0

    def run(self, nodedata):
        logger.debug("Progressor.run()" + str(self._children))
        for i, c in enumerate(self._children):
            if self.index == i:
                logger.debug("Progressor.tick_child() " + c._name)
                result = self.tick_child(c)
                if result.status == NodeStatus.ACTIVE or result.status == NodeStatus.PENDING:
                    return NodeStatus(NodeStatus.ACTIVE,
                                      str("Executing " + self._name + ":" + c._name))
                if not result.status == NodeStatus.SUCCESS:
                    return NodeStatus(result.status,
                                      str("Failed to complete " + self._name + ":" + c._name))
                self.index += 1

        return NodeStatus(NodeStatus.SUCCESS, str("All children succeeded in " + self._name))


class Majority(Behavior):

    """ A Majority runs every child at each timestep (in order).
        This behavior returns Status.ACTIVE until a majority is reached.
        If a majority of children fail, the behavior fails.
        If a majority of children succeed, the behavior succeeds.
        50/50 splits err on the side of success.
        多数在每个时间步（按顺序）运行每个子项。
        此行为返回状态.活动，直到达到多数。
        如果大多数子项失败，行为就会失败。
        如果大多数子项成功了，行为就会成功。
        50/50 在成功的一边分裂错误。
    """

    def __init__(self, name, *args, **kwargs):
        super(Majority, self).__init__(name,
                                       configure_cb=self.configure,
                                       run_cb=self.run,
                                       *args, **kwargs)

    def configure(self, nodedata):
        logger.debug("Majority.config() " + str(self._children))
        self.num_fail = 0
        self.num_succeed = 0
        self._open_nodes = []
        for child in self._children:
            self._open_nodes.append(child._id)

    def run(self, nodedata):
        logger.debug("Majority.run() " + str(self._children))
        num_children = float(len(self._children))
        for c in self._children:
            if c._id in self._open_nodes:
                logger.debug("Majority.tick_child() " + c._name)
                result = self.tick_child(c)
                if result.status == NodeStatus.FAIL:
                    self.num_fail += 1
                if result.status == NodeStatus.SUCCESS:
                    self.num_succeed += 1

                logger.debug("num_fail: " + str(self.num_fail))
                logger.debug("num_succeed: " + str(self.num_succeed))

                logger.debug(
                    "num_fail/num_children= " + str(self.num_fail / num_children))
                if self.num_fail / num_children > 0.5:
                    return NodeStatus(NodeStatus.FAIL, "The majority of children failed")
                logger.debug(
                    "num_succeed/num_children= " + str(self.num_succeed / num_children))
                if self.num_succeed / num_children >= 0.5:
                    return NodeStatus(NodeStatus.SUCCESS, "The majority of children succeeded")

        return NodeStatus(NodeStatus.ACTIVE, str("Executing " + self._name))


class First(Behavior):

    """ A First runs every child at each timestep (in order).
        The result of the first child to finish is returned,
        all other running children are canceled.
        A First 在每个时间步（按顺序）运行每个子项。
        返回第一个完成的子项的结果，
        所有其他跑步的子项将被取消。
    """

    def __init__(self, name, *args, **kwargs):
        super(First, self).__init__(name, run_cb=self.run, *args, **kwargs)

    def run(self, nodedata):
        logger.debug("First.run() " + str(self._children))
        for c in self._children:
            logger.debug("First.tick_child() " + c._name)
            result = self.tick_child(c)
            if result.status == NodeStatus.FAIL or result.status == NodeStatus.SUCCESS:
                return result

        return NodeStatus(NodeStatus.ACTIVE)
