from enum import Enum
from typing import List, Optional, Any

class Status(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class Node:
    def __init__(self, name: str, tag: str = None):
        self.name = name
        self.tag = tag  # Semantic tag for preference matching
        self.parent: Optional['Node'] = None

    def tick(self, blackboard: dict) -> Status:
        raise NotImplementedError

class Sequence(Node):
    def __init__(self, name: str, children: List[Node]):
        super().__init__(name)
        self.children = children
        for child in children:
            child.parent = self

    def tick(self, blackboard: dict) -> Status:
        for child in self.children:
            status = child.tick(blackboard)
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS

class Selector(Node):
    def __init__(self, name: str, children: List[Node]):
        super().__init__(name)
        self.children = children
        for child in children:
            child.parent = self

    def tick(self, blackboard: dict) -> Status:
        for child in self.children:
            status = child.tick(blackboard)
            if status == Status.SUCCESS:
                return Status.SUCCESS
            elif status == Status.RUNNING:
                return Status.RUNNING
        return Status.FAILURE

class Condition(Node):
    def __init__(self, name: str, condition_fn, tag: str = None):
        super().__init__(name, tag)
        self.condition_fn = condition_fn

    def tick(self, blackboard: dict) -> Status:
        return Status.SUCCESS if self.condition_fn(blackboard) else Status.FAILURE

class Action(Node):
    def __init__(self, name: str, action_fn, tag: str = None):
        super().__init__(name, tag)
        self.action_fn = action_fn

    def tick(self, blackboard: dict) -> Status:
        return self.action_fn(blackboard)

class PreferenceSequence(Node):
    def __init__(self, name: str, children: List[Node], preference_key: str):
        super().__init__(name)
        self.children = children
        self.preference_key = preference_key  # e.g., 'cleaning_order'
        for child in children:
            child.parent = self

    def tick(self, blackboard: dict) -> Status:
        preferences = blackboard.get('preferences', {})
        order = preferences.get(self.preference_key, [])
        # Order children by tag according to user preference
        ordered_children = sorted(
            self.children,
            key=lambda c: order.index(c.tag) if c.tag in order else len(order)
        )
        for child in ordered_children:
            status = child.tick(blackboard)
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS 