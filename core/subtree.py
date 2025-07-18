from .bt_node import Node, Status
from typing import Callable

class Subtree(Node):
    def __init__(self, name: str, builder_fn: Callable[[], Node]):
        super().__init__(name)
        self.builder_fn = builder_fn
        self._subtree = None

    def tick(self, blackboard: dict) -> Status:
        if self._subtree is None:
            self._subtree = self.builder_fn()
        return self._subtree.tick(blackboard) 