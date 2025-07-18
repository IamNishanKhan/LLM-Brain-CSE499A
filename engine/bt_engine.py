from trees.main_tree import build_main_tree
from core.bt_node import Status
import time

class BTEngine:
    def __init__(self, blackboard=None):
        self.blackboard = blackboard or {}
        self.tree = build_main_tree()

    def tick(self):
        print("\n--- Ticking Behavior Tree ---")
        status = self.tree.tick(self.blackboard)
        print(f"BT Status: {status.name}")
        return status

    def run(self, ticks=1, tick_interval=1.0):
        for i in range(ticks):
            print(f"\nTick {i+1}:")
            self.tick()
            time.sleep(tick_interval) 