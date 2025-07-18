from engine.bt_engine import BTEngine
import time
from llm.llm_interface import interpret_command
from llm.intent_mapper import update_blackboard_from_intent

def scenario_emergency():
    print("\n--- Scenario: Emergency Detected ---")
    blackboard = {
        'emergency_detected': True,
        'scheduled_task_time': False,
        'user_request_received': False,
        'routine_check_required': False,
        'user_acknowledged': False,
        'anomaly_detected': False,
        'preferences': {},
    }
    engine = BTEngine(blackboard)
    engine.run(ticks=2, tick_interval=1)

def scenario_scheduled_task():
    print("\n--- Scenario: Scheduled Task ---")
    blackboard = {
        'emergency_detected': False,
        'scheduled_task_time': True,
        'user_request_received': False,
        'routine_check_required': False,
        'user_acknowledged': False,
        'anomaly_detected': False,
        'preferences': {},
    }
    engine = BTEngine(blackboard)
    engine.run(ticks=2, tick_interval=1)

def scenario_user_request():
    print("\n--- Scenario: User Request ---")
    blackboard = {
        'emergency_detected': False,
        'scheduled_task_time': False,
        'user_request_received': True,
        'routine_check_required': False,
        'user_acknowledged': False,
        'anomaly_detected': False,
        'preferences': {},
    }
    engine = BTEngine(blackboard)
    engine.run(ticks=2, tick_interval=1)

def scenario_routine_check():
    print("\n--- Scenario: Routine Check ---")
    blackboard = {
        'emergency_detected': False,
        'scheduled_task_time': False,
        'user_request_received': False,
        'routine_check_required': True,
        'user_acknowledged': False,
        'anomaly_detected': True,
        'preferences': {},
    }
    engine = BTEngine(blackboard)
    engine.run(ticks=2, tick_interval=1)

def scenario_cleaning_with_preferences():
    print("\n--- Scenario: Cleaning with User Preferences ---")
    blackboard = {
        'emergency_detected': False,
        'scheduled_task_time': True,  # Triggers cleaning as a scheduled task
        'user_request_received': False,
        'routine_check_required': False,
        'user_acknowledged': True,
        'anomaly_detected': False,
        'preferences': {
            'cleaning_order': ['glasses', 'table', 'floor']  # User prefers this order
        },
    }
    engine = BTEngine(blackboard)
    engine.run(ticks=1, tick_interval=1)
    print("\nNow changing preference: clean table first, then glasses, then floor.")
    blackboard['preferences']['cleaning_order'] = ['table', 'glasses', 'floor']
    engine.run(ticks=1, tick_interval=1)

def interactive_mode():
    print("\n--- Interactive LLM Mode ---")
    blackboard = {
        'emergency_detected': False,
        'scheduled_task_time': False,
        'user_request_received': False,
        'routine_check_required': False,
        'user_acknowledged': False,
        'anomaly_detected': False,
        'preferences': {
            'cleaning_order': ['glasses', 'table', 'floor']
        },
    }
    engine = BTEngine(blackboard)
    while True:
        command = input("\nEnter a human command (or 'quit' to exit): ")
        if command.lower() in ('quit', 'exit'):
            break
        # Simple preference update by command
        if command.lower().startswith('set cleaning order'):
            # Example: set cleaning order to table, glasses, floor
            parts = command.lower().replace('set cleaning order to', '').strip().split(',')
            order = [p.strip() for p in parts if p.strip()]
            if order:
                blackboard['preferences']['cleaning_order'] = order
                print(f"Updated cleaning order preference: {order}")
                continue
        intent = interpret_command(command)
        print(f"LLM interpreted intent: {intent}")
        update_blackboard_from_intent(intent, blackboard)
        engine.tick()

if __name__ == "__main__":
    print("Select mode:\n1. Demo scenarios\n2. Interactive LLM mode")
    mode = input("Enter 1 or 2: ").strip()
    if mode == '2':
        interactive_mode()
    else:
        scenario_emergency()
        time.sleep(1)
        scenario_scheduled_task()
        time.sleep(1)
        scenario_user_request()
        time.sleep(1)
        scenario_routine_check()
        time.sleep(1)
        scenario_cleaning_with_preferences() 