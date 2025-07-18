from core.bt_node import Status
from typing import Dict

def assess_emergency(blackboard: Dict) -> Status:
    print("Assessing emergency...")
    return Status.SUCCESS

def provide_first_aid(blackboard: Dict) -> Status:
    print("Providing first aid...")
    return Status.SUCCESS

def call_emergency_services(blackboard: Dict) -> Status:
    print("Calling emergency services!")
    blackboard['emergency_services_called'] = True
    return Status.SUCCESS

def notify_caregiver(blackboard: Dict) -> Status:
    print("Notifying caregiver...")
    blackboard['caregiver_notified'] = True
    return Status.SUCCESS

def announce_task(blackboard: Dict) -> Status:
    print("Announcing scheduled task...")
    blackboard['task_announced'] = True
    return Status.SUCCESS

def wait_and_repeat_announcement(blackboard: Dict) -> Status:
    print("Waiting and repeating announcement...")
    return Status.SUCCESS

def assist_with_task(blackboard: Dict) -> Status:
    print("Assisting with task...")
    return Status.SUCCESS

def interpret_request(blackboard: Dict) -> Status:
    print("Interpreting user request...")
    blackboard['request_interpreted'] = True
    return Status.SUCCESS

def execute_user_request(blackboard: Dict) -> Status:
    print("Executing user request...")
    return Status.SUCCESS

def perform_routine_check(blackboard: Dict) -> Status:
    print("Performing routine check...")
    return Status.SUCCESS

def log_status(blackboard: Dict) -> Status:
    print("Logging status...")
    return Status.SUCCESS

# Cleaning actions for preference demo
def clean_glasses(blackboard: Dict) -> Status:
    print("Cleaning glasses...")
    return Status.SUCCESS

def clean_table(blackboard: Dict) -> Status:
    print("Cleaning table...")
    return Status.SUCCESS

def clean_floor(blackboard: Dict) -> Status:
    print("Cleaning floor...")
    return Status.SUCCESS 