from typing import Dict

def emergency_detected(blackboard: Dict) -> bool:
    return blackboard.get('emergency_detected', False)

def scheduled_task_time(blackboard: Dict) -> bool:
    return blackboard.get('scheduled_task_time', False)

def user_request_received(blackboard: Dict) -> bool:
    return blackboard.get('user_request_received', False)

def routine_check_required(blackboard: Dict) -> bool:
    return blackboard.get('routine_check_required', False)

def user_acknowledged(blackboard: Dict) -> bool:
    return blackboard.get('user_acknowledged', False)

def anomaly_detected(blackboard: Dict) -> bool:
    return blackboard.get('anomaly_detected', False) 