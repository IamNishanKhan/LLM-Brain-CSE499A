def update_blackboard_from_intent(intent: str, blackboard: dict):
    blackboard['emergency_detected'] = (intent == 'emergency')
    blackboard['scheduled_task_time'] = (intent == 'scheduled_task')
    blackboard['user_request_received'] = (intent == 'user_request')
    blackboard['routine_check_required'] = (intent == 'routine_check') 