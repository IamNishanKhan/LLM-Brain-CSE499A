from core.bt_node import Sequence, Selector, Condition, Action, PreferenceSequence
from nodes.conditions import (
    emergency_detected, scheduled_task_time, user_request_received, routine_check_required,
    user_acknowledged, anomaly_detected
)
from nodes.actions import (
    assess_emergency, provide_first_aid, call_emergency_services, notify_caregiver,
    announce_task, wait_and_repeat_announcement, assist_with_task, interpret_request,
    execute_user_request, perform_routine_check, log_status,
    clean_glasses, clean_table, clean_floor
)

def build_cleaning_subtree():
    return PreferenceSequence("CleaningPreferenceSequence", [
        Action("CleanGlasses", clean_glasses, tag="glasses"),
        Action("CleanTable", clean_table, tag="table"),
        Action("CleanFloor", clean_floor, tag="floor"),
    ], preference_key="cleaning_order")

def build_main_tree():
    return Sequence("Root", [
        Selector("TriggerSelector", [
            Condition("EmergencyDetected", emergency_detected),
            Condition("ScheduledTaskTime", scheduled_task_time),
            Condition("UserRequestReceived", user_request_received),
            Condition("RoutineCheckRequired", routine_check_required),
        ]),
        Selector("MainSelector", [
            Sequence("EmergencySequence", [
                Condition("EmergencyDetected", emergency_detected),
                Action("AssessEmergency", assess_emergency, tag="assess_emergency"),
                Selector("AidOrCallSelector", [
                    Action("ProvideFirstAid", provide_first_aid, tag="first_aid"),
                    Action("CallEmergencyServices", call_emergency_services, tag="call_emergency"),
                ]),
                Action("NotifyCaregiver", notify_caregiver, tag="notify_caregiver"),
            ]),
            Sequence("ScheduledTaskSequence", [
                Condition("ScheduledTaskTime", scheduled_task_time),
                Action("AnnounceTask", announce_task, tag="announce_task"),
                Selector("AckOrRepeatSelector", [
                    Condition("UserAcknowledged", user_acknowledged),
                    Action("WaitAndRepeatAnnouncement", wait_and_repeat_announcement, tag="repeat_announcement"),
                ]),
                # Insert cleaning subtree as a scheduled task
                build_cleaning_subtree(),
                Action("AssistWithTask", assist_with_task, tag="assist_task"),
            ]),
            Sequence("UserRequestSequence", [
                Condition("UserRequestReceived", user_request_received),
                Action("InterpretRequest", interpret_request, tag="interpret_request"),
                Action("ExecuteUserRequest", execute_user_request, tag="execute_request"),
            ]),
            Sequence("RoutineCheckSequence", [
                Condition("RoutineCheckRequired", routine_check_required),
                Action("PerformRoutineCheck", perform_routine_check, tag="routine_check"),
                Selector("AnomalyOrLogSelector", [
                    Condition("AnomalyDetected", anomaly_detected),
                    Action("LogStatus", log_status, tag="log_status"),
                ]),
            ]),
        ])
    ]) 