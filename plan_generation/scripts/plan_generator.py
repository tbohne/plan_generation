#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from plan_generation.srv import get_plan, get_planResponse
from plan_generation.msg import plan, action

class PlanGenerator():

    def __init__(self):
        self.generate_plan()
        self.plan_service = None

        rospy.Subscriber('/sim_extended_idle_time', String, self.sim_idle_time_callback, queue_size=1)
        rospy.Subscriber('/toggle_unavailable_plan_service', String, self.toggle_unavailable_service_callback, queue_size=1)
        rospy.Subscriber('/sim_empty_plan', String, self.sim_empty_plan_callback, queue_size=1)
        rospy.Subscriber('/sim_infeasible_plan', String, self.sim_infeasible_plan_callback, queue_size=1)
        self.sim_extended_idle_time = False
        self.sim_empty_plan = False
        self.sim_infeasible_plan = False
        self.service_available = True

    def sim_idle_time_callback(self, msg):
        self.sim_extended_idle_time = True

    def sim_empty_plan_callback(self, msg):
        self.sim_empty_plan = True

    def sim_infeasible_plan_callback(self, msg):
        self.sim_infeasible_plan = True

    def toggle_unavailable_service_callback(self):
        if self.service_available:
            if self.plan_service is not None:
                self.plan_service.shutdown()
        else:
            self.provide_service()
        self.service_available = not self.service_available

    def generate_plan(self):
        self.generated_plan = plan()
        self.generated_plan.actions = []
        plan_actions = None

        try:
            with open(rospy.get_param('/plan_path'), 'r') as file:
                plan_actions = file.readlines()
        except IOError as e:
            rospy.loginfo("IO error: %s", e)
        
        if plan_actions:
            for pa in plan_actions:
                a = action()
                action_params = pa.strip().split(",")
                a.name = action_params[0]
                if a.name == "drive_to":
                    a.pose = [float(i) for i in action_params[1:]]
                self.generated_plan.actions.append(a)

    def retrieve_plan(self, req):
        if self.sim_extended_idle_time:
            self.sim_extended_idle_time = False
            return None
        res = get_planResponse()
        res.generated_plan = self.generated_plan
        res.succeeded = len(self.generated_plan.actions) > 0
        if self.sim_empty_plan:
            res.generated_plan.actions = []
            self.sim_empty_plan = False
        elif self.sim_infeasible_plan:
            if len(res.generated_plan.actions) > 0:
                res.generated_plan.actions[0].name = "unknown"
            self.sim_infeasible_plan = False
        return res

    def provide_service(self):
        rospy.loginfo("providing plan generation service..")
        self.plan_service = rospy.Service('arox_planner/get_plan', get_plan, self.retrieve_plan)

def node():
    rospy.init_node('plan_generator')
    rospy.wait_for_message('SMACH_runnning', String)
    plan_generator = PlanGenerator()

    rospy.loginfo("setting handcrafted plan..")
    plan_generator.generate_plan()

    plan_generator.provide_service()

    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
