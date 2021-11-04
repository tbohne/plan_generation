#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from plan_generation.srv import get_plan, get_planResponse
from plan_generation.msg import plan, action

class PlanGenerator():

    def __init__(self):
        self.generate_plan()

    def generate_plan(self):
        self.generated_plan = plan()
        self.generated_plan.actions = []

        with open(rospy.get_param('/plan_path'), 'r') as file:
            plan_actions = file.readlines()
        
        if plan_actions:
            for pa in plan_actions:                
                a = action()
                action_params = pa.strip().split(",")
                a.name = action_params[0]
                if a.name == "drive_to":
                    a.pose = [float(i) for i in action_params[1:]]
                self.generated_plan.actions.append(a)

    def retrieve_plan(self, req):
        res = get_planResponse()
        res.generated_plan = self.generated_plan
        res.succeeded = len(self.generated_plan.actions) > 0
        return res

def node():
    rospy.init_node('plan_generator')
    rospy.wait_for_message('SMACH_runnning', String)
    plan_generator = PlanGenerator()

    rospy.loginfo("setting handcrafted plan..")
    plan_generator.generate_plan()

    rospy.loginfo("providing plan generation service..")
    service = rospy.Service('arox_planner/get_plan', get_plan, plan_generator.retrieve_plan)

    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
