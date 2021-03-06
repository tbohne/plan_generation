#!/usr/bin/env python
from datetime import datetime
import rospy
import copy
from std_msgs.msg import String
from plan_generation.srv import get_plan, get_planResponse
from plan_generation.msg import plan, action

IDLE_TIME = 60


class PlanGenerator:
    """
    Parses and provides handcrafted plans as service based on csv data.
    """

    def __init__(self):
        self.generate_plan()
        self.plan_service = None
        self.sim_extended_idle_time = False
        self.sim_empty_plan = False
        self.sim_infeasible_plan = False
        self.service_available = True
        self.start_idle_time = None
        self.generated_plan = None

        rospy.Subscriber('/activate_plan_service', String, self.activate_service_callback, queue_size=1)
        rospy.Subscriber('/sim_extended_idle_time', String, self.sim_idle_time_callback, queue_size=1)
        rospy.Subscriber('/toggle_unavailable_plan_service', String, self.toggle_unavailable_service_callback,
                         queue_size=1)
        rospy.Subscriber('/sim_empty_plan', String, self.sim_empty_plan_callback, queue_size=1)
        rospy.Subscriber('/sim_infeasible_plan', String, self.sim_infeasible_plan_callback, queue_size=1)

    def activate_service_callback(self, msg):
        """
        Callback that allows external activation of the plan provision service.

        :param msg: callback message
        """
        rospy.loginfo("activate service: %s", msg.data)
        self.provide_service()

    def sim_idle_time_callback(self, msg):
        """
        Initiates simulation of an extended idle time.

        :param msg: callback message
        """
        rospy.loginfo("sim idle time: %s", msg.data)
        self.sim_extended_idle_time = True

    def sim_empty_plan_callback(self, msg):
        """
        Initiates simulation of an empty plan provision.

        :param msg: callback message
        """
        rospy.loginfo("sim empty plan: %s", msg.data)
        self.sim_empty_plan = True

    def sim_infeasible_plan_callback(self, msg):
        """
        Initiates simulation of an infeasible plan provision.

        :param msg: callback message
        """
        rospy.loginfo("sim infeasible plan: %s", msg.data)
        self.sim_infeasible_plan = True

    def toggle_unavailable_service_callback(self, msg):
        """
        Toggles (activates / deactivates) plan service availability.

        :param msg: callback message
        """
        rospy.loginfo("toggle plan service availability: %s", msg.data)
        if self.service_available:
            if self.plan_service:
                self.plan_service.shutdown()
        else:
            self.provide_service()
        self.service_available = not self.service_available

    def generate_plan(self):
        """
        Generates a plan based on the configured csv file.
        """
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
        """
        Plan retrieval -- creates a response for a plan request.

        :param req: request message
        :return: response containing the generated plan
        """
        global IDLE_TIME
        rospy.loginfo("plan retrieval initiated..")

        res = get_planResponse()
        res.generated_plan = copy.deepcopy(self.generated_plan)
        res.succeeded = len(self.generated_plan.actions) > 0
        if self.sim_empty_plan:
            rospy.loginfo("simulating empty plan..")
            res.generated_plan.actions = []
            self.sim_empty_plan = False
        elif self.sim_infeasible_plan:
            if len(res.generated_plan.actions) > 0:
                rospy.loginfo("simulating infeasible plan..")
                res.generated_plan.actions[0].name = "unknown"
            self.sim_infeasible_plan = False
        elif self.sim_extended_idle_time:
            rospy.loginfo("simulating extended idle time..")
            self.start_idle_time = datetime.now()
            self.sim_extended_idle_time = False
            res.succeeded = False

        if self.start_idle_time:
            if (datetime.now() - self.start_idle_time).total_seconds() < IDLE_TIME:
                res.succeeded = False
            else:
                self.start_idle_time = None
        return res

    def provide_service(self):
        """
        Provides plan retrieval as a service.
        """
        rospy.loginfo("providing plan generation service..")
        self.plan_service = rospy.Service('plan_generation/get_plan', get_plan, self.retrieve_plan)


def node():
    """
    Simple node that parses and provides handcrafted plans as service based on csv data.
    """
    rospy.init_node('plan_generator')
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
