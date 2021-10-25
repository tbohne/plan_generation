#!/usr/bin/env python
import rospy
from plan_generation.srv import get_plan
from plan_generation.msg import plan, action

class PlanGenerator():

    def __init__(self):
        self.generated_plan()

    def generate_plan(self):
        self.generated_plan = plan() 
        self.generated_plan.actions = []

        #########################################################################
        ############################## CUSTOM PLAN ##############################
        #########################################################################
        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.32043739026998)
        a.flt_args.append(8.153532860027937)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320634371203766)
        a.flt_args.append(8.153602612483242)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320556806473469)
        a.flt_args.append(8.153450831140496)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320530579550272)
        a.flt_args.append(8.153441902826218)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        ###################################################
        a = action()
        a.name = "drive_to_container"
        a.task = "charge"
        self.generated_plan.actions.append(a)

        # after recharge - back to point 3
        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320556806473469)
        a.flt_args.append(8.153450831140496)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)
        ###################################################

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.32045524689854)
        a.flt_args.append(8.153333088995939)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320461943134248)
        a.flt_args.append(8.153286773365616)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320559038552034)
        a.flt_args.append(8.15322371714602)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        ###################################################
        a = action()
        a.name = "drive_to_container"
        a.task = "charge"
        self.generated_plan.actions.append(a)

        # after recharge - back to point 3
        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320556806473469)
        a.flt_args.append(8.153450831140496)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)
        ###################################################

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320487612037802)
        a.flt_args.append(8.15310764906039)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320696869403719)
        a.flt_args.append(8.153179075574624)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320591403691296)
        a.flt_args.append(8.153232087440657)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320668193394333)
        a.flt_args.append(8.153359501925676)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)

        a = action()
        a.name = "drive_to"
        a.str_args.append("wgs84")
        a.flt_args.append(52.320663946244821)
        a.flt_args.append(8.153400609372676)
        a.flt_args.append(0)
        a.task = "null"
        self.generated_plan.actions.append(a)
    
        ###################################################
        a = action()
        a.name = "drive_to_container"
        a.task = "charge"
        self.generated_plan.actions.append(a)
        ###################################################

        #########################################################################
        #########################################################################
        #########################################################################

    def get_plan(self):
        res = get_plan()
        res.plan = self.plan
        if len(self.plan.actions) == 0:
            res.succeeded = False
        else: 
            res.succeeded = True
        return res

def node():
    rospy.init_node('plan_generation')
    plan_generator = PlanGenerator()

    rospy.loginfo("setting handcrafted plan..")
    plan_generator.generate_plan()

    rospy.loginfo("providing plan generation service..")
    service = rospy.Service('arox_planner/get_plan', get_plan, plan_generator.get_plan)

    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
