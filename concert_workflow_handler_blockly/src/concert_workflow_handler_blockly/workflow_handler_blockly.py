#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/concert_workflow_engine/license/LICENSE
#
# Used for running services that are defined by a multi-master style
# roslaunch - aka a link graph. In these services, the entities are all
# fixed and locked in (as well as their connections) from the start of the
# service to its termination.
#
##############################################################################
# Imports
##############################################################################

import copy
import yaml

import rospy
import rospkg

import rocon_python_utils
from concert_workflow_engine_msgs.msg import Workflow, EnableWorkflows, WorkflowsStatus

##############################################################################
# Classes
##############################################################################


class WorkflowHandlerBlockly(object):

    def __init__(self, service_name='', workflows='', namespace=''):
        self.service_name = service_name
        self.workflows = workflows
        self.msg = EnableWorkflows()
        if namespace:
            self.namespace = namespace + '/'
        self.enable_workflows = rospy.Publisher(self.namespace + 'enable_workflows', EnableWorkflows, latch=True)
        rospy.Subscriber(self.namespace + 'get_workflows_status', WorkflowsStatus, self._update_blockly_workflow_status)
        self._load_workflows()
        self._enable()

    def _update_blockly_workflow_status(self, data):
        pass

    def _load_workflows(self):
        self.msg.service_name = self.service_name
        try:
            wfs_file_name = rocon_python_utils.ros.find_resource_from_string(self.workflows)
        except (rospkg.ResourceNotFound) as e:
            raise rospkg.ResourceNotFound('[%s] %s' % (self.service_name, str(e)))
            # rospy.logwarn()

        with open(wfs_file_name) as wfsf:
            loaded_workflows = yaml.load(wfsf)
            for wf in loaded_workflows['workflows']:
                try:
                    workflow = Workflow()
                    wf_file_name = rocon_python_utils.ros.find_resource_from_string(wf['workflow'])
                    with open(wf_file_name) as wff:
                        workflow.data = wff.readline()
                    self.msg.workflows.append(workflow)
                except (rospkg.ResourceNotFound) as e:
                    rospy.logwarn('Blockly Workflow Handler : %s' % str(e))
                    continue

    def spin(self):
        rospy.spin()

    def shutdown(self):
        self._disable()

    def _enable(self):
        self.msg.enable = True
        self.enable_workflows.publish(self.msg)
        self._loginfo('enable workflow [service name: %s]' % str(self.msg.service_name))

    def _disable(self):
        self.msg.enable = False
        self.enable_workflows.publish(self.msg)
        self._loginfo('disable workflow [service name: %s]' % str(self.msg.service_name))

    def _loginfo(self, msg):
        rospy.loginfo("Blockly Workflow Handler : %s" % str(msg))

    def _logwarn(self, msg):
        rospy.logwarn("Blockly Workflow Handler : %s" % str(msg))
