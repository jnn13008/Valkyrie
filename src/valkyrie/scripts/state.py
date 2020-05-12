#!/usr/bin/env python
import rospy
from valkyrie.srv import setState, setStateResponse
from valkyrie.srv import getState, getStateResponse

class state():
    """Holds the state for the project"""
    _state_log = list()
    _state = 0
    def __init__(self):
        self.serv = "valkyrie/{}state"
        rospy.loginfo("__init__ State object")
        self._comment = "Init"
        rospy.loginfo("publich " + self.serv.format("get"))
        self._getState = rospy.Service(self.serv.format("get"),
                getState,self._handl_getState_srv)
        rospy.loginfo("publich " + self.serv.format("set"))
        self._setState = rospy.Service(self.serv.format("set"),
                setState,self._hadle_setState_srv)

    def _hadle_setState_srv(self, req):
        """ Sets the state for the robot """
        rospy.loginfo("State set to: " + str(req))
        if len(self._state_log) > 5:
            self._state_log = self._state_log[1:]
        self._state_log.append(req.state)
        self._state = req.state
        self._comment = req.comment
        if self._plan_path():
            self._compute_path()
            self._exec_path()
        res = setStateResponse(1)
        return res

    def _handl_getState_srv(self, req):
        """ returns the state for the robot """
        rospy.loginfo("Returned state: " + str(req))
        res = getStateResponse(self._state, self._comment)
        return res

    def _compute_path(self):
        """ Implement this in sub class """
        rospy.logwarn_once("[_compute_path()] Not implemented!")
        pass

    def _exec_path(self):
        """ Implement this in sub class """
        rospy.logwarn_once("[_exec_path()] Not implemented!")
        pass

    def _plan_path(self):
        """ Implement this in sub class """
        rospy.logwarn_once("[_plan_path()] Not implemented!")
        pass

    def getStateLog(self):
        return self._state_log

    def setState(self, state, comment):
        """ Sets the state of the robot """
        self._state = state
        self._comment = comment

    def getState(self):
        """ Returns a list of state and comment """
        return [self._state, self._comment]



def main():
    rospy.loginfo("valkyrie_state [Init]")
    rospy.init_node("valkyrie_state")
    s = state()
    rospy.loginfo("valkyrie_state [Running]")
    rospy.spin()



if __name__ == '__main__':
    main()
