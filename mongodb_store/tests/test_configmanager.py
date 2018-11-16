#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
from mongodb_store.srv import SetParam
import rospy

class TestConfigManager(unittest.TestCase):
    def test_param_None(self):
        proxy = rospy.ServiceProxy('/config_manager/set_param', SetParam)
        proxy.wait_for_service()

        try:
            result = proxy('{\"path\":\"/chris\",\"value\": null}')
            self.assertFalse(result.success, "Expected SetParameter to fail when adding None value")

        except rospy.service.ServiceException:
            self.fail("Should not crash when providing a null value")



if __name__ == '__main__':
    import rostest
    PKG = 'mongodb_store'
    rospy.init_node('test_config_manager')
    rostest.rosrun(PKG, 'test_config_manager', TestConfigManager)


