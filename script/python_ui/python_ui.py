#!/usr/bin/python2.7

import os,sys
sys.path = [''] + [os.getcwd() + '/../../share/rtm_client'] + ['/usr/lib/python2.7/dist-packages'] + sys.path[1:]

from hrpsys.hrpsys_config import *
import OpenHRP
rtm.nsport=2809
# try:
#     from hrpsys_config import *
#     import OpenHRP
#     rtm.nsport=2809
# except:
#     print "import without hrpsys"
#     import rtm
#     from rtm import *
#     from OpenHRP import *
#     import waitInput
#     from waitInput import *
#     import socket
#     import time

class JVRC(HrpsysConfigurator):
    # additional rtc
    stick = None
    stick_svc = None
    stick_version = None

    seq = None
    seq_svc = None
    seq_version = None

    wpg = None
    wpg_svc = None
    wpg_version = None

    arm = None
    arm_svc = None
    arm_version = None

    sh = None
    sh_svc = None
    sh_version = None

    st = None
    st_svc = None
    st_version = None

    camera = None
    camera_svc = None
    camera_version = None

    pcl = None
    pcl_svc = None
    pcl_version = None
    
    arm = None
    arm_svc = None
    arm_version = None
    
    
    def getRTCList(self):
          return [
              # ['stick', 'GamepadRTC'],
              ['kf', 'creekStateEstimator'],
              # ['seq', 'creekSequencePlayer'],
              ['wpg', 'sony'],
              # ['arm', 'creekArmControlCartesian'],
              ['sh', 'creekReferenceHolder'],
              ['st', 'Stabilizer']
            ]
      
    def checkSimulationMode(self):
        self.hgc = findRTC("HGcontroller0")
        self.pdc = findRTC("creekPdServo0")
        if self.hgc or self.pdc:
            self.simulation_mode = True
            print(self.configurator_name + "find pd")
        else:
            self.simulation_mode = False

    def connectComps(self):
        rtm.connectPorts(self.rh.port("q"), self.sh.port("qCur"))
        rtm.connectPorts(self.sh.port("qOut"),   self.pdc.port("qRef"))

        rtm.connectPorts(self.rh.port("gyrometer"), self.kf.port("rate"))
        rtm.connectPorts(self.rh.port("gsensor"),   self.kf.port("acc"))

        if self.seq != None:
            rtm.connectPorts(self.sh.port("qOut"),self.seq.port("qInit"))
            rtm.connectPorts(self.sh.port("basePosOut"), self.seq.port("basePosInit"))
            rtm.connectPorts(self.sh.port("baseRpyOut"), self.seq.port("baseRpyInit"))
            rtm.connectPorts(self.sh.port("zmpRefOut"), self.seq.port("zmpRefInit"))
            rtm.connectPorts(self.seq.port("qRef"),    self.sh.port("qIn"))
            rtm.connectPorts(self.seq.port("basePos"), self.sh.port("basePosIn"))
            rtm.connectPorts(self.seq.port("baseRpy"), self.sh.port("baseRpyIn"))
            rtm.connectPorts(self.seq.port("zmpRef"),  self.sh.port("zmpRefIn"))

        # rtm.connectPorts(rh.port("q"),   self.pdcC.port("qIn"))
        # rtm.connectPorts(kf.port("rpy"),    self.pdcC.port("baseRpy"))
        # rtm.connectPorts(sh.port("basePosOut"),  self.pdcC.port("basePos"))

        if self.wpg != None:
            rtm.connectPorts(self.rh.port("rfsensor"),  self.wpg.port("rfsensor"))
            rtm.connectPorts(self.rh.port("lfsensor"),  self.wpg.port("lfsensor"))
            rtm.connectPorts(self.rh.port("rhsensor"),  self.wpg.port("rhsensor"))
            rtm.connectPorts(self.rh.port("lhsensor"),  self.wpg.port("lhsensor"))
            rtm.connectPorts(self.sh.port("qOut"),   self.wpg.port("mc"))
            rtm.connectPorts(self.wpg.port("refq"),  self.sh.port("qIn"))
            rtm.connectPorts(self.wpg.port("rzmp"), self.sh.port("zmpRefIn"))
            rtm.connectPorts(self.wpg.port("basePosOut"), self.sh.port("basePosIn"))
            rtm.connectPorts(self.wpg.port("baseRpyOut"), self.sh.port("baseRpyIn"))

        if self.st!= None:
            rtm.connectPorts(self.kf.port("rpy"), self.st.port("rpy"))
            ## old wu st
            #rtm.connectPorts(self.rh.port("rfsensor"), self.st.port("forceR"))
            #rtm.connectPorts(self.rh.port("lfsensor"), self.st.port("forceL"))
            rtm.connectPorts(self.rh.port("rfsensor"), self.st.port("rfsensor"))
            rtm.connectPorts(self.rh.port("lfsensor"), self.st.port("lfsensor"))
            rtm.connectPorts(self.rh.port("q"), self.st.port("qCurrent"))
            rtm.connectPorts(self.sh.port("qOut"), self.st.port("qRef"))
            rtm.connectPorts(self.sh.port("zmpRefOut"), self.st.port("zmpRef"))
            rtm.connectPorts(self.sh.port("basePosOut"), self.st.port("basePosIn"))
            rtm.connectPorts(self.sh.port("baseRpyOut"), self.st.port("baseRpyIn"))
            rtm.connectPorts(self.wpg.port("contactStates"), self.st.port("contactStates"))
            rtm.connectPorts(self.wpg.port("toeheelRatio"), self.st.port("toeheelRatio"))
            rtm.connectPorts(self.wpg.port("controlSwingSupportTime"), self.st.port("controlSwingSupportTime"))
            #rtm.connectPorts(wpg.port("localEEpos"), st.port("localEEpos"))
            rtm.connectPorts(self.st.port("q"),  self.pdc.port("qRef"))
            rtm.connectPorts(self.st.port("q"),  self.wpg.port("mc"))
        else:
            rtm.connectPorts(self.sh.port("qOut"), self.pdc.port("qRef"))

        if self.camera != None:
            rtm.connectPorts(self.rh.port("rcamera"),  self.camera.port("rcamera"))
            rtm.connectPorts(self.rh.port("lcamera"),  self.camera.port("lcamera"))
            rtm.connectPorts(self.rh.port("rhcamera"), self.camera.port("rhcamera"))
            rtm.connectPorts(self.rh.port("lhcamera"), self.camera.port("lhcamera"))

            # rtm.connectPorts(servoC.port("rcameraPose"),    camera.port("rcameraPose"))
            # rtm.connectPorts(servoC.port("lcameraPose"),    camera.port("lcameraPose"))
            # rtm.connectPorts(servoC.port("rhcameraPose"),   camera.port("rhcameraPose"))
            # rtm.connectPorts(servoC.port("lhcameraPose"),   camera.port("lhcameraPose"))

        if self.stick != None:
            rtm.connectPorts(self.stick.port("axes"), self.wpg.port("axes"))
            rtm.connectPorts(self.stick.port("buttons"), self.wpg.port("buttons"))

        if self.pcl != None:
            rtm.connectPorts(self.rh.port("ranger"), self.pcl.port("ranger"))
            rtm.connectPorts(self.rh.port("q"),   self.pcl.port("qCur"))
            rtm.connectPorts(self.kf.port("rpy"),    self.pcl.port("baseRpyAct"))
            #rtm.connectPorts(sh.port("baseRpyOut"), pcl.port("baseRpyAct"))
            rtm.connectPorts(self.sh.port("baseRpyOut"), self.pcl.port("baseRpy"))
            rtm.connectPorts(self.sh.port("basePosOut"), self.pcl.port("basePos"))
            rtm.connectPorts(self.pcl.port("baseRpyOut"), self.wpg.port("baseRpyInit"))
            rtm.connectPorts(self.pcl.port("basePosOut"), self.wpg.port("basePosInit"))
            rtm.connectPorts(self.pcl.port("baseRpyOut"), self.sh.port("baseRpyIn"))
            rtm.connectPorts(self.pcl.port("basePosOut"), self.sh.port("basePosIn"))
            rtm.connectPorts(self.stick.port("axes"), self.pcl.port("axes"))
            rtm.connectPorts(self.stick.port("buttons"), self.pcl.port("buttons"))

        if self.arm != None:
            self.rtm.connectPorts(sh.port("qOut"),        self.arm.port("qCur"))
            self.rtm.connectPorts(sh.port("basePosOut"),  self.arm.port("basePos"))
            self.rtm.connectPorts(sh.port("baseRpyOut"),  self.arm.port("baseRpy"))
            self.rtm.connectPorts(stick.port("axes"),     self.arm.port("axes"))
            self.rtm.connectPorts(stick.port("buttons"),  self.arm.port("buttons"))
            self.rtm.connectPorts(arm.port("qRef"),       self.sh.port("qIn"))

    def init(self,robotname):
        print(self.configurator_name + "finding RTCManager and RobotHardware")
        self.waitForRTCManagerAndRobotHardware(robotname)
        self.createComps()
        self.connectComps()
        self.activateComps()

    def set_st_parameter(self):
        print >> sys.stderr, "2. setParameter"
        stp_org = self.st_svc.getParameter()
        # for eefm
        tmp_leg_inside_margin = 0.055
        tmp_leg_outside_margin= 0.075
        tmp_leg_front_margin  = 0.13
        tmp_leg_rear_margin   = 0.1
        rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
        lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp_org.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [lleg_vertices, rleg_vertices, larm_vertices, rarm_vertices])
        stp_org.eefm_leg_inside_margin = tmp_leg_inside_margin
        stp_org.eefm_leg_outside_margin = tmp_leg_outside_margin
        stp_org.eefm_leg_front_margin = tmp_leg_front_margin
        stp_org.eefm_leg_rear_margin=tmp_leg_rear_margin
        # stp_org.eefm_k1 = [-1.2301343, -1.2301343] #JVRCTSML
        # stp_org.eefm_k2 = [-0.3565158, -0.3565158]
        # stp_org.eefm_k3 = [-0.2219826, -0.2219826]
        stp_org.eefm_k1 = [-1.226668,  -1.226668] # JVRCTSML-bush
        stp_org.eefm_k2 = [-0.3515684, -0.3515684]
        stp_org.eefm_k3 = [-0.2204348, -0.2204348]
        stp_org.eefm_rot_damping_gain = [[20*1.6*1.5, 20*1.6*1.5, 1e5]]*4
        stp_org.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.0*1.5]]*4
        stp_org.eefm_swing_rot_damping_gain = stp_org.eefm_rot_damping_gain[0]
        stp_org.eefm_swing_pos_damping_gain = stp_org.eefm_pos_damping_gain[0]
        stp_org.eefm_use_swing_damping = False
        #stp_org.eefm_cogvel_cutoff_freq = 5
        #stp_org.foot_origin_offset = [[0.0,0.0,-0.1]]*2
        stp_org.st_algorithm = OpenHRP.StabilizerService.EEFMQPCOP
        #stp_org.st_algorithm = OpenHRP.StabilizerService.EEFM
        #stp_org.st_algorithm = OpenHRP.StabilizerService.EEFMQP
        self.st_svc.setParameter(stp_org)
        print >> sys.stderr, "2. setParameter finish"

    def go_halfsit(self):
        #raw_input('>>halfsitting? ')
        self.wpg_svc.testMove()
        self.wpg_svc.start()

    def stepping(self):
        #raw_input('>>stepping? ')
        self.wpg_svc.stepping()

    def set_pos_damping_gain(self, x, y, z):
        stp = self.st_svc.getParameter()
        stp.eefm_pos_damping_gain = [[3500*50*x, 3500*50*y, 3500*1.0*1.5*z]]*4
        self.st_svc.setParameter(stp)

    def set_pos_time_const_support(self, x):
        stp = self.st_svc.getParameter()
        stp.eefm_pos_time_const_support = [[x, x, x]]*4
        stp.eefm_pos_time_const_swing = x
        self.st_svc.setParameter(stp)

    def set_rot_time_const(self, x):
        stp = self.st_svc.getParameter()
        stp.eefm_rot_time_const = [[x, x, x]]*4
        self.st_svc.setParameter(stp)

    def set_rot_damping_gain(self, rgain, pgain, ygain):
        stp = self.st_svc.getParameter()
        stp.eefm_rot_damping_gain = [[rgain, pgain, 1e5*ygain]]*4
        self.st_svc.setParameter(stp)

    def set_eefm_body_params(self, gain, time_const):
        stp = self.st_svc.getParameter()
        stp.eefm_body_attitude_control_gain = [gain, gain]
        stp.eefm_body_attitude_control_time_const = [time_const, time_const]
        self.st_svc.setParameter(stp)

    def set_fsensor_cutoffFreq(self, f):
        stp = self.st_svc.getParameter()
        stp.fsensor_cutoff_freq = f
        self.st_svc.setParameter(stp)

    def set_vel(self, x, y, th):
        self.wpg_svc.setObjectV(x, y, 0, 0, 0, th)
#############################################    
def main():
    global hcf
    print "start"
    hcf = JVRC()
    hcf.init("JVRC-TSML")
    hcf.set_st_parameter()
    hcf.go_halfsit()
    hcf.set_pos_damping_gain(1,1,400)
    hcf.set_pos_time_const_support(0.02)
    #bush
    # hcf.set_rot_damping_gain(25, 25, 1)
    # hcf.set_rot_time_const(0.02)
    #hcf.set_rot_damping_gain(1000, 1000, 1)
    # hcf.set_rot_damping_gain(100, 100, 1)
    # hcf.set_rot_time_const(0.01)
    # hcf.set_eefm_body_params(0, 1e5)

    hcf.set_pos_damping_gain(1e9,1e9,11400)
    hcf.set_pos_time_const_support(0.02)
    hcf.set_rot_damping_gain(2250, 2250, 1e9)
    hcf.set_rot_time_const(0.02)
    hcf.set_eefm_body_params(0.07, 0.04)

    hcf.set_fsensor_cutoffFreq(5)
    #raw_input('>>start ST? ')
    hcf.st_svc.startStabilizer()
    hcf.stepping()
    
if __name__ == '__main__':
    main()
    #raw_input('>>end? ')




