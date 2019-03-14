#!/home/player/tsml/bin/rtm_python

try:
    from hrpsys_config import *
    import OpenHRP
    rtm.nsport=2809
except:
    print "import without hrpsys"
    import rtm
    from rtm import *
    from OpenHRP import *
    import waitInput
    from waitInput import *
    import socket
    import time

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
        raw_input('>>halfsitting? ')
        self.wpg_svc.testMove()

#############################################    
def main():
    global hcf
    print "start"
    hcf = JVRC()
    hcf.init ("JVRC-TSML")
    

if __name__ == '__main__':
    main()
    raw_input('>>end? ')




