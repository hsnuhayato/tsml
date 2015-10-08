import os
from user import *

funcList = [
    #["stop-rt-system", deactivateComps],
    " ",
    "seq play",
    goInital,
    goHalfSitting,
    " ",
    ["move base pos rel", movePosRel, "io", "0 0 -0.1"],
    " ",
    ["setJointAngle", setJointAngle, "io", "NECK_Y 30 3.0"],
    " ",
    " ",
    "camera",
    searchOn,
    searchOff,
    showQrDataList,
    " ",
    " ",
    "util",
    showRobotState,
    ["log start", logRobotState],
    #["save data", cameraSave, "io", "D1_O1-2_01.csv"],
    "\n",
    "wu walk",
    wuHalfSitting,
    wuStart,
    wuStepping,
    #["setv5", setObjectV, "io", "5 0 0 0 0 0"],
    #setObjectZero,
    " ",
    wuStop,
    #wuOmniWalkSwitch,
    " ",
    " ",
    "st",
    stOn,
    stOff,
    " ",
    " ",
    "pcl",
    pclStart,
    pclMatchingMap,
    pclUpdateMap,
    " ",
    pclClearWorld,
    pclClearCloud,
    #pclCangeMode,
    " ",
    #["save pcl", pclSave, "io", "D1_3D_01.csv"],
    " ",
    "arm",
    #rarmActive,
    #larmActive,
    ["set arm velocity", setArmVelocity, "io", "0.04 0.2 0.2"],
    " ",
    "\n",
    #" ",
    "gamepad",
    ["walk", gamepadWalk],
    ["PCL",  gamepadPcl],
    ["rarm", gamepadArmR],
    ["larm", gamepadArmL],
    " ",
    " ",
    "stairs",
    pclGround,
    " ",
    ["detect landing point R", pclDetectLandingPointR],
    ["detect landing point L", pclDetectLandingPointL],
    " ",
    ["set foot pos R", pclSetFootPosR],
    ["set foot pos L", pclSetFootPosL],
    " ",
    " ",
    "JVRC",
    ["save QR data", cameraSave, "io", "D1_O1-2_01.csv"],
    " ",
    ["save pcd data", pclSave, "io", "D1_3D_01.csv"],
    " ",
]
