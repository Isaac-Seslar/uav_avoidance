# Generated with SMOP  0.41-beta
from libsmop import *
# GenerateTrajectoryPoint.m

    # FUNCTION: GenerateTrajectoryPoint
# DESCRIPTION: Calculates the trajectory at a point in time
# INPUTS: tPoint - desired point in time for trajectory point
#         type - Type of trajectory generation desired.
# OUTPUTS: x - position at time tPoint,
#          v - velocity at time tPoint
#          a - accelertion at time tPoint
# AUTHOR: Michal Rittikaidachar
# REVISIION HISTORY:`REV A - 5/19/20
# NOTES:Assumes the trajectory starts at time 0
#      Only Miniumum Jerk Trajectorty is implemented.
#      Currenlty loads set points from excel file in init function should
#      pass it a matrix of set point pair and time.
#      Time vector for set points defined in initFunction
    
    
@function
def GenerateTrajectoryPoint(t=None,*args,**kwargs):
    varargin = GenerateTrajectoryPoint.varargin
    nargin = GenerateTrajectoryPoint.nargin

    
    
    if isempty(jointPositions):
        disp(' Initializing trajectory Gen')
        jointPositions,tVec=initGenerator(nargout=2)
# GenerateTrajectoryPoint.m:22
    else:
        #disp( ' Trajectory Gen Already Initialized');
        pass
    
    
    lenT=length(t)
# GenerateTrajectoryPoint.m:27
    if lenT == 1:
        idx=discretize(t,tVec)
# GenerateTrajectoryPoint.m:29
        if isnan(idx):
            xVec=dot((t < tVec(1)),jointPositions(1,arange()).T) + dot((t > tVec(end())),jointPositions(end(),arange()).T)
# GenerateTrajectoryPoint.m:31
            vVec=0
# GenerateTrajectoryPoint.m:32
            aVec=0
# GenerateTrajectoryPoint.m:33
        else:
            D=tVec(idx + 1) - tVec(idx)
# GenerateTrajectoryPoint.m:35
            T=(t - tVec(idx)) / D
# GenerateTrajectoryPoint.m:36
            xVec,vVec,aVec=min_jerk(jointPositions(idx,arange()).T,jointPositions(idx + 1,arange()).T,T,D,nargout=3)
# GenerateTrajectoryPoint.m:37
    else:
        xVec=zeros(size(jointPositions,2),lenT)
# GenerateTrajectoryPoint.m:40
        vVec=zeros(size(jointPositions,2),lenT)
# GenerateTrajectoryPoint.m:41
        aVec=zeros(size(jointPositions,2),lenT)
# GenerateTrajectoryPoint.m:42
        index=discretize(t,tVec)
# GenerateTrajectoryPoint.m:44
        uindex=unique(index(logical_not(isnan(index))))
# GenerateTrajectoryPoint.m:45
        for ii in arange(1,length(uindex)).reshape(-1):
            idx=uindex(ii)
# GenerateTrajectoryPoint.m:47
            mask=index == idx
# GenerateTrajectoryPoint.m:48
            time=t(mask)
# GenerateTrajectoryPoint.m:49
            D=tVec(idx + 1) - tVec(idx)
# GenerateTrajectoryPoint.m:50
            T=(time - tVec(idx)) / D
# GenerateTrajectoryPoint.m:51
            x,v,a=min_jerk(jointPositions(idx,arange()).T,jointPositions(idx + 1,arange()).T,T,D,nargout=3)
# GenerateTrajectoryPoint.m:52
            xVec[arange(),mask]=x
# GenerateTrajectoryPoint.m:53
            vVec[arange(),mask]=v
# GenerateTrajectoryPoint.m:54
            aVec[arange(),mask]=a
# GenerateTrajectoryPoint.m:55
        if any(t < tVec(1)):
            xVec[arange(),t < tVec(1)]=jointPositions(1,arange()).T
# GenerateTrajectoryPoint.m:59
            vVec[arange(),t < tVec(1)]=0
# GenerateTrajectoryPoint.m:60
            aVec[arange(),t < tVec(1)]=0
# GenerateTrajectoryPoint.m:61
        if any(t > tVec(end())):
            xVec[arange(),t > tVec(end())]=jointPositions(end(),arange()).T
# GenerateTrajectoryPoint.m:64
            vVec[arange(),t > tVec(end())]=0
# GenerateTrajectoryPoint.m:65
            aVec[arange(),t > tVec(end())]=0
# GenerateTrajectoryPoint.m:66
    
    return xVec,vVec,aVec
    
if __name__ == '__main__':
    pass
    
    
@function
def initGenerator(*args,**kwargs):
    varargin = initGenerator.varargin
    nargin = initGenerator.nargin

    params,states0,tspan,absError,relError=wiper_dynamics.parameters(0,nargout=5)
# GenerateTrajectoryPoint.m:72
    rA=params(5)
# GenerateTrajectoryPoint.m:73
    rC=params(7)
# GenerateTrajectoryPoint.m:74
    data=xlsread('+wiper_dynamics/AssemblyProcedureUpdated.xlsx')
# GenerateTrajectoryPoint.m:75
    tVec=data(arange(),14)
# GenerateTrajectoryPoint.m:76
    jointPositions=data(arange(),concat([arange(6,8),arange(1,2)]))
# GenerateTrajectoryPoint.m:77
    jointPositions[arange(),1]=dot(jointPositions(arange(),1),(rA / rC))
# GenerateTrajectoryPoint.m:78
    jointPositions=multiply(jointPositions,concat([pi / 180,pi / 180,1 / 1000,pi / 180,1 / 1000]))
# GenerateTrajectoryPoint.m:79
    return jointPositions,tVec
    
if __name__ == '__main__':
    pass
    
    
@function
def min_jerk(xi=None,xf=None,T=None,D=None,*args,**kwargs):
    varargin = min_jerk.varargin
    nargin = min_jerk.nargin

    vi=0
# GenerateTrajectoryPoint.m:84
    ai=0
# GenerateTrajectoryPoint.m:85
    a0=copy(xi)
# GenerateTrajectoryPoint.m:86
    a1=multiply(D,vi)
# GenerateTrajectoryPoint.m:87
    a2=(multiply(D,ai)) / 2
# GenerateTrajectoryPoint.m:88
    a3=multiply(- (dot(3.0,D ** 2.0) / 2),ai) - multiply(dot(6.0,D),vi) + dot(10.0,(xf - xi))
# GenerateTrajectoryPoint.m:89
    a4=multiply((dot(3.0,D ** 2.0) / 2),ai) + multiply(dot(8.0,D),vi) - dot(15.0,(xf - xi))
# GenerateTrajectoryPoint.m:90
    a5=multiply(- (D ** 2.0 / 2),ai) - multiply(dot(3.0,D),vi) + dot(6.0,(xf - xi))
# GenerateTrajectoryPoint.m:91
    x=(a0 + multiply(a1,T) + multiply(a2,T ** 2) + multiply(a3,T ** 3) + multiply(a4,T ** 4) + multiply(a5,T ** 5))
# GenerateTrajectoryPoint.m:92
    v=((a1 + multiply(dot(2,a2),T) + multiply(dot(3,a3),T ** 2) + multiply(dot(4,a4),T ** 3) + multiply(dot(5,a5),T ** 4)) / D)
# GenerateTrajectoryPoint.m:93
    a=((dot(2,a2) + multiply(dot(6,a3),T) + multiply(dot(12,a4),T ** 2) + multiply(dot(20,a5),T ** 3)) / D ** 2)
# GenerateTrajectoryPoint.m:94
    return x,v,a
    
if __name__ == '__main__':
    pass
    