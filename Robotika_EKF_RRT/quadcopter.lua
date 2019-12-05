function state2hover(msg)
    state2hover_ = msg.data
end

function state2landing(msg)
    state2landing_ = msg.data    
end

function sysCall_init()
    pos_pub = simROS.advertise("/quad_pos", "geometry_msgs/Vector3")
    simROS.publisherTreatUInt8ArrayAsString(pos_pub)
    
    targetObj=sim.getObjectHandle('Quadricopter_target')
    sim.setObjectParent(targetObj,-1,true)
    
    baseHandle=sim.getObjectHandle('Quadricopter_base')
    awalPos = sim.getObjectPosition(baseHandle, -1)
    
    TAKE_OFF = 1
    MANEUVER = 2
    HOVERING = 3
    LANDING = 4

-- 8.8601e-01 (atas)
   NodeLanding =  1.1101e-01 + 0.234 -- + tinggi UGV
   NodePos = {{-1.3778e+00, 1.0780e+00, 8.8601e-01}, --start untuk take-off
              {-9.7500e-01, -1.4750e+00, 8.8601e-01},
              {2.0750e+00, -1.5750e+00, 8.8601e-01},
              {2.9250e+00, 1.1750e+00, 8.8601e-01}}
    
    n_maneuver = 4
    state_man_now = 1
    state_man_next = 2
    
    stateNow = TAKE_OFF
    nextstate = MANEUVER
    Dx = NodePos[nextstate][1] - awalPos[1]
    Dy = NodePos[nextstate][2] - awalPos[2]
    Dz = NodePos[stateNow][3] - awalPos[3]
   
    stepX = 300.0
    stepY = 300.0
    stepZ = 20
    
    dx = Dx/stepX
    dy = Dy/stepY
    dz = Dz/stepZ
  
    cusPos = awalPos
    
    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    heli=sim.getObjectAssociatedWithScript(sim.handle_self)
    particlesTargetVelocities={0,0,0,0}

    pParam=2
    iParam=0
    dParam=0
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0

    prevEuler=0

    count = 0
    state2hover_ = false
    state2landing_ = false
end

function sysCall_actuation()
    s=sim.getObjectSizeFactor(baseHandle)
    pos=sim.getObjectPosition(baseHandle,-1)
    
    table_pos = {}
    table_pos["x"] = pos[1]
    table_pos["y"] = pos[2]
    table_pos["z"] = pos[3]
    simROS.publish(pos_pub, table_pos)
   
    -- Vertical control:
    targetPos=sim.getObjectPosition(targetObj,-1)
    pos=sim.getObjectPosition(baseHandle,-1)
    l=sim.getVelocity(heli)
    e=(targetPos[3]-pos[3])
    cumul=cumul+e
    pv=pParam*e
    thrust=5.335+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
    lastE=e

    posTargetNow=sim.getObjectPosition(targetObj,-1)
    if(stateNow == TAKE_OFF) and (nextstate == MANEUVER) then
        print("Dz: "..Dz)
        Dz = NodePos[stateNow][3] - posTargetNow[3]
        if (math.abs(Dz) < 0.01) then
            stateNow = MANEUVER
            nextstate = HOVERING
        else
            dz = Dz/stepZ
            cusPos[3] = cusPos[3] + dz
            sim.setObjectPosition(targetObj, -1, cusPos)
        end
    end
    -- Horizontal control:

    sp=sim.getObjectPosition(targetObj,baseHandle)
    m=sim.getObjectMatrix(baseHandle,-1)
    vx={1,0,0}
    vx=sim.multiplyVector(m,vx)
    vy={0,1,0}
    vy=sim.multiplyVector(m,vy)
    alphaE=(vy[3]-m[12])
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE=(vx[3]-m[12])
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    betaCorr=betaCorr-sp[1]*0.005-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]

    if(stateNow == MANEUVER) and (nextstate == HOVERING) then
        print("Dx: "..Dx..", Dy: "..Dy)
        Dx = NodePos[state_man_next][1] - posTargetNow[1]
        Dy = NodePos[state_man_next][2] - posTargetNow[2]
        if (math.abs(Dx) < 0.05) and (math.abs(Dy) < 0.05) then
            -- stateNow = HOVERING
            if(state_man_next < n_maneuver) then
               state_man_next = state_man_next + 1
            else
               state_man_next = 1
            end
            
            if(state_man_now < 3) then
               state_man_now = state_man_now + 1
            else
               state_man_now = 1
            end
        else
            if(math.abs(Dx) > 1.5) then
                stepX = 300.0
            elseif(math.abs(Dx) > 0.8) then
                stepX = 150.0
            else
                stepX = 80.0
            end

            if(math.abs(Dy) > 1.5) then
                stepY = 300.0
            elseif(math.abs(Dy) > 0.8) then
                stepY = 150.0
            else
                stepY = 80.0
            end
            
            dx = Dx/stepX
            dy = Dy/stepY
            
            cusPos[1] = cusPos[1] + dx
            cusPos[2] = cusPos[2] + dy
            sim.setObjectPosition(targetObj, -1, cusPos)
        end
    end

    sub_state2hover = simROS.subscribe('/state2hover_topic', 'std_msgs/Bool', 'state2hover')
    simROS.subscriberTreatUInt8ArrayAsString(sub_state2hover)
    
    sub_state2landing = simROS.subscribe('/state2landing_topic', 'std_msgs/Bool', 'state2landing')
    simROS.subscriberTreatUInt8ArrayAsString(sub_state2landing)
    
    if(state2landing_ == true) then
        state2hover_ = false
        stateNow = LANDING
    end
    
    if(state2hover_ == true) then
        stateNow = HOVERING
    end
    
    if(stateNow == HOVERING) then
--[[
        count = count+1
        if(count == 100) then
            stateNow = LANDING
        end
]]
    end
    
    if(stateNow == LANDING) then
        print("Dz: "..Dz)
        Dz = NodeLanding - posTargetNow[3]
        if (math.abs(Dz) < 0.01) then
            thrust = 0.0
        else
            dz = Dz/stepZ
            cusPos[3] = cusPos[3] + dz
            sim.setObjectPosition(targetObj, -1, cusPos)
        end
    end

    -- Rotational control:
    euler=sim.getObjectOrientation(baseHandle,targetObj)
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3]

    -- Decide of the motor velocities:
    particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)
   
    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end    
end 

