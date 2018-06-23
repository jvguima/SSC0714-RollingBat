-- This is a very simple EXAMPLE navigation program, which avoids obstacles using the Braitenberg algorithm

if (sim_call_type==sim.syscb_init) then 
    usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,16,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    laserScannerHandle=sim.getObjectHandle("LaserScanner_2D")
    laserScannerObjectName=sim.getObjectName(laserScannerHandle) -- is not necessarily "LaserScanner_2D"!!!
    communicationTube=sim.tubeOpen(0,'NEXT_ROTATION',1)
    noDetectionDist=1.45
    estadoDeOperacao="Segue a parede"
    turnTime=0
    turnDirection=0
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    --braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    --braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    v0=2
    minDist=0.5
end 

if (sim_call_type==sim.syscb_cleanup) then 
 
end 

if (sim_call_type==sim.syscb_actuation) then 
    for i=1,16,1 do
        res,dist=sim.readProximitySensor(usensors[i])
        if (res>0) and (dist<noDetectionDist) then
            detect[i]=dist
        else
            detect[i]=0
        end
    end
    buffer=sim.tubeRead(communicationTube)
    if (buffer~=nil) and (detect[7]>0) then
        turnDirection= tonumber(buffer)
    end

    Msg=string.format("Dist8: %.4f ### Dist9: %.4f Turn: %d",detect[8],detect[9],turnDirection)
    sim.addStatusbarMessage(Msg .. estadoDeOperacao)
    

    if (estadoDeOperacao=="Segue a parede") then
    
        vLeft=v0
        vRight=v0
        
        if (detect[8]>0.5) and (detect[9]>0.5) then
            estadoDeOperacao="Volta para parede"
        end
       
        if(detect[8]>detect[9])then
            vRight=vRight-0.2
        elseif (detect[8]<detect[9]) then
            vLeft=vLeft-0.2
        end
        if (detect[9]==0) or (detect[16]==0) then
            estadoDeOperacao="Faz a curva"
        end
        
    end

    if (estadoDeOperacao=="Volta para parede") then
    
        vLeft=v0
        vRight=0
  
        if (turnTime<55) then
            turnTime=turnTime+1
        else
            vRight=v0
            if(detect[4]<minDist) or (detect[5]<minDist) or (detect[6]<minDist) or (detect[7]<minDist) then
                vLeft=0
                if(turnTime<110) then
                    turnTime=turnTime+1
                else
                    turnTime=0
                    estadoDeOperacao="Segue a parede"
                end
            end
        end

    end

    if (estadoDeOperacao=="Faz a curva") then
        if (turnDirection==0) then
            vLeft=v0
            vright=v0
            if (detect[9]>0) then
                estadoDeOperacao="Segue a parede"
            end
        elseif (turnDirection==1) then
            vLeft=3.4
            vRight=2.0
            if (detect[16]>0) then
                estadoDeOperacao="Segue a parede"
            end
        elseif (turnDirection==-1) then
            vLeft=2.8
            vRight=3.4
            if (detect[9]>0) then
               estadoDeOperacao="Segue a parede"
            end
        end
    end
    
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)
end 