-- This is a very simple EXAMPLE navigation program, which avoids obstacles using the Braitenberg algorithm

if (sim_call_type==sim.syscb_init) then 
    usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,16,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    noDetectionDist=1.45
    estadoDeOperacao="Segue a parede"
    turnTime=0
    turnDirection=1
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    --braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    --braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    v0=2
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

    sim.addStatusbarMessage(estadoDeOperacao)

    if (estadoDeOperacao=="Segue a parede") then
    
        vLeft=v0
        vRight=v0
  
        --Msg=string.format("Dist8: %.4f ### Dist9: %.4f",detect[8],detect[9])
        --sim.addStatusbarMessage(Msg)
        
        if (detect[9]==0) or (detect[16]==0) then
            estadoDeOperacao="Faz a curva"
        end
        
        if (detect[8]>0.5) and (detect[9]>0.5) then
            estadoDeOperacao="Volta para parede"
        end
       
        if(detect[8]>detect[9])then
            vRight=vRight-0.2
        else if (detect[8]<detect[9]) then
            vLeft=vLeft-0.2
        end
        end
        
    end

    if (estadoDeOperacao=="Volta para parede") then
    
        vLeft=v0
        vRight=0
  
        if (turnTime<55) then
            turnTime=turnTime+1
        else
            vRight=v0
            if(detect[4]<0.5) or (detect[5]<0.5) or (detect[6]<0.5) or (detect[7]<0.5) then
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
        end
        if (turnDirection==1) then
            vLeft=3.4
            vRight=1.4
        end
        if(detect[9]>0)then
            while turnTime < 200 do
                turnTime=turnTime+1
            end
            turnTime=0
            estadoDeOperacao="Segue a parede"
        end
    end
    
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)
end 