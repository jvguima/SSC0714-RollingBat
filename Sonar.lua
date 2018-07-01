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
    v0=2
    minDist=0.5
        state=0
    
    --Braitenberg variables
    braitenbergDetect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}

    maxDetectionDist=0.4
    braitenbergNoDetectionDist = 0.55

    sensorObstacleThreshold = 0.5

end 

if (sim_call_type==sim.syscb_cleanup) then 
 
end 

if (sim_call_type==sim.syscb_actuation) then 
    for i=1,16,1 do
        res,dist=sim.readProximitySensor(usensors[i])
        if (res>0) and (dist<noDetectionDist) then
            detect[i]=dist
            if (res>0) and (dist<=sensorObstacleThreshold) then
                braitenbergDetect[i]=1-((dist-maxDetectionDist)/(braitenbergNoDetectionDist-maxDetectionDist))
            else
                braitenbergDetect[i]=0
            end
        else
            detect[i]=0
            braitenbergDetect[i]=0
        end
    end
    buffer=sim.tubeRead(communicationTube)
    if (buffer~=nil) and (detect[7]>0) and (detect[7]<0.5) and (estadoDeOperacao~="Faz a curva") then
        turnDirection= tonumber(buffer)
    end

    Msg=string.format("Dist7: %.4f Dist10: %.4f ### Dist9: %.4f Turn: %d state: %d \n",detect[7],detect[10],detect[9],turnDirection, state)
    sim.addStatusbarMessage(Msg .. estadoDeOperacao)
    
    --SEGUE PAREDE======================================================================
    if (estadoDeOperacao=="Segue a parede") then
    
        vLeft=v0
        vRight=v0
        
        --OBSTACULO DETECTADO
        if(detect[4]>0 and detect[4]<=sensorObstacleThreshold and detect[5]>0 and detect[5]<=sensorObstacleThreshold) then 
            estadoDeOperacao="Evita obstaculo"
        end
        if (detect[8]>0.5) and (detect[9]>0.5) then
            state=0
            estadoDeOperacao="Volta para parede"
        end
       
        if(detect[8]>detect[9])then
            vRight=vRight-0.35
        elseif (detect[8]<detect[9]) then
            vLeft=vLeft-0.35
        end
        if (detect[8]==0) or (detect[1]==0) then
            estadoDeOperacao="Faz a curva"
        end
        
    end
    
    --VOLTA A PAREDE======================================================================
    if (estadoDeOperacao=="Volta para parede") then
        vLeft=v0
        vRight=0
  
        if(detect[4]~=0 and detect[5]~=0 or state==1) then
            state=1
            vRight=v0
            if(detect[4]<minDist) or (detect[5]<minDist) or (detect[6]<minDist) or (detect[7]<minDist) then
                vLeft=0
                if(detect[9]-detect[8]<0.1 and detect[8]<minDist) then
                    turnTime=0
                    estadoDeOperacao="Segue a parede"
                end
            end
        end

    end
    
    --CURVA======================================================================
    if (estadoDeOperacao=="Faz a curva") then
        --if(detect[9]==0) then
            if(detect[4]>0 and detect[4]<=sensorObstacleThreshold and detect[5]>0 and detect[5]<=sensorObstacleThreshold) then 
                estadoDeOperacao="Evita obstaculo"
            end
            
            --Reto
            if (turnDirection==0) then
                vLeft=v0
                vright=v0
                if (detect[9]>0) then
                    estadoDeOperacao="Segue a parede"
                end
            end
            --Direita
            if (turnDirection==1) then
                vLeft=2.1
                vRight=1.0
           
                if (detect[7]~=0.0) and (detect[7]<=0.45 ) then
                    vRight=0
                    vLeft=0
                    estadoDeOperacao="Segue a parede"
                    if (detect[8]>0)then
                        estadoDeOperacao="Segue a parede"
                    end
                end
            end
            --Esquerda
            if (turnDirection==-1) then
                vLeft=2.2
                vRight=2.9
                if (detect[9]>0 and detect[9]<0.5) then
                   estadoDeOperacao="Segue a parede"
                end
            end
        --end
    end
    
    --EVITA OBSTACULOS======================================================================
    if (estadoDeOperacao=="Evita obstaculo") then

        vLeft=v0
        vRight=v0
        
        if(detect[5]==0 and detect[6]==0) then 
            for i=1,6,1 do
                vLeft=vLeft+braitenbergL[i]*braitenbergDetect[i]
                vRight=vRight+10*braitenbergR[i]*braitenbergDetect[i]
            end
        elseif(detect[4]==0 and detect[5]==0 and (detect[10]>0.6)) then
            estadoDeOperacao="Segue a parede"
        else
            for i=1,6,1 do
                vLeft=vLeft+2*braitenbergL[i]*braitenbergDetect[i]
                vRight=vRight+braitenbergR[i]*braitenbergDetect[i]
            end
        end
    end
    
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)
end 