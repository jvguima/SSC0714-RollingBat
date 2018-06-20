if (sim_call_type==sim.syscb_init) then
    laserHandle=sim.getObjectHandle("LaserScannerLaser_2D")
    jointHandle=sim.getObjectHandle("LaserScannerJoint_2D")
    graphHandle=sim.getObjectHandle("LaserScannerGraph_2D")
    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objName=sim.getObjectName(modelHandle)
    communicationTube=sim.tubeOpen(0,objName..'_2D_SCANNER_DATA',1)
end

if (sim_call_type==sim.syscb_cleanup) then
    sim.resetGraph(graphHandle)
end

if (sim_call_type==sim.syscb_sensing) then
    scanningAngle=tonumber(sim.getScriptSimulationParameter(sim.handle_self,"scanningAngle"))
    if (scanningAngle==nil) then
        scanningAngle=180
        sim.setScriptSimulationParameter(sim.handle_self,"scanningAngle",scanningAngle)
    end
    if (scanningAngle<5) then
        scanningAngle=5
    end
    if (scanningAngle>180) then
        scanningAngle=180
    end
    scanningDensity=tonumber(sim.getScriptSimulationParameter(sim.handle_self,"scanningDensity"))
    if (scanningDensity==nil) then
        scanningDensity=2
        sim.setScriptSimulationParameter(sim.handle_self,"scanningDensity",scanningDensity)
    end
    if (scanningDensity<0.1) then
        scanningDensity=0.1
    end
    if (scanningDensity>5) then
        scanningDensity=5
    end


    sim.resetGraph(graphHandle)
    numberOfPoints=scanningAngle*scanningDensity+1
    p=-scanningAngle*math.pi/360
    stepSize=math.pi/(scanningDensity*180)
    points={}
    modelInverseMatrix=simGetInvertedMatrix(sim.getObjectMatrix(modelHandle,-1))

    PilarsDetected=0;

    --Itera nos pontos de Laser encontrados
    for i=1,numberOfPoints+1,1 do

        sim.setJointPosition(jointHandle,p)
        p=p+stepSize
        --pt é: table of 3 numbers indicating the relative coordinates of the detected point if result is
        result,dist,pt=sim.handleProximitySensor(laserHandle) -- pt is RELATIVE to te rotating laser beam!
        angle = p*180/3.1415935;


        --Laser Detectou algo
        if result>0 then

            -- We put the RELATIVE coordinate of that point into the table that we will return:
            m=sim.getObjectMatrix(laserHandle,-1)
            pt=sim.multiplyVector(m,pt)
            pt=sim.multiplyVector(modelInverseMatrix,pt) -- after this instruction, pt will be relative to the model base!


            --[[
            Se i>0 e se i-1 detectou algo
                Ve a diferenca de distancia relativa entre ambos
             --]]
            if i>0 then
                prevAngle = points [(i-1)*2-1]
                prevDist = points [(i-1)*2]
            end


            table.insert(points,i*2-1,angle)
            table.insert(points,i*2,dist)


            --[[table.insert(points,pt[1])
            table.insert(points,pt[2])
            table.insert(points,pt[3])--]]

            Msg=""
            if(prevDist~=nil and prevAngle~=nil) then
                Msg=string.format(">>>: %.2f ; %.2f ; %.2f | Angle: %.2f - Dist: %.4f | PrevA: %.2f PrevD %.2f",pt[1], pt[2], pt[3],angle,dist, prevAngle, prevDist)
            else
                Msg=string.format(">>>: %.2f ; %.2f ; %.2f | Angle: %.2f - Dist: %.4f",pt[1], pt[2], pt[3],angle,dist)
            end
            sim.addStatusbarMessage(Msg)
        end
        sim.handleGraph(graphHandle,0.0)
    end



    TotPoints=table.getn(points)
    --Msg=string.format("First: %.4f - %.4f - %.4f ### Last: %.4f - %.4f - %.4f",points[1],points[2],points[3],points[TotPoints-2],points[TotPoints-1],points[TotPoints])
    sim.addStatusbarMessage("--------------------------------------------------")

    -- Now send the data:
    if #points>0 then
        sim.tubeWrite(communicationTube,sim.packFloatTable(points))
    end

    -- To read the data from another script, use following instructions (in that other script):
    --
    -- INITIALIZATION PHASE:
    -- laserScannerHandle=sim.getObjectHandle("LaserScanner_2D")
    -- laserScannerObjectName=sim.getObjectName(laserScannerHandle) -- is not necessarily "LaserScanner_2D"!!!
    -- communicationTube=sim.tubeOpen(0,laserScannerObjectName..'_2D_SCANNER_DATA',1)
    --
    -- TO READ THE DATA:
    -- data=sim.tubeRead(communicationTube)
    -- if (data) then
    --     laserDetectedPoints=sim.unpackFloatTable(data)
    -- end
    --
    -- laserDetectedPoints is RELATIVE to the model base!
end
