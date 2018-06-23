if (sim_call_type==sim.syscb_init) then
    laserHandle=sim.getObjectHandle("LaserScannerLaser_2D")
    jointHandle=sim.getObjectHandle("LaserScannerJoint_2D")
    graphHandle=sim.getObjectHandle("LaserScannerGraph_2D")
    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    --objName=sim.getObjectName(modelHandle)
    --communicationTube=sim.tubeOpen(0,objName..'_2D_SCANNER_DATA',1)
    communicationTube=sim.tubeOpen(0,'NEXT_ROTATION',1)

    TURN_LEFT=-1
    MOVE_FORWARD=0
    TURN_RIGHT=1

end

if (sim_call_type==sim.syscb_cleanup) then
    sim.resetGraph(graphHandle)
end

if (sim_call_type==sim.syscb_sensing) then

    -- PEGA VARIAVEIS DE PARAMETROS =========================================================
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

    PILLAR_SEPARATION_THRESHOLD = tonumber(sim.getScriptSimulationParameter(sim.handle_self,"pillarSeparationThreshold"))
    if (PILLAR_SEPARATION_THRESHOLD==nil) then
        PILLAR_SEPARATION_THRESHOLD=1
        sim.setScriptSimulationParameter(sim.handle_self,"pillarSeparationThreshold",PILLAR_SEPARATION_THRESHOLD)
    end

    sim.resetGraph(graphHandle)

    numberOfPoints=scanningAngle*scanningDensity+1
    p=-scanningAngle*math.pi/360
    stepSize=math.pi/(scanningDensity*180)
    points={}
    modelInverseMatrix=simGetInvertedMatrix(sim.getObjectMatrix(modelHandle,-1))

    --Numedo do primeiro feixe a detectar o pilar atual
    currentPillarFirstDetection=-1
    --Numero do ultimo feixe a detectar o pilar
    currentPillarLastDetection=-1
    numberOfPillars=0
    --Tabela que armazena as coordenadas X e Y de cada pilar
    pillarCoordinates={}

    --======================================================================================

    --Itera nos pontos de Laser emitidos
    for i=1,numberOfPoints+1,1 do

        sim.setJointPosition(jointHandle,p)
        p=p+stepSize
        --pt é: table of 3 numbers indicating the relative coordinates of the detected point if result is
        result,dist,pt=sim.handleProximitySensor(laserHandle) -- pt is RELATIVE to te rotating laser beam!
        angle = p*180/3.1415935;

        --LASER DETECTOU ALGO
        if result>0 then

            -- We put the RELATIVE coordinate of that point into the table that we will return:
            m=sim.getObjectMatrix(laserHandle,-1)
            pt=sim.multiplyVector(m,pt)
            pt=sim.multiplyVector(modelInverseMatrix,pt) -- after this instruction, pt will be relative to the model base!


            --[[
            Pega a distancia e angulo do feixe anterior
            Se o feixe anterior (i-1) nao detectou nada, tentar acessar a posicao (i-1)
            na table points resultara em NULO -> Isso e o indicativo de que houve um gap entre
            os pilares
             --]]
            if i>0 then
                prevAngle = points [(i-1)*4-3]
                prevDist = points [(i-1)*4-2]
                prevX = points[(i-1)*4-1]
                prevY = points[(i-1)*4]
            end

            --Insere as informacoes do feixe atual
            table.insert(points,i*4-3,angle)    --Angulo
            table.insert(points,i*4-2,dist)     --Distancia
            table.insert(points,i*4-1,pt[2])    --X
            table.insert(points,i*4,pt[3])      --Y


            Msg=""


            if(prevDist~=nil) then
                --A diferenca de distancia eh maior que o limiar -> esta detectado outro pilar
                if math.abs(dist-prevDist) > PILLAR_SEPARATION_THRESHOLD then

                    avgX, avgY = calculatePillarAvgCoord(points,currentPillarFirstDetection, currentPillarLastDetection)
                    pillarCoordinates[numberOfPillars*2-1]=avgX
                    pillarCoordinates[numberOfPillars*2]=avgY
                    --Msg=string.format("<<>> AvgX:%.2f | AvgY:%.2f",avgX,avgY)
                    --sim.addStatusbarMessage(Msg)

                    currentPillarFirstDetection = i
                    numberOfPillars = numberOfPillars+1
                --Esta detectando o mesmo pilar
                else
                    --Atualiza a ultima detecao do pilar atualmente analisado
                    currentPillarLastDetection = i
                    --Msg=string.format(">>>: X %.2f ; Y %.2f | Angle: %.2f - Dist: %.4f | PrevA: %.2f PrevD %.2f PrevX %.2f PrevY %.2f", pt[2], pt[3],angle,dist, prevAngle, prevDist, prevX, prevY)
                    --sim.addStatusbarMessage(Msg)
                end


            --DETECTOU UM NOVO PILAR
            else
                numberOfPillars = numberOfPillars+1
                currentPillarFirstDetection = i
            end

        --NAO detectou nada
        elseif result==0 then
            --Se estava detectando algo na ultima iteracao havia um pilar la
            if( i-1 == currentPillarLastDetection and currentPillarFirstDetection > 0 and
                currentPillarLastDetection > 0 and currentPillarFirstDetection < currentPillarLastDetection) then

                avgX, avgY = calculatePillarAvgCoord(points,currentPillarFirstDetection, currentPillarLastDetection)
                pillarCoordinates[numberOfPillars*2-1]=avgX
                pillarCoordinates[numberOfPillars*2]=avgY
                --Msg=string.format("<<>> AvgX:%.2f | AvgY:%.2f",avgX,avgY)
                --sim.addStatusbarMessage(Msg)
            end
        end

        sim.handleGraph(graphHandle,0.0)
    end

    nextRotation = determineNextRoation(pillarCoordinates)
    if(nextRotation~=nil)then
        sim.addStatusbarMessage(string.format(">>NEXT ROTATION %f",nextRotation))
        sim.tubeWrite(communicationTube, nextRotation, 1)
    end
    sim.addStatusbarMessage("--------------------------------------------------")

    --[[ Now send the data:
    if #points>0 then
        sim.tubeWrite(communicationTube,sim.packFloatTable(points))
    end--]]

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



--[[
Funçao que calcula a posicao media de um pilar iterando pelos X e Y relativos
dos feixes que o detectaram.
PARAM: points -> tabela de pontos que contem para cada feixe o angulo, distancia, relX e relY
PARAM: startIndex -> indice de inicio para o calculo da media
PARAM: endIndex -> indice de fim para o calculo da media

RETURN: Coordenadas medias X e Y
--]]
function calculatePillarAvgCoord(points, startIndex, endIndex)
    sumX=0
    sumY=0
    --[[for i=startIndex, endIndex, 1 do
        sumX = sumX + points[i*4-1]
        sumY = sumY + points[i*4]
    end

    return sumX/(endIndex-startIndex), sumY/(endIndex-startIndex)--]]
    return (points[endIndex*4-1]+points[startIndex*4-1])/2, (points[endIndex*4]+points[startIndex*4])/2

end



--[[
Funçao que determina a proxima rotacao a ser feita no cruzamento baseado nas coordenadas
relativas X e Y de cada um dos vetores detectados. Se diferente do que 3 pilares forem detectados
Nenhuma rotacao eh realizada

PARAM: pillarCoordinates -> Table de tamanho 3*2, que armazena as coordenadas X e Y de cada um dos 3 pilares

RETURN: inteiro contendo o codigo para o proximo movimento
--]]
function determineNextRoation(pillarCoordinates)
    if table.getn(pillarCoordinates) ~= 6 then
        --sim.addStatusbarMessage("FORWARD")
        return MOVE_FORWARD
    else
        pillarsToTheRight=0
        for i=1,3,1 do
            if pillarCoordinates[2*i] < 0 then
                pillarsToTheRight = pillarsToTheRight+1
            end
        end

        if pillarsToTheRight == 2 then
            --sim.addStatusbarMessage("LEFT")
            return TURN_LEFT
        elseif pillarsToTheRight==1 then
            --sim.addStatusbarMessage("RIGHT")
            return TURN_RIGHT
        end
    end
end
