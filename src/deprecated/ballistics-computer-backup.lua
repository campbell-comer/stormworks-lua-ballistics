---@diagnostic disable: undefined-global, lowercase-global
-- [Deprecated] Ballistics Computer (Backup)
--
-- Earlier version of the 3D ballistics computer. Superseded by
-- ballistics-computer-3d.lua which adds secant/bisection TOF
-- solving and improved lead calculation.

gN = input.getNumber
sN = output.setNumber
gB = input.getBool
sB = output.setBool
pgN = property.getNumber
pgB = property.getBool
sin = math.sin
cos = math.cos
atan = math.atan
acos = math.acos
asin = math.asin
log = math.log
exp = math.exp
abs = math.abs
pi = math.pi
pi2 = pi * 2

selection = pgN("Weapon Type")

--table that has the data for velocity, drag, and life time of all weapons
data = {
    {800, 0.025, 120},
    {1000, 0.02, 150},
    {1000, 0.01, 300},
    {900, 0.005, 600},
    {800, 0.002, 1500},
    {700, 0.001, 2400},
    {600, 0.0005, 2400}
}

del = {}

g = 30

altOffset = 0

--creates a new 3D vec
Vector = function(b, c, e)
    return {x = b, y = c, z = e}
end

function nilVec()
	return Vector(0,0,0)
end

deltaAccelRaw = nilVec()
deltaAccel = nilVec()

--gets rid of NaN values that could cause a bug in the script
function removeNaNfromX(x)
    return (x ~= x or abs(x) == math.huge) and 0 or x
end

function sign(x) 
	return x < 0 and -1 or 1
end

--clamp a number between 2 values
function clamp(a, b, c)
    return math.min(math.max(a, b), c)
end

function vecScale(A, B)
    return Vector(A.x * B.x, A.y * B.y, A.z * B.z)
end

function vecScaleN(A, n)
    return Vector(A.x * n, A.y * n, A.z * n)
end

--normalize a vec
function normalize(S)
    return divVecs(S, vecLength(S))
end

--cross product of two 3D vecs (generates a 3rd orthogonal vec)
function crossProd(a, b)
    return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
end

-- dot product of two 3D vecs
function dotProd(a, b)
    return a.x * b.x + a.y * b.y + a.z * b.z
end

-- spherical to cartesian coordinate conversion, with distance scalar if needed
function vecFromEuler(yawangle, pitchangle, distance)
     c = distance or 1
     plane = (cos(pitchangle) * c)
     return Vector(
         (sin(yawangle) * plane),
         (cos(yawangle) * plane),
         (sin(pitchangle) * c)
     )

end

-- Cartesian vector to spherical (yaw, pitch) angles
function vecToEuler(vec)
 	return Vector(
 		acos(normalize(Vector(vec.x, vec.y, 0)).y) * sign(vec.x),
 		asin(normalize(vec).z), 
 		0
 	)
end

-- Convert wind angle and speed to a velocity vector
function windToVec(angle,velocity)
	 wx = sin(angle) * velocity
	 wy = cos(angle) * velocity
	return Vector(wx,wy,0)
end

--transforms rotation vec from local to global coordinate system
function toGlobalVec(vec, relAxes)
    return addVecs(
        addVecs(vecScaleN(relAxes.r, vec.x), vecScaleN(relAxes.fwd, vec.y)),
        vecScaleN(relAxes.up, vec.z)
    )
end

--basic vector functions
function vecPowerN(a, n)
    return Vector(a.x ^ n, a.y ^ n, a.z ^ n)
end

function subVecs(A, B)
    return Vector(A.x - B.x, A.y - B.y, A.z - B.z)
end

function addVecs(A, B)
    return Vector(A.x + B.x, A.y + B.y, A.z + B.z)
end

function vecEquals(a)
    return Vector(a, a, a)
end

function divVecs(A, R)
    return Vector(A.x / R, A.y / R, A.z / R)
end

function vecLength(S)
    return (S.x * S.x + S.y * S.y + S.z * S.z) ^ .5
end

function dist3D(a,b)
	return vecLength(subVecs(a, b))
end

function calcY(pos,vel,base,t)
	return (D*gt-D^(t+1)*gt-t*gt+D*t*gt-base*(D^2)+base*D+2*D*(pos+vel*t)-(pos+vel*t)-(D^2)*(pos+vel*t)+base*D^(t+2)-base*D^(t+1))/(D*(D-1)*(-(D^t)+1))
end

function calc(pos,vel,base,t)
	return (-D*base+D^(t+1)*base+(pos+vel*t)-D*(pos+vel*t))/(D*(-D^t+1))
end

-- Per-tick delta for velocity estimation
function deltaVec(vec,id)
    del[id] = del[id] or Vector(0,0,0)
    out = subVecs(vec,del[id])
    del[id] = vec
    return out
end

-- First-order vector smoothing (linear interpolation)
function smoothVec(targetVec,currentVec,constant)
	return addVecs(vecScaleN(currentVec,1-constant),vecScaleN(targetVec,constant))
end

-- relative directional orthogonal vecs of the vehicle
function relativeAxes(HDG_forward, tilt_forward, HDG_right, tilt_right)
    forward_directional = vecFromEuler(-HDG_forward, tilt_forward, 1)
    right_directional = vecFromEuler(-HDG_right, tilt_right, 1)
    vertical_directional = crossProd(right_directional, forward_directional)

    index = {
        fwd = forward_directional,
        r = right_directional,
        up = vertical_directional
    }

    return index
end

--make a vector local to the relative orthogonals of a subgrid
function toLocalVec(vec,relAxes)
	return Vector(
	dotProd(vec,relAxes.r),
	dotProd(vec,relAxes.fwd),
	dotProd(vec,relAxes.up)
		)
end

-- Solve time of flight using Newton's method
function solveTOF(tgtvel, basevel, relpos)
    prev = relpos.y / 30 + relpos.z / 100
    for i = 1, iterations do
        fx = (calc(0, tgtvel.x, basevel.x, prev) ^ 2 + calc(relpos.y, tgtvel.y, basevel.y, prev) ^ 2 + calcY(relpos.z, tgtvel.z, basevel.z, prev) ^ 2) ^ 0.5 / vt - 1
        dx = (fx - ((calc(0, tgtvel.x, basevel.x, prev - 0.001) ^ 2 + calc(relpos.y, tgtvel.y, basevel.y, prev - 0.001) ^ 2 + calcY(relpos.z, tgtvel.z, basevel.z, prev - 0.001) ^ 2) ^ 0.5 / vt - 1)) / 0.001
        prev = prev - fx / dx
    end
    new = prev
    return removeNaNfromX(new)
end

--finds intial altitude offset
function initialCalc(relativePos,relativeVel,relativeBase,TOF,dist)
    verticalReq = calcY(relativePos.z,relativeVel.z,relativeBase.z,TOF)
    offset = asin(verticalReq/vt)
    altOffset = sin(offset) * dist
    return removeNaNfromX(altOffset)
end	

--bullet XYZ at a given point
function calculateXYZ(initialVel,baseVel,t)
    iv=initialVel
    bv=baseVel
    x=((iv.z+bv.z)*((1-D^(t+1))/(1-D))-(iv.z+bv.z))-(1/(1-D))*g*(t-(D-D^(t+1))/(1-D))
    y=(iv.y+bv.y)*((1-D^(t+1))/(1-D))-(iv.y+bv.y)
    l=(iv.x+bv.x)*((1-D^(t+1))/(1-D))-(iv.x+bv.x)
    return Vector(l,x,y)
end

--properties
minElv = pgN("1")
maxElv = pgN("2")
manualSpeed = pgN("3")
aimSpeed = pgN("4")
tickDelay = pgN("5")
iterations = pgN("6")
GunOffset = Vector(pgN("10"), pgN("11"), pgN("12"))
accelCoef = pgN("14")
alternateCoords = pgB("Alt XYZ")
spreadCompensator = pgB("Recoil Comp")
physicsCorrection = pgB("Physics Comp")

function onTick()
	--ballistic variables
    velocity = data[selection][1]
    drag = (data[selection][2]) * 60
    D = 1 - (drag/60)
    gt = g/(60^2)
    vt = velocity/60
    lifeSpan = data[selection][3]
	
	--get global variables
	tgtPosRaw = Vector(gN(1),gN(2),gN(3))
    gunX,gunY,gunZ = gN(4),gN(5),gN(6)
    gunHDG = gN(7) * pi2
    gunHDGright = gN(8) * pi2
    gunTiltRight = gN(9) * pi2
    gunPitch = gN(10) * pi2
    windAngle = gN(11) * pi2
    windSpeed = gN(12)
    vHDG = gN(13) * pi2
    vHDGright = gN(14) * pi2
    vPitch = gN(15) * pi2
    vTiltRight = gN(16) * pi2
    manualUD = gN(17)
    manualLR = gN(18)
    autoAim = gN(19) == 1
    
    -- Alternate coordinate input
    alternateXYZ = Vector(gN(28),gN(29),gN(30))
    
    --bullet spread compensation from another lua block
	horizontalCorrection = spreadCompensator and gN(31) or 0
	verticalCorrection = spreadCompensator and gN(32) or 0
	
	--if both distances are not 0 then targetLock state is true
	
    trackerPosRaw = Vector(trackerX, trackerY, trackerZ)
    gunPosRaw = Vector(gunX, gunY, gunZ)
	
	--local directional vecs of tracker and gun (in case they are on seperate subgrids)
    relAxesGun = relativeAxes(gunHDG, gunPitch, gunHDGright, gunTiltRight)
    relAxesV = relativeAxes(vHDG,vPitch,vHDGright,vTiltRight)
	
	--custom offset for parallax error compensation
    gunGPSGlobalOffset = vecScale(toGlobalVec(GunOffset, relAxesGun), Vector(-1, -1, 0))
    gunALTGlobalOffset = vecScale(toGlobalVec(GunOffset, relAxesGun), Vector(0, 0, -1))
	
    gunPos = addVecs(addVecs(gunPosRaw, gunGPSGlobalOffset), gunALTGlobalOffset)
	
	windVec = windToVec(windAngle,windSpeed)
	
	tgtPosRaw = alternateCoords and alternateXYZ or tgtPosRaw
	
	--target and gun position derivitaves
    tgtPosDelta = deltaVec(tgtPosRaw,1)
    tgtCoords = addVecs(tgtPosRaw, vecScaleN(tgtPosDelta, tickDelay))

    gunPosDelta = deltaVec(gunPos,2)
    gunCoords = addVecs(gunPos, vecScaleN(gunPosDelta, tickDelay))

    tgtVel = subVecs(deltaVec(tgtCoords,3),windVec)

    gunVel = deltaVec(gunCoords,4)

    targetAccel = deltaVec(tgtVel,5)

    gunAccel = deltaVec(gunVel,6)
	
	--relative velocity and accel + smoothed accel since it has noise
    deltaVel = subVecs(tgtVel, gunVel)
    deltaAccelRaw = (subVecs(targetAccel, gunAccel))
    deltaAccel = smoothVec(deltaAccelRaw,deltaAccel,accelCoef)

    --initial distance calculation
    targetDist = dist3D(tgtCoords,gunCoords)
    
    --relative distance to target
    targetRelative = subVecs(tgtCoords,gunCoords)
    
    --three vectors that describe our orientation relative to the horizon
    relFwd = normalize(Vector(targetRelative.x,targetRelative.y,0))
	relR = (Vector(relFwd.y,-relFwd.x,0))
	relUp = Vector(0,0,1)

	relativeHorizon = {	
	fwd = relFwd,
	r = relR,
	up = relUp
		}
     
    --position and velocity vectors relative to the horizon
    tgtVelRelative = toLocalVec(tgtVel,relativeHorizon) 
    baseVelRelative = toLocalVec(gunVel,relativeHorizon)
    tgtPosRelative = Vector(0,vecLength(Vector(targetRelative.x,targetRelative.y,0)),targetRelative.z)
    
	--offset and time to target calculation to compensate for bullet drop
	TOF = solveTOF(tgtVelRelative, baseVelRelative, tgtPosRelative)
	
    --if target is out of range or TOF is nil
    if TOF > lifeSpan or TOF ~= TOF or abs(TOF) == math.huge then
        outOfRange = true
    end

	--initial lead calc 
    leadingAim = addVecs(addVecs(tgtCoords, vecScaleN(deltaVel, TOF)), vecScale(vecEquals(1 / 2), vecScaleN(deltaAccel,TOF^2)))
    --distance to lead point    
    targetDist = dist3D(leadingAim,gunCoords)
    --altitude offset using Newton's method
	altOffset = initialCalc(tgtPosRelative,tgtVelRelative,baseVelRelative,TOF,targetDist)	
    --leading aim recalc 
    leadingAim = addVecs(addVecs(Vector(tgtCoords.x,tgtCoords.y,altOffset+gunCoords.z), vecScaleN(deltaVel, TOF)), vecScale(vecEquals(1 / 2), vecScaleN(deltaAccel,TOF^2)))

	--normalizes the the tracker and target vecs
	leadingAimDelta = subVecs(leadingAim,gunCoords)
	leadingAimNormal = normalize(leadingAimDelta)
	
	--note: we take dot product of target lead vec and relative orthogonal vecs of vehicle - this allows us to accurately hit a target, regardless of gun orientation
	offsetAim = normalize(addVecs(toGlobalVec(vecScaleN(vecFromEuler(horizontalCorrection,verticalCorrection,1),vecLength(leadingAimDelta)),relAxesGun),leadingAimDelta))
	
	--converts global aim vec to a vec local to the gun
	localAimVec = toLocalVec(offsetAim,relAxesGun)
	
	--global aim vec local to vehicle the gun is mounted on
	VehicleAimVec = toLocalVec(offsetAim,relAxesV)
	
	hRelative = atan(localAimVec.x,localAimVec.y)
	hDirect = atan(VehicleAimVec.x,VehicleAimVec.y)
	vDirect = asin(VehicleAimVec.z/vecLength(offsetAim))
	
	--finds needed angles for gun to aim at target
	autoAimH = removeNaNfromX((hRelative/pi2)*-aimSpeed)
	autoAimV = removeNaNfromX((vDirect/pi2)*4)
	
	--adds recoil compensation and ability to manually control
	hCmd = (autoAim and autoAimH  or -manualLR*manualSpeed)
	vCmd = clamp((autoAim and autoAimV or manualUD),minElv,maxElv)

   --calculator error analysis
   initialVel = vecFromEuler(hDirect,vDirect,vt)
   bulletPos = calculateXYZ(initialVel,gunVel,TOF or 0)
   error = subVecs(bulletPos,leadingAimDelta)

	--outputs variables on set channels
    sB(1,outOfRange)
    sB(2,not (tgtCoords.x == 0 or tgtCoords.y == 0 or tgtCoords.z == 0))
    sN(1,leadingAim.x)
    sN(2,leadingAim.y)
    sN(3,leadingAim.z)
    sN(4,hCmd)
    sN(5,vCmd)
    sN(6,leadingAim.x)
    
end
