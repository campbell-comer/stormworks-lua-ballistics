-- Dive-Bomb Assist
--
-- Proportional Navigation (PN) guidance for dive-bomb targeting.
-- Extrapolates aircraft and target positions forward, then computes
-- line-of-sight angles to guide the aircraft onto the target.
--
-- INPUTS (numbers)
--   1..3   Aircraft position XYZ (world)
--   2..4   Target position XYZ (world)
--
-- OUTPUTS (numbers)
--   1      LOS yaw command
--   2      LOS pitch command
--
-- PROPERTIES
--   "DB Nav Constant"   PN guidance gain

------------------------
-- I/O and math aliases
------------------------
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

------------------------
-- Vector math
------------------------
Vector = function(b, c, e)
    return {x = b or 0, y = c or 0, z = e or 0}
end

function nilVec()
	return Vector(0,0,0)
end

function removeNaN(x)
    return (x ~= x or abs(x) == math.huge) and 0 or x
end

function sign(x)
	return x < 0 and -1 or 1
end

function clamp(a, b, c)
    return math.min(math.max(a, b), c)
end

function addVecs(A, B)
    return Vector(A.x + B.x, A.y + B.y, A.z + B.z)
end

function subVecs(A, B)
    return Vector(A.x - B.x, A.y - B.y, A.z - B.z)
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

function vecPowerN(a, n)
    return Vector(a.x ^ n, a.y ^ n, a.z ^ n)
end

function dist3D(a,b)
	return vecLength(subVecs(a, b))
end

-- Per-tick delta for velocity estimation
del = {}
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

function vecScale(A, B)
    return Vector(A.x * B.x, A.y * B.y, A.z * B.z)
end

function vecScaleN(A, n)
    return Vector(A.x * n, A.y * n, A.z * n)
end

function normalize(S)
    return divVecs(S, vecLength(S))
end

function crossProd(a, b)
    return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
end

function dotProd(a, b)
    return a.x * b.x + a.y * b.y + a.z * b.z
end

------------------------
-- Coordinate conversions
------------------------
-- Spherical (yaw, pitch, distance) to Cartesian vector
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

------------------------
-- Reference frame transforms
------------------------
-- Local-to-world: express a local vector in world coordinates
function toGlobalVec(vec, relAxes)
    return addVecs(
        addVecs(vecScaleN(relAxes.r, vec.x), vecScaleN(relAxes.fwd, vec.y)),
        vecScaleN(relAxes.up, vec.z)
    )
end

-- World-to-local: project a world vector onto local axes
function toLocalVec(vec,relAxes)
	return Vector(
	dotProd(vec,relAxes.r),
	dotProd(vec,relAxes.fwd),
	dotProd(vec,relAxes.up)
		)
end

-- Build orthonormal basis from heading and pitch sensor readings
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

function deltaVec(vec,id)
    del[id] = del[id] or Vector(0,0,0)
    out = subVecs(vec,del[id])
    del[id] = vec
    return out
end

------------------------
-- Dive-bomb controller
------------------------
nav = pgN("DB Nav Constant")
ticks = 4
function onTick()
currentPos = Vector(gN(1),gN(2),gN(3))
targetPos = Vector(gN(2),gN(3),gN(4))

currentPosDelta = deltaVector(currentPos,1)
targetPosDelta = deltaVector(targetPos,2)

currentPosExtr = addVecs(currentPos,vecScaleN(currentPosDelta,tick))
targetPosExtr = addVecs(targetPos,vecScaleN(targetPosDelta,ticks))

LOS = vectorToEuler(normalize(subVecs(targetPosExtr,currentPosExtr)))

sN(1,LOS.x)
sN(2,LOS.y)

end
