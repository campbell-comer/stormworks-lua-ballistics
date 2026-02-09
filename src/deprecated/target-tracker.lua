-- [Deprecated] Target Tracker
--
-- Radar-to-world-position converter with GPS offset
-- compensation and velocity extrapolation.
-- Originally by YAGoOaR.

pgn = property.getNumber
pgb = property.getBool
setNum = output.setNumber
getNum = input.getNumber
getB = input.getBool
setB = output.setBool
m = math
absolute = m.abs
cosine = m.cos
sine = m.sin
acosine = m.acos
asine = m.asin
tang = m.tan
arctan = m.atan

pi = m.pi
pi2 = pi * 2

Vector = function(x, y, z) return {x = x, y = y, z = z} end

function vecAdd(A, B)
	return Vector(A.x + B.x, A.y + B.y, A.z + B.z)
end

function vecScale(A, B)
	return Vector(A.x * B.x, A.y * B.y, A.z * B.z)
end

function vecScaleN(A, n)
	return Vector(A.x * n, A.y * n, A.z * n)
end

function vecNeg(A)
	return vecScaleN(A, -1)
end

function vecSub(A, B)
	return vecAdd(A, vecNeg(B))
end

function vecLen(vec)
	return (vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)^.5
end

function vecDiv(A, n)
	return vecScaleN(A, 1/n)
end

function vecNormalize(vec)
	return vecDiv(vec, vecLen(vec))
end

-- Cross product
function vecProd(a, b)
	return Vector(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x		
	)
end

-- Spherical to cartesian
function vecFromEuler(hdg, ptch)
	return Vector(
		sine(hdg) * cosine(ptch),
		cosine(hdg) * cosine(ptch),
		sine(ptch)
	)
end

function sign(x) 
	return x < 0 and -1 or 1
end

-- -- Cartesian to spherical
-- function vecToEuler(vec)
-- 	return Vector(
-- 		acosine(vecNormalize(Vector(vec.x, vec.y, 0)).y) * sign(vec.x),
-- 		asine(vecNormalize(vec).z), 
-- 		0
-- 	)
-- end

-- -- Vector scalar product
-- function vecDot(A, B)
-- 	return A.x * B.x + A.y * B.y + A.z * B.z
-- end

-- -- Scalar projection
-- function scalarProj(A, B)
-- 	return vecDot(A, B) / vecLen(B)
-- end

-- -- Transforms rotation vector vec from global to local coords system (relAxes are vehicle coord system axes)
-- function toLocal(vec, relAxes)
-- 	return Vector(scalarProj(vec, relAxes.right), scalarProj(vec, relAxes.forward), scalarProj(vec, relAxes.up))
-- end

-- Transforms rotation vector vec from local to global coords system (relAxes are vehicle coord system axes)
function toGlobal(vec, relAxes)
	return vecAdd(vecAdd(vecScaleN(relAxes.right, vec.x), vecScaleN(relAxes.forward, vec.y)), vecScaleN(relAxes.up, vec.z))
end


-- Returns vectors of relative vehicle axes: forwardAxis, rightAxis, upAxis
function RelAxesVecs(forwardHDG, forwardPTCH, rightHDG, rightPTCH)
	local relAxes = {
		forward = vecFromEuler(forwardHDG, forwardPTCH),
		right = vecFromEuler(rightHDG, rightPTCH)
	}
	relAxes.up = vecProd(relAxes.right, relAxes.forward)
	return relAxes
end

function deltaVal(initVal, diffMethod)
	local pVal = initVal
	return function(val)
		delta = diffMethod(val, pVal)
		pVal = val
		return delta
	end
end

function radarToLocalDeltaVec(x, y, distFront, distBack)
	hh = sine(x) * distFront
	hv = sine(y) * distBack
	k1 = cosine(y) * distBack
	l = (k1^2 - hh^2)^.5
	return vecAdd(Vector(hh, l, hv), Vector(0, radarBackOffsetY, 0))
end

GPSoffset = Vector(pgn("1"), pgn("2"), pgn("3"))
ALToffset = Vector(pgn("4"), pgn("5"), pgn("6"))
radarBackOffsetY = pgn("7")

deltaTarget = deltaVal(Vector(0, 0, 0), vecSub)

ticksExtr = pgn("ticksExtr")

function onTick()

	if getNum(30) == 0 and getNum(31) == 0 then return end
	
	posRaw = Vector(getNum(21), getNum(22), getNum(23))
	fhdg, fptch, rhdg, rptch = -getNum(24)*pi2, getNum(25)*pi2, -getNum(26)*pi2, getNum(27)*pi2
	radarX, radarY, distFront, distBack = getNum(28)*pi2, getNum(29)*pi2, getNum(30), getNum(31)

	relAxes = RelAxesVecs(fhdg, fptch, rhdg, rptch)

	GPSGlobalOffset = vecScale(toGlobal(GPSoffset, relAxes), Vector(-1, -1, 0))
	ALTGlobalOffset = vecScale(toGlobal(GPSoffset, relAxes), Vector(0, 0, -1))
	pos = vecAdd(vecAdd(posRaw, GPSGlobalOffset), ALTGlobalOffset)

	delta = toGlobal(radarToLocalDeltaVec(radarX, radarY, distFront, distBack), relAxes)

	tgtPos = vecAdd(pos, delta)
	tgtDelta = deltaTarget(tgtPos)
	tgtPosExtr = vecAdd(tgtPos, vecScaleN(tgtDelta, ticksExtr))

	setNum(1, tgtPosExtr.x)
	setNum(2, tgtPosExtr.y)
	setNum(3, tgtPosExtr.z)
end
