-- [Deprecated] Ballistics Neural Network Prototype
--
-- Neural network augmented horizontal aim correction for the
-- ballistics computer. Trained at startup on recorded error data.

gN = input.getNumber
gB = input.getBool
sN = output.setNumber
pgN = property.getNumber
sin = math.sin
cos = math.cos
atan = math.atan
acos = math.acos
asin = math.asin
pi2 = math.pi * 2

-- Define the network architecture

input_size = 4
hidden_size = pgN("Hidden Nodes")
output_size = 1
learning_rate = pgN("Learning Rate")

Vector = function(b, c, e)
    return {x = b, y = c, z = e}
end

-- Initialize the network weights and biases randomly
W1 = {}
for i = 1, input_size do
    W1[i] = {}
    for j = 1, hidden_size do
        W1[i][j] = math.random()
    end
end
b1 = {}
for i = 1, hidden_size do
    b1[i] = math.random()
end
W2 = {}
for i = 1, hidden_size do
    W2[i] = {}
    for j = 1, output_size do
        W2[i][j] = math.random()
    end
end
b2 = {}
for i = 1, output_size do
    b2[i] = math.random()
end

-- Define the activation function (sigmoid)
function sigmoid(x)
    return 1 / (1 + math.exp(-x))
end

-- Define the derivative of the activation function (sigmoid)
function sigmoid_prime(x)
    return sigmoid(x) * (1 - sigmoid(x))
end

-- Define the forward pass function
function forward(x)
    hidden = {}
    for i = 1, hidden_size do
        h = 0
        for j = 1, input_size do
            h = h + x[j] * W1[j][i]
        end
        h = h + b1[i]
        hidden[i] = sigmoid(h)
    end
    output = 0
    for i = 1, hidden_size do
        output = output + hidden[i] * W2[i][1]
    end
    output = output + b2[1]
    return sigmoid(output), hidden
end

-- Define the backward pass function
function backward(x, y, output, hidden)
    delta_output = (output - y) * sigmoid_prime(output)
    delta_hidden = {}
    for i = 1, hidden_size do
        delta_hidden[i] = delta_output * W2[i][1] * sigmoid_prime(hidden[i])
    end
    for i = 1, hidden_size do
        for j = 1, output_size do
            W2[i][j] = W2[i][j] - learning_rate * delta_output * hidden[i]
        end
    end
    for i = 1, input_size do
        for j = 1, hidden_size do
            W1[i][j] = W1[i][j] - learning_rate * delta_hidden[j] * x[i]
        end
    end
    for i = 1, hidden_size do
        b1[i] = b1[i] - learning_rate * delta_hidden[i]
    end
    b2[1] = b2[1] - learning_rate * delta_output
end

-- Define the train function
function train(data, num_epochs)
    for epoch = 1, num_epochs do
        total_loss = 0
        for i = 1, #data do
            x = data[i][1]
            y = data[i][2]
            output, hidden = forward(x)
            loss = (output - y)^2
            total_loss = total_loss + loss
            backward(x, y, output, hidden)
        end
    end
end

function test(data, model)
    -- Compute the network's predictions for the test data
    for i = 1, #data do
        x = data[i][1]
        y = data[i][2]
        hidden = {}
        for j = 1, input_size do
            h = 0
            for k = 1, input_size do
                h = h + x[k] * model.W1[k][j]
            end
            h = h + model.b1[j]
            hidden[j] = sigmoid(h)
        end
        output = 0
        for j = 1, hidden_size do
            output = output + hidden[j] * model.W2[j][1]
        end
        output = output + model.b2[1]
        prediction = sigmoid(output)
    end
end

function sign(x) 
	return x < 0 and -1 or 1
end

function divVecs(A, R)
    return Vector(A.x / R, A.y / R, A.z / R)
end

function vecLength(S)
    return (S.x * S.x + S.y * S.y + S.z * S.z) ^ .5
end

function addVecs(A, B)
    return Vector(A.x + B.x, A.y + B.y, A.z + B.z)
end

function vecScaleN(A, n)
    return Vector(A.x * n, A.y * n, A.z * n)
end

function normalize(S)
    return divVecs(S, vecLength(S))
end

function toGlobalVec(vec, relAxes)
    return addVecs(
        addVecs(vecScaleN(relAxes.r, vec.x), vecScaleN(relAxes.fwd, vec.y)),
        vecScaleN(relAxes.up, vec.z)
    )
end

function vecFromEuler(yawangle, pitchangle, distance)
    c = distance or 1
    plane = (cos(pitchangle) * c)
    return Vector(
        (sin(yawangle) * plane),
        (cos(yawangle) * plane),
        (sin(pitchangle) * c)
    )

end

function vecToEuler(vec)
    return Vector(
        acos(normalize(Vector(vec.x, vec.y, 0)).y) * sign(vec.x),
        asin(normalize(vec).z), 
        0
    )
end

function crossProd(a, b)
    return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
end

-- dot product of two 3D vecs
function dotProd(a, b)
    return a.x * b.x + a.y * b.y + a.z * b.z
end

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

function toLocalVec(vec,relAxes)
	return Vector(
	dotProd(vec,relAxes.r),
	dotProd(vec,relAxes.fwd),
	dotProd(vec,relAxes.up)
		)
end

train_data = {
    { {0.02496,-0.00372,0.00175,203.48},0.0288},
    { {-0.0805,0.0191,0.2772,170.895},-0.09959},
    { {0.0342,-0.0552,0.494,190.173},0.3971},
    { {0.1358,-0.0319,0.2648,106.1844},0.1672},
    { {0.3551,-0.083,0.37724,121.988},0.43781},
    { {-3.0183,-0.01196,0.49771,186.19196},-3.00714},
    { {0.99,-0.02,0.0007,616.5},1.0145},
    { {0.22,-0.0478,0.0765,134.6},0.27},
    { {-0.28,0.0452,0.1406,175.9},-0.324},
    { {-1.62,-0.0046,1.504,708.7},-1.616},
    { {-1.33,0.0045,1.5,706.5},-1.335},
    { {-1.49,-0.002,1.5,676.6},-1.4836},
    { {-1.12,0.013,1.50,1027.6},-1.1377},
    { {-1.23,0.0108,1.50,1091.9},-1.244},
    { {0.74,-0.1,0.31,136},0.838},
    { {-0.21,0.008,0.068,775.4},-0.218},
    { {2.18,0.0177,0.506,569.4},2.164},
    { {-0.827,0.023,0.4857,696.3},-0.8499},
    { {0.86,-0.0275,0.31,530.2},0.8906},
    { {-0.75,0.012,0.44,922.1},-0.758},
    { {2.585,0.017,0.34,846.7},2.567},
    { {-1.87,-0.01,0.16,898.1},-1.863},
    { {0.052,-0.0032,0.45,907.9},0.055}
}

train(train_data, pgN("Training Loops"))

aimSpeed = pgN("4")

-- Horizontal aim correction
function onTick()
	autoAim = gB(3)
	hCmd = gN(4)
	vCmd = gN(5)
	hError = gN(6)
    vError = gN(7)
    targetVel = gN(8)
    dist = gN(9)
    relAxesGun = relativeAxes(gN(10), gN(11), gN(12), gN(13))
    relAxesV = relativeAxes(gN(14), gN(15), gN(16), gN(17))
	
	test_data = {
    {{hCmd, hError, targetVel, dist}}
    }
    
    	
for i = 1, #test_data do
    x = test_data[i][1]
    correction, _ = forward(x)
    horizontalAim = autoAim and (vecToEuler(toLocalVec(toGlobalVec(vecFromEuler(correction,0),relAxesV),relAxesGun)).x/pi2)*-aimSpeed or hCmd
    sN(1,horizontalAim)
    sN(2,hCmd)
	end

end
	