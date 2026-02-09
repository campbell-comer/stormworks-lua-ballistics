-- [Deprecated] PID Auto-Tuner (RLS)
--
-- Recursive Least Squares based PID auto-tuner for
-- engine RPS control.

-- Initial PID parameters
 Kp_initial = 1.0
 Ki_initial = 0.1
 Kd_initial = 0.01

-- PID parameters (initial values)
 Kp = Kp_initial
 Ki = Ki_initial
 Kd = Kd_initial

-- RLS parameters
 P = {{1000, 0, 0}, {0, 1000, 0}, {0, 0, 1000}} -- Initial large values for high uncertainty
 theta = {Kp_initial, Ki_initial, Kd_initial}
 lambda = 0.98 -- Forgetting factor

-- PID control variables
 previousError = 0
 integral = 0

-- Helper function to perform matrix-vector multiplication
 function matrixVectorMultiply(matrix, vector)
     result = {}
    for i = 1, #matrix do
        result[i] = 0
        for j = 1, #vector do
            result[i] = result[i] + matrix[i][j] * vector[j]
        end
    end
    return result
end

-- Helper function to perform scalar-vector multiplication
 function scalarVectorMultiply(scalar, vector)
     result = {}
    for i = 1, #vector do
        result[i] = scalar * vector[i]
    end
    return result
end

-- Helper function to perform vector transpose multiplication
 function vectorTransposeMultiply(vector1, vector2)
     result = 0
    for i = 1, #vector1 do
        result = result + vector1[i] * vector2[i]
    end
    return result
end

-- Helper function to perform matrix update
 function matrixUpdate(matrix, vector, gain, scalar)
    for i = 1, #matrix do
        for j = 1, #matrix[i] do
            matrix[i][j] = (matrix[i][j] - gain[i] * vector[j]) / scalar
        end
    end
end

-- Function to update PID parameters using RLS
function updatePIDParameters(error, controlRPS)
     phi = {previousError, integral, -controlRPS}
     y = error
     P_phi = matrixVectorMultiply(P, phi)
     phi_T_P_phi = vectorTransposeMultiply(phi, P_phi)
     gain = scalarVectorMultiply(1 / (lambda + phi_T_P_phi), P_phi)

    -- Update theta (PID gains)
    for i = 1, #theta do
        theta[i] = theta[i] + gain[i] * (y - vectorTransposeMultiply(phi, theta))
    end

    -- Update P
    matrixUpdate(P, phi, gain, lambda)

    -- Assign updated PID gains
    Kp = theta[1]
    Ki = theta[2]
    Kd = theta[3]
end

function onTick()
    -- Read inputs
     targetRPS = input.getNumber(1)
     controlRPS = input.getNumber(2)

    -- Calculate error
     error = targetRPS - controlRPS

    -- Update integral and derivative
    integral = integral + error
     derivative = error - previousError

    -- Calculate PID output
     output = Kp * error + Ki * integral + Kd * derivative

    -- Update output throttle
     outputThrottle = output

    -- Write output
    output.setNumber(1, outputThrottle)

    -- Update previous error
    previousError = error

    -- Update PID parameters using RLS
    updatePIDParameters(error, controlRPS)
end