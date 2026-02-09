---@diagnostic disable: undefined-global, lowercase-global
-- Ballistic Calculator 3D (Weapon DLC discrete drag + wind) + stable control by Vour
--
-- PURPOSE
--   1) Predict an intercept point in 3D (absolute world XYZ) for a projectile with discrete drag + gravity + wind.
--   2) Output stabilized yaw/pitch commands to aim a turret at that solution.
--   3) Remain stable with noisy trackers and moving platforms.
--
-- INPUT MAP (numbers)
--   1..3   Target absolute XYZ (tracker):              [X,Y,Z]  (world)
--
--   GUN Physics Sensor (externally remapped into LUA)
--     4..6   pos XYZ (PS outs 1..3)
--     7..9   vel XYZ m/s (PS outs 7..9)
--     10     local pitch (turns) (PS out 15)
--     11     local roll  (turns) (PS out 16)
--     12     heading     (turns) (PS out 17)
--
--   BODY Physics Sensor (externally remapped into LUA)
--     13     local pitch (turns) (PS out 15)
--     14     local roll  (turns) (PS out 16)
--     15     heading     (turns) (PS out 17)
--
--   16     wind angle (turns)
--   17     wind speed (m/s)
--   18     manual UD (pitch)
--   19     manual LR (yaw)
--   20     auto aim enable (1 = on)
--
--   31     recoil yaw turns  (only used if Recoil Comp enabled)
--   32     recoil pitch bias (only used if Recoil Comp enabled)
--
-- FIXES INCLUDED (NO NEW TUNABLES)
--   A) "Double lead" removal for slow ground targets:
--      Ballistics already leads by TOF, so the solver must use target_pos_raw (no pre-advance).
--      Control-side LOS-rate feedforward may still look ahead, but:
--        ground: delay_ctrl = min(delay_eff, tof)
--        air:    delay_ctrl = delay_eff
--
--   B) Manual yaw always works (even with no valid target):
--      When the tracker is invalid, we still run the manual yaw loop and output yaw_cmd.

------------------------
-- I/O
------------------------
input_number  = input.getNumber
output_number = output.setNumber
output_bool   = output.setBool
prop_number   = property.getNumber
prop_bool     = property.getBool

------------------------
-- Math
------------------------
m_sin   = math.sin
m_cos   = math.cos
m_atan  = math.atan
m_abs   = math.abs
m_sqrt  = math.sqrt
m_min   = math.min
m_max   = math.max
m_exp   = math.exp
m_log   = math.log
m_floor = math.floor

TAU = math.pi * 2

------------------------
-- Weapon selection table
-- Each row: { muzzle_velocity_m/s, drag_per_tick, lifespan_ticks, wind_affect }
------------------------
weapon_selection = prop_number("Weapon Type")
weapon_data = {
    {800,  0.025, 120,  0.15},
    {1000, 0.02,  150,  0.135},
    {1000, 0.01,  300,  0.13},
    {900,  0.005, 600,  0.125},
    {800,  0.002, 1500, 0.12},
    {700,  0.001, 2400, 0.11},
    {600,  0.0005,2400, 0.105}
}

---------------------------------------------------------
-- Property read with defaults (ONLY used for the properties you requested)
---------------------------------------------------------
function prop_num_default(name, def)
    v = prop_number(name)
    return (v == 0) and def or v
end

---------------------------------------------------------
-- Properties (ONLY THESE are read from property menu)
---------------------------------------------------------
-- Pitch limits are in turns where 1.0 = 90 degrees for Stormworks turret pitch convention
prop_min_pitch = prop_num_default("Min Pitch (deg)", -45) / 90
prop_max_pitch = prop_num_default("Max Pitch (deg)",  90) / 90

-- Manual scaling:
--   yaw uses a rate-style input (keys -> desired yaw rate)
--   pitch uses a position-style input (keys -> desired pitch position)
prop_manual_yaw_speed   = prop_num_default("Manual Yaw Multiplier",   0.1)
prop_manual_pitch_speed = prop_num_default("Manual Pitch Multiplier", 1.2)

-- Offsets in INTERNAL (OLD) local coords:
--   +X right, +Y forward, +Z up
prop_gun_offset = {
    x = prop_num_default("Gun Offset X", 0),
    y = prop_num_default("Gun Offset Y", 0.5),
    z = prop_num_default("Gun Offset Z", 0)
}

-- Muzzle offset in the SAME local coords as above:
-- This is applied AFTER converting turret origin to the true pivot.
prop_muzzle_offset = {
    x = prop_num_default("Muzzle Offset X", 0),
    y = prop_num_default("Muzzle Offset Y", 0),
    z = prop_num_default("Muzzle Offset Z", 0)
}

---------------------------------------------------------
-- Hard-coded "properties" (everything else stays hardcoded)
---------------------------------------------------------
prop_manual_speed    = prop_manual_yaw_speed  -- yaw manual rate scaling
prop_aim_speed       = 7.5                    -- yaw feedback aggressiveness (range-scheduled below)
prop_tick_delay      = 4.0                    -- control latency compensation (ticks)
prop_yaw_track       = 0.35                   -- first-order tracking gain for yaw-rate estimate
prop_air_delay_scale = 0.5                    -- extra delay applied only when airborne + adv enabled
prop_recoil_comp     = false
prop_adv_enable      = true

---------------------------------------------------------
-- Constants / tunables (kept hard-coded)
---------------------------------------------------------
TICKS_PER_SEC   = 60
GRAVITY_MS2     = 30
GRAVITY_TICK2   = GRAVITY_MS2 / (TICKS_PER_SEC*TICKS_PER_SEC)

YAW_MAX_TPS     = 1.25      -- actuator capacity in turns/sec at cmd=1
DROP_HOLD_TICKS = 20        -- tracker hold when it drops
CLOSE_RANGE_HARD_M = 2.2    -- avoids solver overhead very close-in

REL_ACCEL_CAP   = 0.006     -- cap on relative acceleration used for lead

YAW_ERR_DEADBAND_TURNS = 0.0009
MAN_ERR_DEADBAND_TURNS = 0.0007

YAW_FF_BLEND_OLD = 0.55
YAW_FF_BLEND_NEW = 0.45
YAW_RATE_BRAKE_GAIN = 0.016

PITCH_HYSTERESIS = 0.01
PITCH_HYST_MIN         = 0.0015
PITCH_HYST_OMEGA_START = 0.008
PITCH_HYST_OMEGA_END   = 0.040

YAW_CMD_MAX_STEP = 0.22     -- step clamp to prevent oscillation on noisy aim updates

TOF_SCAN_END_TICKS  = 360
TOF_SCAN_STEP_TICKS = 6
TOF_SCAN_TAIL_STEPS = 18
TOF_ITERS = 24

YAW_FF_MAX    = 0.85        -- limit on feedforward command magnitude

YAW_ACT_DELAY_TICKS   = 3.2 -- yaw actuator effective delay for prediction
PITCH_ACT_DELAY_TICKS = 0.9 -- pitch actuator effective delay for prediction

OMEGA_XY_MAX = 0.06         -- cap LOS angular rate components (x/y) to avoid spikes

AIMDIR_LP_START_M   = 650
AIMDIR_LP_END_M     = 1250
AIMDIR_LP_MIN_ALPHA = 0.18

AIR_SPEED_TICK          = 0.2
TARGET_VEL_ALPHA_GROUND = 0.35
TARGET_VEL_ALPHA_AIR    = 0.15

ACC_TRUST_DV_TICK       = 0.06
ACC_TRUST_MIN           = 0.2

AIR_CURV_BLEND          = 0.55
AIR_TURN_ALPHA          = 0.2
AIR_TURN_CAP            = 0.10

---------------------------------------------------------
-- Persistent state table
-- Used for filters, hysteresis, integrators, and last-sample caches
---------------------------------------------------------
state = {}

---------------------------------------------------------
-- Vector math (all vectors are {x,y,z} in OLD world coords: +X east/right, +Y north/forward, +Z up)
---------------------------------------------------------
function vec3(x,y,z) return {x=x or 0, y=y or 0, z=z or 0} end
function vec_add(a,b) return {x=a.x+b.x, y=a.y+b.y, z=a.z+b.z} end
function vec_sub(a,b) return {x=a.x-b.x, y=a.y-b.y, z=a.z-b.z} end
function vec_scale(v,s) return {x=v.x*s, y=v.y*s, z=v.z*s} end
function vec_dot(a,b) return a.x*b.x + a.y*b.y + a.z*b.z end
function vec_cross(a,b)
    return {
        x = a.y*b.z - a.z*b.y,
        y = a.z*b.x - a.x*b.z,
        z = a.x*b.y - a.y*b.x
    }
end
function vec_len(v) return m_sqrt(v.x*v.x + v.y*v.y + v.z*v.z) end
function clamp(x,a,b) return m_min(m_max(x,a),b) end

function remove_nan(x)
    return (x ~= x or m_abs(x) == math.huge) and 0 or x
end

-- Normalize vector; if nearly zero, return forward as a safe default
function vec_norm(v)
    L = vec_len(v)
    if L < 1e-9 then return {x=0,y=1,z=0} end
    return {x=v.x/L, y=v.y/L, z=v.z/L}
end

-- Wrap turns to [-0.5, +0.5) for shortest-angle computations
function wrap_turns(t)
    return (t + 0.5) % 1 - 0.5
end

-- One-tick delta of a vector (used to estimate velocity/acceleration from absolute tracker samples)
function vec_delta(v, id)
    p = state[id]
    if not p then
        state[id] = v
        return {x=0,y=0,z=0}
    end
    out = {x=v.x-p.x, y=v.y-p.y, z=v.z-p.z}
    state[id] = v
    return out
end

-- Tracker robustness: if position is unchanged (or very tiny change), decay the held velocity instead of snapping to 0
-- This prevents noisy trackers from outputting "teleport/no-move" artifacts that would break velocity estimates.
function vel_held(pos, id_delta, id_dt, id_vel, eps)
    d = vec_delta(pos, id_delta)
    mag1 = (m_abs(d.x) + m_abs(d.y) + m_abs(d.z))
    e = (eps or 1e-7)

    HOLD_ZERO_TICKS = 8
    HOLD_DECAY      = 0.70

    if mag1 <= e then
        dt = (state[id_dt] or 1) + 1
        state[id_dt] = dt

        v = state[id_vel] or {x=0,y=0,z=0}
        v = {x=v.x*HOLD_DECAY, y=v.y*HOLD_DECAY, z=v.z*HOLD_DECAY}
        if dt >= HOLD_ZERO_TICKS then v = {x=0,y=0,z=0} end

        state[id_vel] = v
        return v
    else
        dt = (state[id_dt] or 1)
        invdt = 1 / m_max(dt, 1)
        v = {x=d.x*invdt, y=d.y*invdt, z=d.z*invdt}
        state[id_dt] = 1
        state[id_vel] = v
        return v
    end
end

---------------------------------------------------------
-- Orientation math
--
-- axes_from_ypr(yaw,pitch,roll) builds an orthonormal basis:
--   axes.fwd = forward unit vector
--   axes.r   = right unit vector
--   axes.up  = up unit vector
--
-- Then:
--   vec_local_to_world(v_local, axes) = v_local.x * right + v_local.y * forward + v_local.z * up
--   vec_world_to_local(v_world, axes) = dot(v_world, right/forward/up)
--
-- This is used for:
--   - Converting local offsets (turret + muzzle) into world positions
--   - Converting aim and LOS vectors between gun/vehicle frames and world frame
---------------------------------------------------------
function axes_from_ypr(yaw, pitch, roll)
    cy = m_cos(yaw);  sy = m_sin(yaw)
    cp = m_cos(pitch);sp = m_sin(pitch)
    cr = m_cos(roll); sr = m_sin(roll)

    fwd = {x=sy*cp, y=cy*cp, z=sp}

    r = {
        x = cy*cr + sy*sp*sr,
        y = -sy*cr + cy*sp*sr,
        z = -cp*sr
    }

    up = {
        x = cy*sr - sy*cr*sp,
        y = -sy*sr - cy*cr*sp,
        z = cr*cp
    }

    return {fwd=fwd, r=r, up=up}
end

function vec_local_to_world(v, axes)
    return vec_add(
        vec_add(vec_scale(axes.r, v.x), vec_scale(axes.fwd, v.y)),
        vec_scale(axes.up, v.z)
    )
end

function vec_world_to_local(v, axes)
    return {x=vec_dot(v, axes.r), y=vec_dot(v, axes.fwd), z=vec_dot(v, axes.up)}
end

---------------------------------------------------------
-- Discrete-drag projectile model (Weapon DLC style)
--
-- Stormworks Weapon DLC uses per-tick multiplicative drag:
--   v[t+1] = D * v[t] + a   (component-wise, with constant accel 'a' per tick)
--
-- Summation gives displacement over t ticks:
--   s(t) = A(t)*v0 + (t - A(t))*(a/(1-D))
-- where:
--   A(t) = (1 - D^t) / (1 - D)
--
-- We invert that to solve required initial velocity v0 for a desired displacement s(t).
---------------------------------------------------------
function pow_d(D, t)
    if D <= 0 then return 0 end
    return m_exp(m_log(D) * t)
end

function A_of_t(D, t, one_minus_D)
    Dt = pow_d(D, t)
    return (1 - Dt) / one_minus_D
end

function inv_v0_discrete(s, a, D, t, one_minus_D)
    A = A_of_t(D, t, one_minus_D)
    if m_abs(A) < 1e-8 then return s / m_max(t, 1e-3) end
    accel_term = a * (t - A) / one_minus_D
    return (s - accel_term) / A
end

---------------------------------------------------------
-- TOF solver (time-of-flight)
--
-- We need a time t such that required muzzle-relative speed equals weapon muzzle speed:
--   |v_muzzle(t)| = vt
--
-- For a candidate t:
--   1) Predict target position using target_vel and target_acc (relative) in ticks.
--   2) Compute displacement disp = target(t) - gun_pos.
--   3) Invert discrete drag to get required total initial projectile velocity v0_total.
--   4) Convert to muzzle-relative velocity: v_muzzle = v0_total - gun_vel.
--   5) f(t) = |v_muzzle| - vt; root occurs when f(t)=0.
--
-- Implementation details:
--   - Coarse scan brackets a sign change for f(t).
--   - Secant/bisection refinement.
--   - Finalize to integer tick (t0 vs t0+1) with hysteresis (prevents tick-to-tick chatter).
---------------------------------------------------------
function solve_tof(target_pos, target_vel, target_acc, gun_pos, gun_vel, vt, D, one_minus_D, accel_world, life_span_ticks)
    best_t = 2
    best_abs = math.huge
    best_target = target_pos
    best_muzzle = {x=0,y=1,z=0}

    function eval_t(t)
        tgt = {
            x = target_pos.x + target_vel.x*t + 0.5*target_acc.x*t*t,
            y = target_pos.y + target_vel.y*t + 0.5*target_acc.y*t*t,
            z = target_pos.z + target_vel.z*t + 0.5*target_acc.z*t*t
        }
        disp = {x=tgt.x-gun_pos.x, y=tgt.y-gun_pos.y, z=tgt.z-gun_pos.z}

        v0_total = {
            x = inv_v0_discrete(disp.x, accel_world.x, D, t, one_minus_D),
            y = inv_v0_discrete(disp.y, accel_world.y, D, t, one_minus_D),
            z = inv_v0_discrete(disp.z, accel_world.z, D, t, one_minus_D)
        }

        muzzle = {x=v0_total.x-gun_vel.x, y=v0_total.y-gun_vel.y, z=v0_total.z-gun_vel.z}
        sp = m_sqrt(muzzle.x*muzzle.x + muzzle.y*muzzle.y + muzzle.z*muzzle.z)
        f = sp - vt

        af = m_abs(f)
        if af < best_abs then
            best_abs = af
            best_t = t
            best_target = tgt
            best_muzzle = muzzle
        end
        return f, tgt, muzzle
    end

    function finalize_integer(t_cont)
        t0 = clamp(m_floor(t_cont), 1, life_span_ticks)
        f0, tgt0, muz0 = eval_t(t0)

        t1 = clamp(t0 + 1, 1, life_span_ticks)
        f1, tgt1, muz1 = eval_t(t1)

        a0 = m_abs(f0)
        a1 = m_abs(f1)
        SWITCH_MARGIN = m_max(0.002 * vt, 0.0005)

        last = state[901]
        if last == t0 or last == t1 then
            fl, tgtl, muzl = eval_t(last)
            al = m_abs(fl)
            if (m_min(a0, a1) + SWITCH_MARGIN) >= al then
                state[901] = last
                return last, tgtl, muzl, true
            end
        end

        if (a1 + SWITCH_MARGIN) < a0 then
            state[901] = t1
            return t1, tgt1, muz1, true
        else
            state[901] = t0
            return t0, tgt0, muz0, true
        end
    end

    t_min = 0.05
    t_max = m_max(0.5, life_span_ticks)

    aT = t_min
    fa, _, _ = eval_t(aT)

    found = false
    brA = nil; brB = nil
    brFa = nil; brFb = nil

    scan_end = m_min(TOF_SCAN_END_TICKS, t_max)
    step = TOF_SCAN_STEP_TICKS

    t = aT + step
    while t <= scan_end do
        ft, _, _ = eval_t(t)
        if fa*ft <= 0 then
            brA = aT; brB = t
            brFa = fa; brFb = ft
            found = true
            break
        end
        aT = t; fa = ft
        t = t + step
    end

    if not found then
        steps = TOF_SCAN_TAIL_STEPS
        aT = scan_end
        fa, _, _ = eval_t(aT)
        for i=1,steps do
            t = scan_end + (t_max - scan_end) * (i/steps)
            ft, _, _ = eval_t(t)
            if fa*ft <= 0 then
                brA = aT; brB = t
                brFa = fa; brFb = ft
                found = true
                break
            end
            aT = t; fa = ft
        end
    end

    if not found then
        return finalize_integer(best_t)
    end

    a = brA; b = brB
    fa1 = brFa; fb1 = brFb

    for i=1,TOF_ITERS do
        if m_abs(fb1 - fa1) > 1e-9 then
            t = b - fb1*(b-a)/(fb1-fa1)
        else
            t = 0.5*(a+b)
        end
        if t <= a or t >= b then t = 0.5*(a+b) end

        ft, _, _ = eval_t(t)

        if m_abs(ft) < 1e-4 then
            return finalize_integer(t)
        end

        if fa1*ft <= 0 then
            b = t; fb1 = ft
        else
            a = t; fa1 = ft
        end

        if (b - a) < 1e-3 then
            tb = 0.5*(a+b)
            return finalize_integer(tb)
        end
    end

    tb = 0.5*(a+b)
    return finalize_integer(tb)
end

---------------------------------------------------------
-- MAIN LOOP
---------------------------------------------------------
function onTick()
    ------------------------
    -- Inputs
    ------------------------
    target_pos_raw = vec3(input_number(1), input_number(2), input_number(3))

    -- Physics Sensor axis fix:
    -- PS uses: X right, Y up, Z forward
    -- This script uses: X right, Y forward, Z up (OLD coords)
    -- So swap PS Y/Z for position and velocity.
    gun_pos_raw = vec3(input_number(4), input_number(6), input_number(5))
    gun_vel = vec3(
        input_number(7)/TICKS_PER_SEC,
        input_number(9)/TICKS_PER_SEC,
        input_number(8)/TICKS_PER_SEC
    )

    gun_pitch_fwd  = input_number(10) * TAU
    gun_roll_right = input_number(11) * TAU
    gun_hdg        = input_number(12) * TAU

    veh_pitch_fwd  = input_number(13) * TAU
    veh_roll_right = input_number(14) * TAU
    veh_hdg        = input_number(15) * TAU

    wind_angle    = input_number(16) * TAU
    wind_speed_ms = input_number(17)

    manual_ud = input_number(18)
    manual_lr = input_number(19)
    auto_aim  = (input_number(20) == 1)

    recoil_yaw_turns  = (prop_recoil_comp and input_number(31)) or 0
    recoil_pitch_bias = (prop_recoil_comp and input_number(32)) or 0

    ------------------------
    -- Weapon params
    ------------------------
    sel = clamp(weapon_selection, 1, #weapon_data)
    muzzle_velocity_ms = weapon_data[sel][1]
    drag_per_tick      = weapon_data[sel][2]
    life_span_ticks    = weapon_data[sel][3]
    wind_affect        = weapon_data[sel][4] or 0

    muzzle_velocity_tick = muzzle_velocity_ms / TICKS_PER_SEC
    D = clamp(1 - drag_per_tick, 1e-6, 0.999999)
    one_minus_D = 1 - D

    ------------------------
    -- Frames (gun + vehicle)
    ------------------------
    axes_gun = axes_from_ypr(-gun_hdg, gun_pitch_fwd, gun_roll_right)
    axes_veh = axes_from_ypr(-veh_hdg, veh_pitch_fwd, veh_roll_right)

    ------------------------
    -- Apply offsets (turret origin + muzzle)
    --
    -- turret offset: moves from sensor-reported position to true turret pivot/origin
    -- muzzle offset: moves from turret pivot/origin to actual muzzle exit point
    --
    -- Both offsets are defined in gun-local coordinates and rotated into world using axes_gun.
    ------------------------
    gun_offset_world = vec_local_to_world(prop_gun_offset, axes_gun)
    gun_pos = {
        x = gun_pos_raw.x - gun_offset_world.x,
        y = gun_pos_raw.y - gun_offset_world.y,
        z = gun_pos_raw.z - gun_offset_world.z
    }

    muzzle_offset_world = vec_local_to_world(prop_muzzle_offset, axes_gun)
    gun_pos = {
        x = gun_pos.x + muzzle_offset_world.x,
        y = gun_pos.y + muzzle_offset_world.y,
        z = gun_pos.z + muzzle_offset_world.z
    }

    ------------------------
    -- Target validity + dropout hold
    ------------------------
    target_valid = not (target_pos_raw.x == 0 and target_pos_raw.y == 0 and target_pos_raw.z == 0)

    if target_valid then
        state[280] = DROP_HOLD_TICKS
        state[281] = target_pos_raw
    else
        if (state[280] or 0) > 0 then
            state[280] = (state[280] or 0) - 1
            if state[282] then
                target_pos_raw = vec_add(
                    state[281] or target_pos_raw,
                    vec_add(vec_scale(state[282], 1), vec_scale(state[283] or vec3(0,0,0), 0.5))
                )
            else
                target_pos_raw = state[281] or target_pos_raw
            end
            target_valid = true
        end
    end

    ------------------------
    -- If no target at all: still allow manual yaw + manual pitch
    --
    -- Manual yaw here uses the same "angle hold" integrator used in normal manual mode:
    --   - Integrate desired yaw setpoint state[501] using the manual rate command.
    --   - Compute error between setpoint and measured yaw.
    --   - P controller maps error to actuator command in [-1,1].
    --
    -- This avoids outputting yaw=0, which is what previously made manual yaw feel "dead".
    ------------------------
    if not target_valid then
        state[201] = false

        -- Manual yaw (always active)
        gun_yaw_turns = input_number(12)
        if state[501] == nil then state[501] = gun_yaw_turns end

        manual_rate_cmd = (-manual_lr * prop_manual_speed) -- desired yaw rate sign matches previous logic
        state[501] = wrap_turns((state[501] or gun_yaw_turns) + (manual_rate_cmd * YAW_MAX_TPS / TICKS_PER_SEC))

        base_man_err = wrap_turns((state[501] or gun_yaw_turns) - gun_yaw_turns)
        if m_abs(base_man_err) < MAN_ERR_DEADBAND_TURNS and m_abs(manual_rate_cmd) < 0.02 then base_man_err = 0 end
        man_err_turns = wrap_turns(base_man_err + recoil_yaw_turns)

        man_kp = clamp(1 / m_max(YAW_MAX_TPS*0.25, 1e-6), 2, 6)
        yaw_cmd = clamp(man_err_turns * man_kp, -1, 1)

        -- optional smooth step clamp (keeps same stability behavior as normal path)
        last_yaw_cmd = state[92] or 0
        yaw_cmd = clamp(yaw_cmd, last_yaw_cmd - YAW_CMD_MAX_STEP, last_yaw_cmd + YAW_CMD_MAX_STEP)
        state[92] = yaw_cmd

        -- Manual pitch (position command)
        pitch_cmd = clamp(manual_ud * prop_manual_pitch_speed + recoil_pitch_bias, prop_min_pitch, prop_max_pitch)
        state[93] = pitch_cmd

        -- Clear aim/track filters (they depend on valid target)
        state[950] = nil
        state[830] = nil
        state[831] = nil
        state[840] = nil
        state[901] = nil

        output_bool(1, true)
        output_number(1, 0); output_number(2, 0); output_number(3, 0)
        output_number(4, remove_nan(yaw_cmd))
        output_number(5, remove_nan(pitch_cmd))
        return
    end

    ------------------------
    -- Auto-aim edge detect
    ------------------------
    auto_prev = state[260] == true
    state[260] = auto_aim
    auto_just_on = auto_aim and (not auto_prev)

    ------------------------
    -- Initialize state on reacquire
    ------------------------
    was_valid = state[201] == true
    state[201] = true

    if not was_valid then
        state[92]  = state[92] or 0
        state[93]  = state[93] or clamp(0, prop_min_pitch, prop_max_pitch)
        state[261] = 0
        state[271] = input_number(12)
        state[501] = input_number(12)
        state[502] = auto_aim
        state[611] = 0
        state[612] = 0
        state[701] = state[93]
        state[810] = 1; state[811] = vec3(0,0,0)
        state[901] = nil
        state[950] = nil
        state[830] = nil
        state[831] = nil
        state[840] = nil
    end

    if auto_just_on then
        state[261] = 0
        state[271] = input_number(12)
        state[950] = nil
    end

    state[261] = m_min((state[261] or 0) + 1, 60)
    aim_ramp = clamp((state[261] or 0) / 6, 0, 1)

    if (state[502] == true) ~= auto_aim then
        state[501] = input_number(12)
        state[502] = auto_aim
    end

    ------------------------
    -- Target kinematics estimation
    --
    -- target_vel_meas comes from tracker position deltas (held/decayed if tracker stalls).
    -- target_vel is a low-pass filtered version with separate ground/air alphas.
    -- target_acc is delta(target_vel) and optionally corrected by curvature (air only).
    ------------------------
    target_vel_meas = vel_held(target_pos_raw, 1, 810, 811, 1e-7)

    rel_speed = vec_len(vec_sub(target_vel_meas, gun_vel))
    is_air = (rel_speed > AIR_SPEED_TICK)

    tv_prev = state[830] or target_vel_meas
    tv_alpha = is_air and TARGET_VEL_ALPHA_AIR or TARGET_VEL_ALPHA_GROUND
    target_vel = {
        x = tv_prev.x*(1-tv_alpha) + target_vel_meas.x*tv_alpha,
        y = tv_prev.y*(1-tv_alpha) + target_vel_meas.y*tv_alpha,
        z = tv_prev.z*(1-tv_alpha) + target_vel_meas.z*tv_alpha
    }
    state[830] = target_vel

    -- Effective control delay: base tick delay, scaled down for air if enabled (to reduce overlead oscillations)
    delay_eff = prop_tick_delay * (is_air and (prop_adv_enable and prop_air_delay_scale or 1.0) or 1.0)

    -- IMPORTANT (double-lead fix):
    -- Ballistic solver uses *raw* target position; no pre-advance by delay_eff.
    target_pos = target_pos_raw

    -- Gun pose projected forward by control tick delay (compensates sensor->actuator latency)
    gun_pos_d  = vec_add(gun_pos, vec_scale(gun_vel, prop_tick_delay))

    target_acc = vec_delta(target_vel, 5)
    gun_acc    = vec_delta(gun_vel,    6)

    -- Trust scaling for acceleration: if tracker velocity changes are small/noisy, reduce influence of target_acc
    dv_prev_meas = state[831] or target_vel_meas
    dv = vec_sub(target_vel_meas, dv_prev_meas)
    dv_mag = vec_len(dv)
    acc_trust = clamp(dv_mag / m_max(ACC_TRUST_DV_TICK, 1e-6), ACC_TRUST_MIN, 1.0)
    state[831] = target_vel_meas

    if is_air and prop_adv_enable then
        -- Estimate turn-rate omega from change in velocity direction:
        -- omega ≈ (v_prev × v_now) / |v|^2   (per tick)
        v2 = vec_dot(target_vel, target_vel)
        omega_turn = vec_scale(vec_cross(tv_prev, target_vel), 1 / m_max(v2, 1e-9))

        -- Cap omega magnitude to prevent spikes
        omega_turn_mag = vec_len(omega_turn)
        if omega_turn_mag > AIR_TURN_CAP then
            omega_turn = vec_scale(omega_turn, AIR_TURN_CAP / m_max(omega_turn_mag, 1e-9))
        end

        -- Low-pass omega
        ot_prev = state[840] or omega_turn
        omega_turn = {
            x = ot_prev.x*(1-AIR_TURN_ALPHA) + omega_turn.x*AIR_TURN_ALPHA,
            y = ot_prev.y*(1-AIR_TURN_ALPHA) + omega_turn.y*AIR_TURN_ALPHA,
            z = ot_prev.z*(1-AIR_TURN_ALPHA) + omega_turn.z*AIR_TURN_ALPHA
        }
        state[840] = omega_turn

        -- Curvature accel: a_curv = omega × v
        target_acc_curv = vec_cross(omega_turn, target_vel)

        -- Blend measured acceleration toward curvature model
        target_acc = {
            x = target_acc.x*(1-AIR_CURV_BLEND) + target_acc_curv.x*AIR_CURV_BLEND,
            y = target_acc.y*(1-AIR_CURV_BLEND) + target_acc_curv.y*AIR_CURV_BLEND,
            z = target_acc.z*(1-AIR_CURV_BLEND) + target_acc_curv.z*AIR_CURV_BLEND
        }
    else
        state[840] = nil
    end

    state[282] = target_vel
    state[283] = target_acc
    state[281] = target_pos_raw

    -- Relative acceleration used for interception model: (target - gun), scaled by trust and capped
    rel_acc_raw = vec_sub(target_acc, gun_acc)
    rel_acc_raw = vec_scale(rel_acc_raw, acc_trust)

    rel_acc_mag = vec_len(rel_acc_raw)
    rel_acc = (rel_acc_mag > REL_ACCEL_CAP and vec_scale(rel_acc_raw, REL_ACCEL_CAP/rel_acc_mag) or rel_acc_raw)

    ------------------------
    -- Wind acceleration model
    --
    -- wind_local is horizontal wind in vehicle-local frame (no vertical component).
    -- Convert to world and treat as a small acceleration term scaled by wind_affect.
    ------------------------
    wind_speed_tick = wind_speed_ms / TICKS_PER_SEC
    wind_local = vec3(m_sin(wind_angle)*wind_speed_tick, m_cos(wind_angle)*wind_speed_tick, 0)
    wind_world_app = vec_local_to_world(wind_local, axes_veh)

    wind_acc_world = vec_scale(wind_world_app, -(wind_affect / TICKS_PER_SEC))
    accel_world = {x=wind_acc_world.x, y=wind_acc_world.y, z=-GRAVITY_TICK2}

    ------------------------
    -- TOF solve + aim direction (ballistics)
    ------------------------
    los_world = vec_sub(target_pos, gun_pos_d)
    range_m = vec_len(los_world)
    range_safe = m_max(1e-6, range_m)

    range_scale = clamp(range_safe / 140, 0.35, 1.0)
    tof_no_drag = range_safe / m_max(muzzle_velocity_tick, 1e-6)

    intercept = target_pos
    muzzle_vec = {x=0,y=1,z=0}
    tof = 0
    ok_tof = true

    if range_m > CLOSE_RANGE_HARD_M then
        tof, intercept, muzzle_vec, ok_tof =
            solve_tof(target_pos, target_vel, rel_acc, gun_pos_d, gun_vel, muzzle_velocity_tick, D, one_minus_D, accel_world, life_span_ticks)

        if not ok_tof then
            -- Fallback: use no-drag TOF guess but still compute discrete-drag v0 once
            tof = clamp(tof_no_drag, 0.10, life_span_ticks)
            intercept = {
                x = target_pos.x + target_vel.x*tof + 0.5*rel_acc.x*tof*tof,
                y = target_pos.y + target_vel.y*tof + 0.5*rel_acc.y*tof*tof,
                z = target_pos.z + target_vel.z*tof + 0.5*rel_acc.z*tof*tof
            }
            disp = {x=intercept.x-gun_pos_d.x, y=intercept.y-gun_pos_d.y, z=intercept.z-gun_pos_d.z}
            v0_total = {
                x = inv_v0_discrete(disp.x, accel_world.x, D, tof, one_minus_D),
                y = inv_v0_discrete(disp.y, accel_world.y, D, tof, one_minus_D),
                z = inv_v0_discrete(disp.z, accel_world.z, D, tof, one_minus_D)
            }
            muzzle_vec = {x=v0_total.x-gun_vel.x, y=v0_total.y-gun_vel.y, z=v0_total.z-gun_vel.z}
        end
    else
        -- Very close: aim directly at target direction
        intercept = target_pos
        muzzle_vec = vec_scale(vec_norm(vec_sub(target_pos, gun_pos_d)), muzzle_velocity_tick)
        tof = clamp(tof_no_drag, 0.01, 2)
    end

    out_of_range = (range_m > 5000) or (tof_no_drag > life_span_ticks) or (tof ~= tof) or (tof <= 0) or (tof > life_span_ticks)

    -- Aim direction: prefer ballistic muzzle direction if solved, else geometric LOS
    if range_m > CLOSE_RANGE_HARD_M and (m_abs(tof) > 0) then
        aim_dir_world = vec_norm(muzzle_vec)
        if vec_len(aim_dir_world) < 1e-6 then
            aim_dir_world = vec_norm(vec_sub(intercept, gun_pos_d))
        end
    else
        aim_dir_world = vec_norm(vec_sub(intercept, gun_pos_d))
    end

    -- Range-scheduled low-pass on aim direction:
    -- Close: alpha ~ 1 (responsive)
    -- Far:   alpha decreases toward AIMDIR_LP_MIN_ALPHA (stability against noise)
    lp_blend = clamp((range_m - AIMDIR_LP_START_M) / m_max((AIMDIR_LP_END_M - AIMDIR_LP_START_M), 1), 0, 1)
    aim_alpha = 1 - lp_blend * (1 - AIMDIR_LP_MIN_ALPHA)

    aim_prev = state[950] or aim_dir_world
    aim_dir_ctrl = vec_norm({
        x = aim_prev.x*(1-aim_alpha) + aim_dir_world.x*aim_alpha,
        y = aim_prev.y*(1-aim_alpha) + aim_dir_world.y*aim_alpha,
        z = aim_prev.z*(1-aim_alpha) + aim_dir_world.z*aim_alpha
    })
    state[950] = aim_dir_ctrl

    ------------------------
    -- Control-side lookahead (for LOS-rate feedforward ONLY)
    --
    -- delay_ctrl clamps ground lookahead by bullet TOF to prevent overlead on slow movers.
    ------------------------
    delay_ctrl = is_air and delay_eff or m_min(delay_eff, tof)
    target_pos_ctrl = vec_add(target_pos_raw, vec_scale(target_vel, delay_ctrl))

    ------------------------
    -- YAW CONTROL (velocity pivot)
    --
    -- We estimate LOS angular velocity omega from:
    --   omega ≈ (r × v_rel) / |r|^2
    -- where:
    --   r      = line-of-sight vector (vehicle-local)
    --   v_rel  = relative velocity (vehicle-local)
    --
    -- omega.z corresponds to yaw rate around the vertical axis in the vehicle frame.
    -- Feedforward uses this yaw rate normalized by actuator capacity.
    --
    -- Feedback uses predicted aim direction, converted into gun-local frame to form a yaw error.
    ------------------------
    los_local_veh = vec_world_to_local(vec_sub(target_pos_ctrl, gun_pos_d), axes_veh)
    rel_vel_world = vec_sub(target_vel, gun_vel)
    rel_vel_local = vec_world_to_local(rel_vel_world, axes_veh)

    r2 = los_local_veh.x*los_local_veh.x + los_local_veh.y*los_local_veh.y + los_local_veh.z*los_local_veh.z
    inv_r2 = 1 / m_max(r2, 1e-6)

    omega_vec = vec_scale(vec_cross(los_local_veh, rel_vel_local), inv_r2)
    omega_xy_raw_mag = m_sqrt(omega_vec.x*omega_vec.x + omega_vec.y*omega_vec.y)
    omega_vec.x = clamp(omega_vec.x, -OMEGA_XY_MAX, OMEGA_XY_MAX)
    omega_vec.y = clamp(omega_vec.y, -OMEGA_XY_MAX, OMEGA_XY_MAX)

    omega_world = vec_local_to_world(omega_vec, axes_veh)

    -- Predict where the aim direction will be when actuators "arrive" (simple first-order rotation approximation):
    --   d(aim)/dt ≈ omega × aim
    aim_dir_pred_yaw_world   = vec_norm(vec_add(aim_dir_ctrl, vec_scale(vec_cross(omega_world, aim_dir_ctrl), YAW_ACT_DELAY_TICKS)))
    aim_dir_pred_pitch_world = vec_norm(vec_add(aim_dir_ctrl, vec_scale(vec_cross(omega_world, aim_dir_ctrl), PITCH_ACT_DELAY_TICKS)))

    aim_dir_pred_yaw_gun   = vec_world_to_local(aim_dir_pred_yaw_world, axes_gun)
    aim_dir_pred_pitch_veh = vec_world_to_local(aim_dir_pred_pitch_world, axes_veh)

    -- Feedforward yaw command from LOS yaw rate:
    omega_z = omega_vec.z
    yaw_rate_turns_sec = (omega_z * TICKS_PER_SEC) / TAU
    yaw_ff_cmd = clamp(yaw_rate_turns_sec / m_max(YAW_MAX_TPS, 1e-6), -1, 1)

    -- Smooth feedforward to reduce chatter
    state[611] = (state[611] or 0) * YAW_FF_BLEND_OLD + yaw_ff_cmd * YAW_FF_BLEND_NEW
    yaw_ff_cmd = clamp(state[611], -YAW_FF_MAX, YAW_FF_MAX)

    gun_yaw_turns = input_number(12)
    prev_gun_yaw_turns = state[271] or gun_yaw_turns
    gun_yaw_rate_tick = wrap_turns(gun_yaw_turns - prev_gun_yaw_turns)
    state[271] = gun_yaw_turns

    -- Yaw error = atan2(right, forward) of predicted aim direction in gun-local coordinates
    base_yaw_err = wrap_turns(m_atan(aim_dir_pred_yaw_gun.x, aim_dir_pred_yaw_gun.y) / TAU)
    if m_abs(base_yaw_err) < YAW_ERR_DEADBAND_TURNS then base_yaw_err = 0 end
    yaw_err_turns = wrap_turns(base_yaw_err + recoil_yaw_turns)

    -- Range-scheduled proportional gain:
    --   yaw_tau increases slightly at close range (more damping), decreases at far range (more authority).
    yaw_tau = 0.15 + 0.08*(1 - range_scale)
    yaw_kp  = (clamp(prop_aim_speed, 0.5, 2.0) / m_max(YAW_MAX_TPS*yaw_tau, 1e-6)) * aim_ramp
    yaw_kd  = YAW_RATE_BRAKE_GAIN * aim_ramp

    yaw_cmd_auto = clamp((-(yaw_err_turns * yaw_kp)) + yaw_ff_cmd, -1, 1)

    -- Manual mode: integrate a yaw setpoint using manual_rate_cmd, then P-control to it
    manual_rate_cmd = (-manual_lr * prop_manual_speed)
    state[501] = wrap_turns((state[501] or gun_yaw_turns) + (manual_rate_cmd * YAW_MAX_TPS / TICKS_PER_SEC))

    base_man_err = wrap_turns((state[501] or gun_yaw_turns) - gun_yaw_turns)
    if m_abs(base_man_err) < MAN_ERR_DEADBAND_TURNS and m_abs(manual_rate_cmd) < 0.02 then base_man_err = 0 end
    man_err_turns = wrap_turns(base_man_err + recoil_yaw_turns)

    man_kp = clamp(1 / m_max(YAW_MAX_TPS*0.25, 1e-6), 2, 6)
    yaw_cmd_manual = clamp(man_err_turns * man_kp, -1, 1)

    yaw_cmd = auto_aim and yaw_cmd_auto or yaw_cmd_manual

    -- Step clamp (limits instantaneous change)
    last_yaw_cmd = state[92] or 0
    yaw_cmd = clamp(yaw_cmd, last_yaw_cmd - YAW_CMD_MAX_STEP, last_yaw_cmd + YAW_CMD_MAX_STEP)

    -- Estimate actual yaw rate from commanded rate (simple first-order tracking filter),
    -- then apply a braking term proportional to estimated rate to reduce overshoot.
    yaw_rate_cmd_turns_sec = yaw_cmd * YAW_MAX_TPS
    state[612] = (state[612] or 0) + (yaw_rate_cmd_turns_sec - (state[612] or 0)) * prop_yaw_track
    yaw_rate_est_turns_sec = state[612]

    yaw_brake = (yaw_rate_est_turns_sec / m_max(YAW_MAX_TPS, 1e-6)) * yaw_kd
    yaw_cmd = clamp(yaw_cmd - yaw_brake, -1, 1)

    -- Stop conditions to reduce idle twitch
    if auto_aim and base_yaw_err == 0 and m_abs(yaw_ff_cmd) < 0.001 and m_abs(recoil_yaw_turns) < 1e-6 and m_abs(yaw_cmd) < 0.02 then yaw_cmd = 0 end
    if (not auto_aim) and base_man_err == 0 and m_abs(recoil_yaw_turns) < 1e-6 then yaw_cmd = 0 end

    state[92] = yaw_cmd

    ------------------------
    -- PITCH CONTROL
    --
    -- Pitch aims the predicted direction in vehicle frame:
    --   pitch_rad = atan2(z, sqrt(x^2+y^2))
    -- The turret pitch output expects "turns" with factor *4 (Stormworks turret pitch convention).
    --
    -- Hysteresis prevents chatter: small changes within pitch_hyst are ignored.
    -- Hysteresis shrinks at high angular-rate motion (omega_xy_raw_mag) to stay responsive.
    ------------------------
    horiz = m_sqrt(aim_dir_pred_pitch_veh.x*aim_dir_pred_pitch_veh.x + aim_dir_pred_pitch_veh.y*aim_dir_pred_pitch_veh.y)
    pitch_rad = m_atan(aim_dir_pred_pitch_veh.z, m_max(horiz, 1e-9))
    pitch_turns = (pitch_rad / TAU) * 4
    pitch_turns = remove_nan(pitch_turns)

    base_pitch_cmd =
        auto_aim and clamp(pitch_turns, prop_min_pitch, prop_max_pitch)
        or clamp(manual_ud * prop_manual_pitch_speed, prop_min_pitch, prop_max_pitch)

    pitch_hyst_range = PITCH_HYST_MIN + (PITCH_HYSTERESIS - PITCH_HYST_MIN) * lp_blend
    omega_blend = clamp((omega_xy_raw_mag - PITCH_HYST_OMEGA_START) / m_max((PITCH_HYST_OMEGA_END - PITCH_HYST_OMEGA_START), 1e-6), 0, 1)
    pitch_hyst = pitch_hyst_range*(1-omega_blend) + PITCH_HYST_MIN*omega_blend

    prev_base_pitch_cmd = state[701] or base_pitch_cmd
    if m_abs(base_pitch_cmd - prev_base_pitch_cmd) < pitch_hyst then
        base_pitch_cmd = prev_base_pitch_cmd
    else
        state[701] = base_pitch_cmd
    end

    pitch_cmd = clamp(base_pitch_cmd + recoil_pitch_bias, prop_min_pitch, prop_max_pitch)
    state[93] = pitch_cmd

    ------------------------
    -- Outputs
    ------------------------
    output_bool(1, out_of_range)
    output_number(1, intercept.x)
    output_number(2, intercept.y)
    output_number(3, intercept.z)
    output_number(4, remove_nan(yaw_cmd))
    output_number(5, remove_nan(pitch_cmd))
end
