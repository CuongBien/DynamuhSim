import math
import numpy as np

def apply_p4_wall_filter(rx, ry, ryaw, cmd_vx, cmd_vy, cmd_wz, corridor_half_width=0.60):
    vx, vy, wz = float(cmd_vx), float(cmd_vy), float(cmd_wz)
    
    # 1. Yaw constraint: strictly within +/- 45 deg (+/- 0.7854 rad)
    max_yaw = math.pi / 4.0  # 0.7854 rad (45 deg)
    yaw_margin = 0.10  # start steering back at 39 deg
    
    # If yaw exceeds or approaches 45 deg, force angular velocity back to 0
    if ryaw > (max_yaw - yaw_margin):
        wz = min(wz, -1.0 * (ryaw - (max_yaw - yaw_margin)) / yaw_margin - 0.2)
    elif ryaw < -(max_yaw - yaw_margin):
        wz = max(wz, 1.0 * (-ryaw - (max_yaw - yaw_margin)) / yaw_margin + 0.2)
        
    # Hard clip on yaw predict
    pred_yaw = ryaw + wz * 0.1
    if pred_yaw > max_yaw:
        wz = min(wz, (max_yaw - ryaw) / 0.1)
    elif pred_yaw < -max_yaw:
        wz = max(wz, (-max_yaw - ryaw) / 0.1)
        
    wz = float(np.clip(wz, -1.2, 1.2))
    
    # 2. Footprint corners check
    l, w = 0.12, 0.10
    body_corners = [(-l, -w), (-l, w), (l, -w), (l, w)]
    max_safe_y = corridor_half_width - 0.05  # 0.55m max corner (5cm clearance)
    
    # Check predictions for multiple horizons
    horizons = [0.05, 0.10, 0.20, 0.35]
    
    for _ in range(5):
        violates_left = False
        violates_right = False
        
        for dt in horizons:
            yp = ryaw + wz * dt
            # World frame velocities
            vx_w = vx * math.cos(ryaw) - vy * math.sin(ryaw)
            vy_w = vx * math.sin(ryaw) + vy * math.cos(ryaw)
            y_pred = ry + vy_w * dt
            
            cos_yp = math.cos(yp)
            sin_yp = math.sin(yp)
            
            for cx, cy in body_corners:
                cp_y = y_pred + cx * sin_yp + cy * cos_yp
                if cp_y > max_safe_y:
                    violates_left = True
                if cp_y < -max_safe_y:
                    violates_right = True
                    
        if violates_left or ry > 0.35:
            # Pushing into left wall:
            # Clamping vx based on global yaw: if yaw > 0, forward vx pushes left!
            if ryaw > 0.05:
                vx = min(vx, 0.05)  # clamp forward drive towards wall
                wz = min(wz, -0.8)   # steer right (clockwise, yaw -> 0)
            else:
                wz = min(wz, -0.3)
            # Active lateral push back to center
            # We want vy_w <= -0.15 m/s -> vx*sin(yaw) + vy*cos(yaw) <= -0.15
            # In body frame:
            vy = min(vy, -0.20)
            # If robot is nearly aligned, let it move forward along corridor
            if abs(ryaw) < 0.25 and ry < 0.40:
                vx = max(vx, 0.20)
        elif violates_right or ry < -0.35:
            # Pushing into right wall:
            if ryaw < -0.05:
                vx = min(vx, 0.05)
                wz = max(wz, 0.8)   # steer left (counter-clockwise, yaw -> 0)
            else:
                wz = max(wz, 0.3)
            vy = max(vy, 0.20)
            if abs(ryaw) < 0.25 and ry > -0.40:
                vx = max(vx, 0.20)
        else:
            break
            
    vx = float(np.clip(vx, -0.10, 0.50))
    vy = float(np.clip(vy, -0.35, 0.35))
    wz = float(np.clip(wz, -1.2, 1.2))
    
    # Longitudinal boundary limits (corridor bounds)
    if rx >= -6.3 and vx > 0:
        vx = 0.0
    elif rx <= -14.2 and vx < 0:
        vx = 0.0
        
    return vx, vy, wz

# Simulate 50 episodes of 40s (800 steps each = 40,000 steps)
rng = np.random.RandomState(42)
wall_contacts = 0
freezes = 0
total_episodes = 50

for ep in range(total_episodes):
    rx = -13.0
    ry = 0.0
    ryaw = 0.0
    dt = 0.05
    
    current_cmd = (0.25, 0.0, 0.0)
    next_resample = 0.0
    
    ep_wall_contact = False
    ep_frozen_dur = 0.0
    ep_frozen = False
    
    l, w = 0.12, 0.10
    corners = [(-l, -w), (-l, w), (l, -w), (l, w)]
    
    for t_step in range(800):
        t = t_step * dt
        if t >= next_resample:
            next_resample = t + rng.uniform(0.5, 1.5)
            vx_cmd = rng.uniform(-0.10, 0.50)
            vy_cmd = rng.uniform(-0.30, 0.30)
            wz_cmd = rng.uniform(-1.00, 1.00)
            current_cmd = (vx_cmd, vy_cmd, wz_cmd)
            
        vx_c, vy_c, wz_c = current_cmd
        vx, vy, wz = apply_p4_wall_filter(rx, ry, ryaw, vx_c, vy_c, wz_c)
        
        # Integrate motion
        ryaw += wz * dt
        vx_w = vx * math.cos(ryaw) - vy * math.sin(ryaw)
        vy_w = vx * math.sin(ryaw) + vy * math.cos(ryaw)
        rx += vx_w * dt
        ry += vy_w * dt
        
        # Check wall contact: corner |y| >= 0.60m
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)
        for cx, cy in corners:
            cp_y = ry + cx * sin_y + cy * cos_y
            if abs(cp_y) >= 0.60:
                ep_wall_contact = True
                
        # Check freeze: v < 0.05 for >= 3.0s (60 steps)
        spd = math.hypot(vx, vy)
        if spd < 0.05:
            ep_frozen_dur += dt
            if ep_frozen_dur >= 3.0:
                ep_frozen = True
        else:
            ep_frozen_dur = 0.0
            
    if ep_wall_contact:
        wall_contacts += 1
    if ep_frozen:
        freezes += 1

print(f"Total episodes: {total_episodes}")
print(f"Wall contacts: {wall_contacts} ({wall_contacts/total_episodes*100:.1f}%)")
print(f"Freezes: {freezes} ({freezes/total_episodes*100:.1f}%)")
