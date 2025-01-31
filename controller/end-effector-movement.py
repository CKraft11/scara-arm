def calculate_motor_angles(target_height_mm, target_rotation_deg):
    """
    Calculate required motor angles for a ball spline SCARA arm.
    
    Args:
        target_height_mm (float): Desired change in height (mm), positive is up
        target_rotation_deg (float): Desired rotation (degrees), positive is clockwise
    
    Returns:
        tuple: (motor3_angle_deg, motor4_angle_deg)
    """
    # Constants
    SCREW_PITCH = 15  # mm per revolution
    GEAR_RATIO = 3    # motor revolutions per output revolution
    
    # For pure height change (no rotation):
    # - Motor 3 rotates X degrees
    # - Motor 4 stays still
    # 3 motor revolutions (1080°) = 15mm height change
    height_motor3_angle = (target_height_mm / SCREW_PITCH) * 1080
    
    # For pure rotation (no height change):
    # - Both motors need to rotate the same amount
    # - 3 motor revolutions (1080°) = 360° output rotation
    rotation_angle = (target_rotation_deg / 360) * 1080
    
    # Combine the movements
    motor3_angle = height_motor3_angle + rotation_angle
    motor4_angle = rotation_angle
    
    return (motor3_angle, motor4_angle)

# Test case 1: Pure rotation (360° clockwise, no height change)
angles = calculate_motor_angles(0, 360)
print(f"Motor 3: {angles[0]}°, Motor 4: {angles[1]}°")
# Should output: Motor 3: 1080°, Motor 4: 1080°
# Because both motors need to rotate 3 full turns (1080°) for one full output rotation

# Test case 2: Pure height change (15mm up, no rotation)
angles = calculate_motor_angles(15, 0)
print(f"Motor 3: {angles[0]}°, Motor 4: {angles[1]}°")
# Should output: Motor 3: 1080°, Motor 4: 0°
# Because motor 3 needs 3 turns for 15mm height change, motor 4 stays still

# Test case 3: Combined motion (15mm up and 360° rotation)
angles = calculate_motor_angles(15, 360)
print(f"Motor 3: {angles[0]}°, Motor 4: {angles[1]}°")
# Should output: Motor 3: 2160°, Motor 4: 1080°
# Because we combine the pure height motion with the pure rotation motion